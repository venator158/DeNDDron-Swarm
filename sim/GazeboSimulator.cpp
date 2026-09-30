#include "GazeboSimulator.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cctype>
#include <fstream>
#include <cstdlib>
#include <cstring>

GazeboSimulator::GazeboSimulator() {
    std::cout << "[GazeboSimulator] Initializing..." << std::endl;
}

GazeboSimulator::~GazeboSimulator() {
    disconnect();
}

void GazeboSimulator::init() {
    std::cout << "[GazeboSimulator] Connecting to Gazebo..." << std::endl;

    try {
        gazebo::client::setup();
    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Failed to setup Gazebo client: " << e.what() << std::endl;
        return;
    }

    _gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _gznode->Init();

    _factory_pub = _gznode->Advertise<gazebo::msgs::Factory>("~/factory");
    _factory_pub->WaitForConnection();

    _physics_pub = _gznode->Advertise<gazebo::msgs::Model>("~/model/modify");

    // One connected publisher for deletions: transport::requestNoReply() advertises a new
    // publisher per call and its first message is lost before the connection is up.
    _request_pub = _gznode->Advertise<gazebo::msgs::Request>("~/request");
    _request_pub->WaitForConnection();
    
    _stats_sub = _gznode->Subscribe("~/world_stats", &GazeboSimulator::on_world_stats, this);

    std::cout << "[GazeboSimulator] Connected to Gazebo transport" << std::endl;

    if (const char* rtf_env = std::getenv("SIM_RTF"); rtf_env != nullptr && std::strlen(rtf_env) > 0) {
        const double target_rtf = std::clamp(std::atof(rtf_env), 0.1, 10.0);
        _tick_period = std::chrono::microseconds(static_cast<long>(20000.0 / target_rtf));
        std::cout << "[GazeboSimulator] Target real-time factor " << target_rtf << ", tick "
                  << _tick_period.count() << " us" << std::endl;
    }

    // Zenoh Init — connect as a client to the router (same fabric as Python agents).
    // Without this the bridge opens a peer/multicast session that is isolated from
    // the client sessions the Python agents use, so sensor publishes never reach them.
    auto config = zenoh::Config::create_default();
    const char* router_env = std::getenv("ZENOH_ROUTER_IP");
    if (router_env != nullptr && std::strlen(router_env) > 0) {
        std::string router_str(router_env);
        std::string endpoints = "[\"" + router_str + "\"]";
        config.insert_json5("mode", "\"client\"");
        config.insert_json5("connect/endpoints", endpoints);
        std::cout << "[GazeboSimulator] Zenoh: connecting to router at " << router_str << std::endl;
    } else {
        std::cout << "[GazeboSimulator] WARNING: ZENOH_ROUTER_IP not set — using peer/multicast scouting" << std::endl;
    }
    _session.emplace(zenoh::Session::open(std::move(config)));
    std::cout << "[GazeboSimulator] Zenoh session opened." << std::endl;

    std::cout << "[Gazebo] Registering subscribers..." << std::endl;
    auto sub_opt = zenoh::Session::SubscriberOptions::create_default();
    
    _sub_agent_join.emplace(_session->declare_subscriber(
        zenoh::KeyExpr("swarm/agents/join"),
        std::bind(&GazeboSimulator::on_agent_join, this, std::placeholders::_1),
        [](){},
        std::move(sub_opt)
    ));

    auto sub_opt_cmd = zenoh::Session::SubscriberOptions::create_default();
    _sub_cmd_vel.emplace(_session->declare_subscriber(
        zenoh::KeyExpr("swarm/*/cmd_vel"),
        std::bind(&GazeboSimulator::on_cmd_vel, this, std::placeholders::_1),
        [](){},
        std::move(sub_opt_cmd)
    ));

    auto sub_opt_tracks = zenoh::Session::SubscriberOptions::create_default();
    _sub_threat_tracks.emplace(_session->declare_subscriber(
        zenoh::KeyExpr("sim/threat_tracks"),
        std::bind(&GazeboSimulator::on_threat_track, this, std::placeholders::_1),
        [](){},
        std::move(sub_opt_tracks)
    ));

    auto pub_opt_clock = zenoh::Session::PublisherOptions::create_default();
    _pub_clock.emplace(_session->declare_publisher(zenoh::KeyExpr("sim/clock"), std::move(pub_opt_clock)));

    auto sub_opt_despawn = zenoh::Session::SubscriberOptions::create_default();
    _sub_agent_despawn.emplace(_session->declare_subscriber(
        zenoh::KeyExpr("swarm/agents/despawn"),
        std::bind(&GazeboSimulator::on_agent_despawn, this, std::placeholders::_1),
        [](){},
        std::move(sub_opt_despawn)
    ));

    load_spawn_config();
    configure_fuze();

    std::cout << "[GazeboSimulator] Ready. Waiting for agents..." << std::endl;
}

void GazeboSimulator::load_spawn_config() {
    const char* env_path = std::getenv("SWARM_RUNTIME_CONFIG");
    _spawn_config_path = env_path != nullptr ? env_path : "/home/app/config/swarm_runtime.json";

    std::ifstream config_file(_spawn_config_path);
    if (!config_file.is_open()) {
        std::cout << "[GazeboSimulator] No runtime spawn config found at "
                  << _spawn_config_path
                  << ". Falling back to default placement." << std::endl;
        return;
    }

    try {
        json cfg = json::parse(config_file);
        if (!cfg.contains("agents") || !cfg["agents"].is_object()) {
            std::cerr << "[GazeboSimulator] Runtime config missing 'agents' object." << std::endl;
            return;
        }

        _spawn_config.clear();
        _goal_config.clear();
        for (auto it = cfg["agents"].begin(); it != cfg["agents"].end(); ++it) {
            const std::string agent_id = it.key();
            const json& agent_cfg = it.value();
            if (agent_cfg.contains("spawn") && agent_cfg["spawn"].is_object()) {
                const json& spawn = agent_cfg["spawn"];
                _spawn_config[agent_id] = SpawnPoint{
                    spawn.value("x", 0.0),
                    spawn.value("y", 0.0),
                    spawn.value("z", 1.0)
                };
            }
            if (agent_cfg.contains("goal") && agent_cfg["goal"].is_object()) {
                const json& goal = agent_cfg["goal"];
                _goal_config[agent_id] = SpawnPoint{
                    goal.value("x", 0.0),
                    goal.value("y", 0.0),
                    goal.value("z", 1.0)
                };
            }
        }

        std::cout << "[GazeboSimulator] Loaded spawn config for "
                  << _spawn_config.size() << " agents from "
                  << _spawn_config_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Failed to parse runtime spawn config: "
                  << e.what() << std::endl;
    }
}

void GazeboSimulator::disconnect() {
    _session.reset();
    try {
        gazebo::client::shutdown();
    } catch (...) {}
}

void GazeboSimulator::on_agent_join(const zenoh::Sample& sample) {
    try {
        std::string payload = sample.get_payload().as_string();
        auto join_msg = json::parse(payload);
        std::string agent_id = join_msg["agent_id"];

        std::cout << "[GazeboSimulator] Received join event from: " << agent_id << std::endl;

        json initial_pos = {{"x", 0.0}, {"y", 0.0}, {"z", 1.0}};
        spawn_drone(agent_id, initial_pos);

    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Error processing agent join: " << e.what() << std::endl;
    }
}

void GazeboSimulator::on_agent_despawn(const zenoh::Sample& sample) {
    try {
        auto msg = json::parse(sample.get_payload().as_string());
        std::string agent_id = msg["agent_id"];
        std::string reason = msg.value("reason", "unknown");
        std::cout << "[GazeboSimulator] Despawn requested for " << agent_id
                  << " (reason: " << reason << ")" << std::endl;
        std::lock_guard<std::mutex> lock(_state_mtx);
        _pending_despawns.push_back(agent_id);
    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Error processing agent despawn: " << e.what() << std::endl;
    }
}

void GazeboSimulator::apply_pending_despawns() {
    std::vector<std::string> ids;
    std::vector<std::string> models;
    {
        std::lock_guard<std::mutex> lock(_state_mtx);
        ids.swap(_pending_despawns);
        models.swap(_pending_model_deletes);
        for (const auto& id : ids) {
            _spawned_agents[id] = false;
            _drone_states.erase(id);
        }
    }
    for (const auto& id : ids) {
        if (_gznode) {
            delete_model(id);
            delete_model(id + "_goal");
        }
        std::cout << "[GazeboSimulator] Despawned drone: " << id << std::endl;
    }
    for (const auto& name : models) {
        if (_gznode) {
            delete_model(name);
        }
    }
}

void GazeboSimulator::delete_model(const std::string& name) {
    if (!_request_pub) return;
    gazebo::msgs::Request* req = gazebo::msgs::CreateRequest("entity_delete", name);
    _request_pub->Publish(*req);
    delete req;
}

void GazeboSimulator::on_threat_track(const zenoh::Sample& sample) {
    // {threat_id, type, level, status, t0, p0{x,y,z}, v{x,y,z}} from the ship's radar picture
    try {
        auto msg = json::parse(sample.get_payload().as_string());
        const std::string threat_id = msg["threat_id"];
        const std::string status = msg.value("status", "active");
        const std::string model = "threat_" + threat_id;

        if (status != "active") {
            std::lock_guard<std::mutex> lock(_state_mtx);
            if (_threat_markers.erase(threat_id) > 0) {
                _pending_model_deletes.push_back(model);
                std::cout << "[GazeboSimulator] Threat " << threat_id << " " << status << std::endl;
            }
            return;
        }

        const auto& p0 = msg["p0"];
        const auto& v = msg["v"];
        ThreatMarker marker{
            ignition::math::Vector3d(p0["x"].get<double>(), p0["y"].get<double>(), p0["z"].get<double>()),
            ignition::math::Vector3d(v["x"].get<double>(), v["y"].get<double>(), v["z"].get<double>()),
            msg["t0"].get<double>()
        };
        bool is_new = false;
        {
            std::lock_guard<std::mutex> lock(_state_mtx);
            is_new = _threat_markers.find(threat_id) == _threat_markers.end();
            _threat_markers[threat_id] = marker;
        }
        if (is_new && _factory_pub) {
            gazebo::msgs::Factory factory_msg;
            factory_msg.set_sdf(generate_threat_sdf(threat_id, msg.value("type", "unknown"), msg.value("level", 1),
                                                    marker.p0.X(), marker.p0.Y(), marker.p0.Z()));
            _factory_pub->Publish(factory_msg);
            std::cout << "[GazeboSimulator] Threat marker: " << threat_id << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Error processing threat track: " << e.what() << std::endl;
    }
}

void GazeboSimulator::move_threat_markers(double sim_time) {
    if (!_physics_pub) return;
    struct MarkerPose { std::string name; ignition::math::Vector3d p; double yaw; };
    std::vector<MarkerPose> poses;
    {
        std::lock_guard<std::mutex> lock(_state_mtx);
        for (const auto& [id, m] : _threat_markers) {
            poses.push_back({"threat_" + id, m.p0 + m.v * (sim_time - m.t0), std::atan2(m.v.Y(), m.v.X())});
        }
    }
    for (const auto& mp : poses) {
        gazebo::msgs::Model msg;
        msg.set_name(mp.name);
        gazebo::msgs::Vector3d* pos = msg.mutable_pose()->mutable_position();
        pos->set_x(mp.p.X());
        pos->set_y(mp.p.Y());
        pos->set_z(mp.p.Z());
        // Nose (+x of the model) along the direction of flight.
        gazebo::msgs::Quaternion* rot = msg.mutable_pose()->mutable_orientation();
        rot->set_x(0.0);
        rot->set_y(0.0);
        rot->set_z(std::sin(mp.yaw / 2.0));
        rot->set_w(std::cos(mp.yaw / 2.0));
        _physics_pub->Publish(msg);
    }
}

namespace {
// One SDF <visual>: geometry XML, pose "x y z roll pitch yaw", colour, optional glow/transparency.
std::string sdf_visual(const std::string& name, const std::string& geometry, const std::string& pose,
                       double r, double g, double b, double glow = 0.0, double transparency = 0.0) {
    std::stringstream v;
    v << "<visual name='" << name << "'><pose>" << pose << "</pose><geometry>" << geometry << "</geometry>"
      << "<material><ambient>" << r << " " << g << " " << b << " 1</ambient>"
      << "<diffuse>" << r << " " << g << " " << b << " 1</diffuse>"
      << "<emissive>" << r * glow << " " << g * glow << " " << b * glow << " 1</emissive></material>";
    if (transparency > 0.0) v << "<transparency>" << transparency << "</transparency>";
    v << "</visual>";
    return v.str();
}
std::string box(double x, double y, double z) {
    std::stringstream g;
    g << "<box><size>" << x << " " << y << " " << z << "</size></box>";
    return g.str();
}
std::string cyl(double r, double l) {
    std::stringstream g;
    g << "<cylinder><radius>" << r << "</radius><length>" << l << "</length></cylinder>";
    return g.str();
}
std::string sph(double r) {
    std::stringstream g;
    g << "<sphere><radius>" << r << "</radius></sphere>";
    return g.str();
}
// SDF cylinders are built along z; appending this to a pose lays one along x.
const std::string ALONG_X = " 0 1.5708 0";
}  // namespace

std::string GazeboSimulator::generate_threat_sdf(const std::string& threat_id, const std::string& type,
                                                 int level, double x, double y, double z) {
    // Visual-only primitives (no collision, no physics cost), drawn ~1.5x real size so they stay
    // visible at 200 m. Nose points along +x; move_threat_markers yaws the model along its track.
    // The link is kinematic, not static: Gazebo ignores ~/model/modify pose updates on static models.
    std::stringstream v;
    if (type == "uav") {                       // fixed-wing drone, yellow
        const double r = 1.0, g = 0.85, b = 0.15;
        v << sdf_visual("fuselage", cyl(0.18, 2.2), "0 0 0" + ALONG_X, r, g, b, 0.4)
          << sdf_visual("wing", box(0.5, 3.2, 0.06), "0.2 0 0 0 0 0", r, g, b, 0.4)
          << sdf_visual("tailplane", box(0.3, 1.0, 0.05), "-1.0 0 0 0 0 0", r, g, b, 0.4)
          << sdf_visual("fin", box(0.3, 0.05, 0.5), "-1.0 0 0.25 0 0 0", r, g, b, 0.4)
          << sdf_visual("prop", cyl(0.35, 0.04), "1.15 0 0" + ALONG_X, 0.1, 0.1, 0.1, 0.0, 0.3);
    } else if (type == "missile") {            // finned missile, orange
        const double r = 1.0, g = 0.5, b = 0.1;
        v << sdf_visual("body", cyl(0.22, 3.4), "0 0 0" + ALONG_X, r, g, b, 0.4)
          << sdf_visual("nose", sph(0.22), "1.7 0 0 0 0 0", 0.9, 0.9, 0.9, 0.2)
          << sdf_visual("fins_v", box(0.5, 0.04, 0.9), "-1.45 0 0 0 0 0", r, g, b, 0.4)
          << sdf_visual("fins_h", box(0.5, 0.9, 0.04), "-1.45 0 0 0 0 0", r, g, b, 0.4)
          << sdf_visual("exhaust", sph(0.25), "-1.85 0 0 0 0 0", 1.0, 0.9, 0.3, 1.0, 0.2);
    } else {                                   // cruise missile (or unknown type), red, longer, winged
        const double r = 0.95, g = 0.15, b = 0.1;
        v << sdf_visual("body", cyl(0.28, 5.2), "0 0 0" + ALONG_X, r, g, b, 0.4)
          << sdf_visual("nose", sph(0.28), "2.6 0 0 0 0 0", 0.9, 0.9, 0.9, 0.2)
          << sdf_visual("wings", box(0.6, 2.6, 0.05), "0.2 0 0 0 0 0", r, g, b, 0.4)
          << sdf_visual("fins_v", box(0.5, 0.04, 1.0), "-2.35 0 0 0 0 0", r, g, b, 0.4)
          << sdf_visual("fins_h", box(0.5, 1.0, 0.04), "-2.35 0 0 0 0 0", r, g, b, 0.4)
          << sdf_visual("exhaust", sph(0.3), "-2.75 0 0 0 0 0", 1.0, 0.9, 0.3, 1.0, 0.2);
    }
    (void)level;   // the type already encodes the level; shape and colour identify it

    std::stringstream sdf;
    sdf << "<?xml version='1.0'?><sdf version='1.6'>"
        << "<model name='threat_" << threat_id << "'>"
        << "<pose>" << x << " " << y << " " << z << " 0 0 0</pose>"
        << "<link name='base'><kinematic>1</kinematic><gravity>0</gravity>"
        << v.str()
        << "</link></model></sdf>";
    return sdf.str();
}

void GazeboSimulator::on_cmd_vel(const zenoh::Sample& sample) {
    std::string key = std::string(sample.get_keyexpr().as_string_view());
    std::string payload = sample.get_payload().as_string();
    
    size_t first_slash = key.find('/');
    size_t second_slash = key.find('/', first_slash + 1);
    if(first_slash != std::string::npos && second_slash != std::string::npos) {
        std::string agent_id = key.substr(first_slash + 1, second_slash - first_slash - 1);
        try {
            auto cmd_msg = json::parse(payload);
            update_drone_velocity(agent_id, cmd_msg);

            std::lock_guard<std::mutex> lock(_state_mtx);
            if (!_seen_cmd_vel[agent_id]) {
                _seen_cmd_vel[agent_id] = true;
                std::cout << "[GazeboSimulator] Receiving cmd_vel for " << agent_id << std::endl;
            }
        } catch(...) {}
    }
}

std::string GazeboSimulator::generate_drone_sdf(const std::string& agent_id, double x, double y, double z) {
    std::stringstream sdf;
    
    struct Color { double r, g, b; };
    std::vector<Color> palette = {
        {1.0, 0.2, 0.2}, // Vivid Red
        {0.2, 1.0, 0.2}, // Vivid Green
        {0.2, 0.5, 1.0}, // Vivid Blue
        {1.0, 1.0, 0.2}, // Yellow
        {0.2, 1.0, 1.0}, // Cyan
        {1.0, 0.2, 1.0}, // Magenta
        {1.0, 0.6, 0.1}, // Orange
        {0.6, 0.2, 1.0}, // Purple
        {0.0, 1.0, 0.6}  // Spring Green
    };
    
    int color_idx = 0;
    int trailing_num = -1;
    int power = 1;
    for (int i = static_cast<int>(agent_id.size()) - 1; i >= 0; --i) {
        if (std::isdigit(agent_id[i])) {
            if (trailing_num == -1) trailing_num = 0;
            trailing_num += (agent_id[i] - '0') * power;
            power *= 10;
        } else if (trailing_num != -1) {
            break;
        }
    }
    
    if (trailing_num > 0) {
        color_idx = (trailing_num - 1) % palette.size();
    } else {
        color_idx = std::hash<std::string>{}(agent_id) % palette.size();
    }
    
    double r = palette[color_idx].r;
    double g = palette[color_idx].g;
    double b = palette[color_idx].b;

    sdf << "<?xml version='1.0'?>"
        << "<sdf version='1.6'>"
        << "  <model name='" << agent_id << "'>"
        << "    <pose>" << x << " " << y << " " << z << " 0 0 0</pose>"
        << "    <link name='base'>"
        << "      <kinematic>1</kinematic>"
        << "      <inertial>"
        << "        <mass>1.0</mass>"
        << "        <inertia>"
        << "          <ixx>0.0083</ixx><ixy>0</ixy><ixz>0</ixz>"
        << "          <iyy>0.0083</iyy><iyz>0</iyz><izz>0.0083</izz>"
        << "        </inertia>"
        << "      </inertial>"
        << "      <collision name='collision'>"
        << "        <geometry><sphere><radius>1.25</radius></sphere></geometry>"
        << "      </collision>"
        // Quadcopter, ~2 m across (drawn larger than a real small drone so it stays visible):
        // body in the swarm colour, two crossed arms, four translucent rotor discs.
        << sdf_visual("body", box(0.7, 0.7, 0.25), "0 0 0 0 0 0", r, g, b, 0.2)
        << sdf_visual("arm_a", box(2.0, 0.1, 0.08), "0 0 0.05 0 0 0.7854", 0.2, 0.2, 0.22)
        << sdf_visual("arm_b", box(2.0, 0.1, 0.08), "0 0 0.05 0 0 -0.7854", 0.2, 0.2, 0.22)
        << sdf_visual("rotor_1", cyl(0.42, 0.03), "0.707 0.707 0.12 0 0 0", 0.05, 0.05, 0.05, 0.0, 0.4)
        << sdf_visual("rotor_2", cyl(0.42, 0.03), "-0.707 0.707 0.12 0 0 0", 0.05, 0.05, 0.05, 0.0, 0.4)
        << sdf_visual("rotor_3", cyl(0.42, 0.03), "-0.707 -0.707 0.12 0 0 0", 0.05, 0.05, 0.05, 0.0, 0.4)
        << sdf_visual("rotor_4", cyl(0.42, 0.03), "0.707 -0.707 0.12 0 0 0", 0.05, 0.05, 0.05, 0.0, 0.4)
        << "    </link>"
        << "  </model>"
        << "</sdf>";
    return sdf.str();
}

std::string GazeboSimulator::generate_goal_sdf(const std::string& agent_id, double x, double y, double z) {
    std::stringstream sdf;
    
    struct Color { double r, g, b; };
    std::vector<Color> palette = {
        {1.0, 0.2, 0.2}, // Vivid Red
        {0.2, 1.0, 0.2}, // Vivid Green
        {0.2, 0.5, 1.0}, // Vivid Blue
        {1.0, 1.0, 0.2}, // Yellow
        {0.2, 1.0, 1.0}, // Cyan
        {1.0, 0.2, 1.0}, // Magenta
        {1.0, 0.6, 0.1}, // Orange
        {0.6, 0.2, 1.0}, // Purple
        {0.0, 1.0, 0.6}  // Spring Green
    };
    
    int color_idx = 0;
    int trailing_num = -1;
    int power = 1;
    for (int i = static_cast<int>(agent_id.size()) - 1; i >= 0; --i) {
        if (std::isdigit(agent_id[i])) {
            if (trailing_num == -1) trailing_num = 0;
            trailing_num += (agent_id[i] - '0') * power;
            power *= 10;
        } else if (trailing_num != -1) {
            break;
        }
    }
    
    if (trailing_num > 0) {
        color_idx = (trailing_num - 1) % palette.size();
    } else {
        color_idx = std::hash<std::string>{}(agent_id) % palette.size();
    }
    
    double r = palette[color_idx].r;
    double g = palette[color_idx].g;
    double b = palette[color_idx].b;

    sdf << "<?xml version='1.0'?>"
        << "<sdf version='1.6'>"
        << "  <model name='" << agent_id << "_goal'>"
        << "    <static>true</static>"
        << "    <pose>" << x << " " << y << " " << z << " 0 0 0</pose>"
        << "    <link name='link'>"
        << "      <visual name='visual'>"
        << "        <geometry><sphere><radius>0.8</radius></sphere></geometry>"
        << "        <material>"
        << "          <ambient>" << r << " " << g << " " << b << " 1.0</ambient>"
        << "          <diffuse>" << r << " " << g << " " << b << " 1.0</diffuse>"
        << "        </material>"
        << "        <transparency>0.3</transparency>"
        << "      </visual>"
        << "    </link>"
        << "  </model>"
        << "</sdf>";
    return sdf.str();
}

void GazeboSimulator::spawn_drone(const std::string& agent_id, const json& initial_pos) {
    std::lock_guard<std::mutex> lock(_state_mtx);
    if (_spawned_agents.find(agent_id) != _spawned_agents.end()) return;

    if (!_factory_pub) return;

    double x = -45.0;
    double y = 0.0;
    double z = 20.0;

    auto cfg_it = _spawn_config.find(agent_id);
    if (cfg_it != _spawn_config.end()) {
        x = cfg_it->second.x;
        y = cfg_it->second.y;
        z = cfg_it->second.z;
    } else {
        // For ids like "drone_7" or "7", spread drones along Y while preserving
        // legacy positions for the first three: -20, 0, +20.
        int trailing_number = -1;
        int power = 1;
        bool found_digit = false;
        for (int i = static_cast<int>(agent_id.size()) - 1; i >= 0; --i) {
            unsigned char c = static_cast<unsigned char>(agent_id[static_cast<size_t>(i)]);
            if (std::isdigit(c)) {
                if (!found_digit) {
                    trailing_number = 0;
                    found_digit = true;
                }
                trailing_number += (agent_id[static_cast<size_t>(i)] - '0') * power;
                power *= 10;
            } else if (found_digit) {
                break;
            }
        }

        if (found_digit && trailing_number > 0) {
            y = (static_cast<double>(trailing_number) - 2.0) * 20.0;
        } else {
            x = initial_pos.value("x", 0.0) + (std::rand() % 10 - 5) * 1.0;
            y = initial_pos.value("y", 0.0) + (std::rand() % 10 - 5) * 1.0;
            z = initial_pos.value("z", 1.0);
        }
    }

    std::string sdf_str = generate_drone_sdf(agent_id, x, y, z);
    gazebo::msgs::Factory factory_msg;
    factory_msg.set_sdf(sdf_str);
    _factory_pub->Publish(factory_msg);

    auto goal_it = _goal_config.find(agent_id);
    if (goal_it != _goal_config.end()) {
        std::string goal_sdf_str = generate_goal_sdf(agent_id, goal_it->second.x, goal_it->second.y, goal_it->second.z);
        gazebo::msgs::Factory goal_factory_msg;
        goal_factory_msg.set_sdf(goal_sdf_str);
        _factory_pub->Publish(goal_factory_msg);
    }

    _spawned_agents[agent_id] = true;
    _drone_states[agent_id] = DroneState{
        ignition::math::Vector3d(x, y, z),
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond()
    };

    std::cout << "[GazeboSimulator] Spawned drone: " << agent_id << " at (" << x << ", " << y << ")" << std::endl;
}

void GazeboSimulator::update_drone_velocity(const std::string& agent_id, const json& cmd_msg) {
    try {
        double vx = cmd_msg["linear"]["x"].get<double>();
        double vy = cmd_msg["linear"]["y"].get<double>();
        double vz = cmd_msg["linear"]["z"].get<double>();
        double yaw_rate = cmd_msg["angular"]["z"].get<double>();

        std::lock_guard<std::mutex> lock(_state_mtx);
        if (_drone_states.find(agent_id) != _drone_states.end()) {
            auto& state = _drone_states[agent_id];

            // Deadband + low-pass filter to reduce cmd jitter from asynchronous updates.
            const double deadband = 0.02;
            if (std::abs(vx) < deadband) vx = 0.0;
            if (std::abs(vy) < deadband) vy = 0.0;
            if (std::abs(vz) < deadband) vz = 0.0;
            if (std::abs(yaw_rate) < deadband) yaw_rate = 0.0;

            const double alpha = 0.35;
            ignition::math::Vector3d cmd_linear(vx, vy, vz);
            state.linear_velocity = state.linear_velocity * (1.0 - alpha) + cmd_linear * alpha;
            state.angular_velocity = ignition::math::Vector3d(0, 0, yaw_rate * alpha + state.angular_velocity.Z() * (1.0 - alpha));
        }
    } catch (...) {}
}

json GazeboSimulator::simulate_lidar(const std::string& agent_id) {
    const int num_rays = 32;
    const double max_range = 50.0;
    const double drone_radius = 3.0;  // Inflated for safer inter-drone separation
    const double ship_radius = 16.0;

    ignition::math::Vector3d agent_pos(0, 0, 0);
    std::vector<ignition::math::Vector3d> dynamic_obstacles;

    {
        std::lock_guard<std::mutex> lock(_state_mtx);
        auto self_it = _drone_states.find(agent_id);
        if (self_it != _drone_states.end()) {
            agent_pos = self_it->second.position;
        }

        for (const auto& [other_id, state] : _drone_states) {
            if (other_id != agent_id) {
                dynamic_obstacles.push_back(state.position);
            }
        }
    }

    auto ray_circle_intersection = [](double ox, double oy, double dx, double dy,
                                      double cx, double cy, double radius) -> double {
        // Solve ||(o + t*d) - c||^2 = r^2 in 2D. Return nearest positive t.
        const double rx = ox - cx;
        const double ry = oy - cy;

        const double b = 2.0 * (dx * rx + dy * ry);
        const double c = rx * rx + ry * ry - radius * radius;
        const double disc = b * b - 4.0 * c;
        if (disc < 0.0) {
            return -1.0;
        }

        const double sqrt_disc = std::sqrt(disc);
        const double t1 = (-b - sqrt_disc) / 2.0;
        const double t2 = (-b + sqrt_disc) / 2.0;

        if (t1 > 0.0) return t1;
        if (t2 > 0.0) return t2;
        return -1.0;
    };

    // Compact scan: ray i points at angle i * angle_step; ranges rounded to cm.
    json ranges = json::array();
    json hits = json::array();
    for (int i = 0; i < num_rays; ++i) {
        const double angle = (2.0 * M_PI * i) / static_cast<double>(num_rays);
        const double dx = std::cos(angle);
        const double dy = std::sin(angle);

        double closest = max_range;
        bool hit = false;

        // Static ship obstacle at origin
        {
            const double t_ship = ray_circle_intersection(
                agent_pos.X(), agent_pos.Y(), dx, dy, 0.0, 0.0, ship_radius);
            if (t_ship > 0.0 && t_ship < closest) {
                closest = t_ship;
                hit = true;
            }
        }

        // Dynamic obstacles (other drones)
        for (const auto& p : dynamic_obstacles) {
            const double t = ray_circle_intersection(
                agent_pos.X(), agent_pos.Y(), dx, dy, p.X(), p.Y(), drone_radius);
            if (t > 0.0 && t < closest) {
                closest = t;
                hit = true;
            }
        }

        const double measured = std::max(0.0, std::min(closest, max_range));
        ranges.push_back(std::round(measured * 100.0) / 100.0);
        hits.push_back(hit ? 1 : 0);
    }
    return {{"angle_step", 2.0 * M_PI / num_rays}, {"ranges", ranges}, {"hits", hits}};
}

namespace {
double env_double(const char* name, double fallback) {
    const char* v = std::getenv(name);
    return (v != nullptr && std::strlen(v) > 0) ? std::atof(v) : fallback;
}
}  // namespace

void GazeboSimulator::configure_fuze() {
    const char* on = std::getenv("FUZE");
    _fuze_on = !(on != nullptr && (std::strcmp(on, "0") == 0 || std::strcmp(on, "off") == 0));
    _fuze_period = 1.0 / std::max(1.0, env_double("FUZE_HZ", 50.0));
    _fuze_range = env_double("FUZE_RANGE_M", 10.0);
    _fuze_noise = std::max(0.0, env_double("FUZE_NOISE_M", 0.1));
    _fuze_latency = std::max(0.0, env_double("FUZE_LATENCY_S", 0.0));
    _fuze_rng.seed(static_cast<unsigned>(env_double("FUZE_SEED", 0.0)));
    std::cout << "[GazeboSimulator] Proximity fuze " << (_fuze_on ? "on" : "off") << ": range " << _fuze_range
              << " m, " << 1.0 / _fuze_period << " Hz, noise " << _fuze_noise << " m, latency " << _fuze_latency
              << " s" << std::endl;
}

void GazeboSimulator::publish_fuze(double sim_time, const std::vector<std::string>& agents) {
    // Truth snapshot: drones and threats (the markers move along the ship's truth tracks).
    std::map<std::string, ignition::math::Vector3d> drones;
    std::vector<ignition::math::Vector3d> threats;
    {
        std::lock_guard<std::mutex> lock(_state_mtx);
        for (const auto& [id, st] : _drone_states) drones[id] = st.position;
        for (const auto& [id, m] : _threat_markers) threats.push_back(m.p0 + m.v * (sim_time - m.t0));
    }
    const double ship_radius = 16.0;   // the ship is a vertical cylinder at the origin (as for the lidar)
    std::normal_distribution<double> noise(0.0, _fuze_noise > 0.0 ? _fuze_noise : 1.0);
    auto round_mm = [](double v) { return std::round(v * 1000.0) / 1000.0; };
    for (const auto& agent_id : agents) {
        auto self = drones.find(agent_id);
        if (self == drones.end()) continue;
        const ignition::math::Vector3d me = self->second;
        std::vector<ignition::math::Vector3d> seen;
        for (const auto& [id, pos] : drones)
            if (id != agent_id && (pos - me).Length() <= _fuze_range) seen.push_back(pos - me);
        for (const auto& pos : threats)
            if ((pos - me).Length() <= _fuze_range) seen.push_back(pos - me);
        const double r_xy = std::hypot(me.X(), me.Y());
        if (r_xy > 1e-6 && r_xy - ship_radius <= _fuze_range) {
            // nearest point of the hull, at the drone's altitude
            const double k = ship_radius / r_xy;
            seen.push_back(ignition::math::Vector3d(me.X() * k, me.Y() * k, me.Z()) - me);
        }
        json objects = json::array();
        for (const auto& rel : seen) {
            const double nx = _fuze_noise > 0.0 ? noise(_fuze_rng) : 0.0;
            const double ny = _fuze_noise > 0.0 ? noise(_fuze_rng) : 0.0;
            const double nz = _fuze_noise > 0.0 ? noise(_fuze_rng) : 0.0;
            objects.push_back({round_mm(rel.X() + nx), round_mm(rel.Y() + ny), round_mm(rel.Z() + nz)});
        }
        _fuze_queue.push_back({sim_time + _fuze_latency, "drone/" + agent_id + "/fuze",
                               json{{"sim_time", sim_time}, {"objects", objects}}.dump()});
    }
}

void GazeboSimulator::flush_fuze(double sim_time) {
    while (!_fuze_queue.empty() && _fuze_queue.front().release <= sim_time + 1e-9) {
        if (_session) {
            _session->put(zenoh::KeyExpr(_fuze_queue.front().topic), zenoh::Bytes(_fuze_queue.front().payload),
                          zenoh::Session::PutOptions::create_default());
        }
        _fuze_queue.pop_front();
    }
}

json GazeboSimulator::get_drone_pose(const std::string& agent_id) {
    std::lock_guard<std::mutex> lock(_state_mtx);
    if (_drone_states.find(agent_id) != _drone_states.end()) {
        const auto& state = _drone_states[agent_id];
        auto mm = [](double v) { return std::round(v * 1000.0) / 1000.0; };
        return {
            {"x", mm(state.position.X())},
            {"y", mm(state.position.Y())},
            {"z", mm(state.position.Z())},
            {"roll", 0.0},
            {"pitch", 0.0},
            {"yaw", 0.0},
            {"vx", mm(state.linear_velocity.X())},
            {"vy", mm(state.linear_velocity.Y())},
            {"vz", mm(state.linear_velocity.Z())}
        };
    }
    return {{"x",0},{"y",0},{"z",1},{"roll",0},{"pitch",0},{"yaw",0},{"vx",0},{"vy",0},{"vz",0}};
}

void GazeboSimulator::publish_sensor_data(const std::string& agent_id, const json& sensor_data) {
    std::string topic = "drone/" + agent_id + "/sensors";
    std::string payload = sensor_data.dump();
    auto put_opt = zenoh::Session::PutOptions::create_default();
    if (_session) {
        _session->put(zenoh::KeyExpr(topic), zenoh::Bytes(payload), std::move(put_opt));
    }
}

void GazeboSimulator::on_world_stats(ConstWorldStatisticsPtr &_msg) {
    if (!_msg->has_sim_time()) return;
    const double sim = _msg->sim_time().sec() + _msg->sim_time().nsec() * 1e-9;
    const double real = _msg->has_real_time() ? _msg->real_time().sec() + _msg->real_time().nsec() * 1e-9 : 0.0;
    std::lock_guard<std::mutex> lock(_state_mtx);
    if (_stats_real_time > 0.0 && real > _stats_real_time && sim >= _sim_time) {
        const double rtf = (sim - _sim_time) / (real - _stats_real_time);
        _rtf = std::clamp(0.8 * _rtf + 0.2 * rtf, 0.0, 10.0);
    }
    if (sim + 1.0 < _clock_out) {
        _clock_out = sim;           // world reset: let the clock go back
    }
    _paused = _msg->has_paused() && _msg->paused();
    _sim_time = sim;
    _stats_real_time = real;
    _stats_wall = std::chrono::steady_clock::now();
}

double GazeboSimulator::estimated_sim_time() {
    double est = _sim_time;
    if (_sim_time > 0.0 && !_paused) {
        const double since = std::chrono::duration<double>(std::chrono::steady_clock::now() - _stats_wall).count();
        est += std::min(since, 0.5) * _rtf;
    }
    _clock_out = std::max(_clock_out, est);   // never step backwards between stats
    return _clock_out;
}

void GazeboSimulator::step() {
    apply_pending_despawns();

    double current_sim_time = 0.0;
    // Snapshot live drones: zenoh callbacks mutate _spawned_agents concurrently.
    std::vector<std::string> active_agents;
    {
        std::lock_guard<std::mutex> lock(_state_mtx);
        current_sim_time = estimated_sim_time();
        for (const auto& [agent_id, spawned] : _spawned_agents) {
            if (spawned) active_agents.push_back(agent_id);
        }
    }

    double dt = 0.0;
    if (_last_sim_time > 0.0 && current_sim_time > _last_sim_time) {
        dt = current_sim_time - _last_sim_time;
    } 
    
    if (_last_sim_time == 0.0 && current_sim_time > 0.0) {
        _last_sim_time = current_sim_time;
    }

    // Only publish when simulation time has advanced by at least 0.02s (50Hz) to avoid DDoSing Zenoh
    // Pose at 50 Hz; a lidar scan rides along every 0.1 s of sim time (10 Hz).
    if (current_sim_time - _last_sensor_pub_time >= 0.02) {
        _last_sensor_pub_time = current_sim_time;
        const bool with_lidar = current_sim_time - _last_lidar_pub_time >= 0.1;
        if (with_lidar) _last_lidar_pub_time = current_sim_time;
        for (const auto& agent_id : active_agents) {
            json sensor_data = {
                {"sim_time", current_sim_time},
                {"pose", get_drone_pose(agent_id)}
            };
            if (with_lidar) {
                sensor_data["lidar"] = simulate_lidar(agent_id);
            }
            publish_sensor_data(agent_id, sensor_data);
        }
    }

    if (_fuze_on && current_sim_time - _last_fuze_pub_time >= _fuze_period - 1e-3) {
        _last_fuze_pub_time = current_sim_time;
        publish_fuze(current_sim_time, active_agents);
    }
    flush_fuze(current_sim_time);

    if (current_sim_time - _last_clock_pub_time >= 0.1 && _pub_clock) {
        _last_clock_pub_time = current_sim_time;
        _pub_clock->put(zenoh::Bytes(json{{"sim_time", current_sim_time}}.dump()));
    }

    if (dt > 0.0) {
        move_threat_markers(current_sim_time);
    }
    
    for (const auto& agent_id : active_agents) {
        {

            // Publish LinkData to force Gazebo to move the visual model
            if (_physics_pub && dt > 0.0) {
                ignition::math::Vector3d publish_pos;
                ignition::math::Quaterniond publish_ori;
                bool has_state = false;

                {
                    std::lock_guard<std::mutex> lock(_state_mtx);
                    auto it = _drone_states.find(agent_id);
                    if (it != _drone_states.end()) {
                        auto& state = it->second;

                        // Integrate position using gazebo simulation time
                        state.position += state.linear_velocity * dt;

                        // Hard world safety constraints.
                        const double min_z = 1.0;
                        const double max_z = 60.0;
                        if (state.position.Z() < min_z) {
                            state.position.Z(min_z);
                            if (state.linear_velocity.Z() < 0.0) {
                                state.linear_velocity.Z(0.0);
                            }
                        } else if (state.position.Z() > max_z) {
                            state.position.Z(max_z);
                            if (state.linear_velocity.Z() > 0.0) {
                                state.linear_velocity.Z(0.0);
                            }
                        }

                        // Keep drones outside the ship collider at origin.
                        const double ship_keepout = 17.5;  // ship radius + drone radius buffer
                        const double px = state.position.X();
                        const double py = state.position.Y();
                        const double r_xy = std::hypot(px, py);
                        if (r_xy < ship_keepout) {
                            const double nx = (r_xy > 1e-6) ? (px / r_xy) : 1.0;
                            const double ny = (r_xy > 1e-6) ? (py / r_xy) : 0.0;
                            state.position.X(nx * ship_keepout);
                            state.position.Y(ny * ship_keepout);

                            // Remove inward radial velocity so the agent cannot tunnel through.
                            const double inward = state.linear_velocity.X() * (-nx) + state.linear_velocity.Y() * (-ny);
                            if (inward > 0.0) {
                                state.linear_velocity.X(state.linear_velocity.X() + nx * inward);
                                state.linear_velocity.Y(state.linear_velocity.Y() + ny * inward);
                            }
                        }

                        publish_pos = state.position;
                        publish_ori = state.orientation;
                        has_state = true;
                    }
                }

                if (has_state) {
                    gazebo::msgs::Model msg;
                    msg.set_name(agent_id);

                    gazebo::msgs::Pose* pose_ptr = msg.mutable_pose();
                    gazebo::msgs::Vector3d* pos = pose_ptr->mutable_position();
                    pos->set_x(publish_pos.X());
                    pos->set_y(publish_pos.Y());
                    pos->set_z(publish_pos.Z());

                    gazebo::msgs::Quaternion* rot = pose_ptr->mutable_orientation();
                    rot->set_x(publish_ori.X());
                    rot->set_y(publish_ori.Y());
                    rot->set_z(publish_ori.Z());
                    rot->set_w(publish_ori.W());

                    _physics_pub->Publish(msg);
                }
            }
        }
    }
    
    if (dt > 0.0) {
        _last_sim_time = current_sim_time;
    }
    
    std::this_thread::sleep_for(_tick_period); // 50 Hz of sim time
}
