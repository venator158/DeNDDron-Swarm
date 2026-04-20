#include "GazeboSimulator.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cctype>
#include <fstream>
#include <cstdlib>

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
    
    _stats_sub = _gznode->Subscribe("~/world_stats", &GazeboSimulator::on_world_stats, this);

    std::cout << "[GazeboSimulator] Connected to Gazebo transport" << std::endl;

    // Zenoh Init
    auto config = zenoh::Config::create_default();
    _session.emplace(zenoh::Session::open(std::move(config)));

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

    auto pub_opt = zenoh::Session::PublisherOptions::create_default();
    _pub_metrics.emplace(_session->declare_publisher(
        zenoh::KeyExpr("swarm/metrics"), 
        std::move(pub_opt)
    ));

    load_spawn_config();

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
        for (auto it = cfg["agents"].begin(); it != cfg["agents"].end(); ++it) {
            const std::string agent_id = it.key();
            const json& agent_cfg = it.value();
            if (!agent_cfg.contains("spawn") || !agent_cfg["spawn"].is_object()) {
                continue;
            }

            const json& spawn = agent_cfg["spawn"];
            _spawn_config[agent_id] = SpawnPoint{
                spawn.value("x", 0.0),
                spawn.value("y", 0.0),
                spawn.value("z", 1.0)
            };
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
        << "      <visual name='visual'>"
        << "        <geometry><sphere><radius>1.25</radius></sphere></geometry>"
        << "        <material>"
        << "          <ambient>1.0 0.1 0.1 1.0</ambient>"
        << "          <diffuse>1.0 0.2 0.2 1.0</diffuse>"
        << "        </material>"
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
            _drone_states[agent_id].linear_velocity = ignition::math::Vector3d(vx, vy, vz);
            _drone_states[agent_id].angular_velocity = ignition::math::Vector3d(0, 0, yaw_rate);
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

    json lidar_data = json::array();
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
        lidar_data.push_back({
            {"angle", angle},
            {"distance", measured},
            {"intensity", hit ? 0.9 : 0.2},
            {"ray_id", i}
        });
    }
    return lidar_data;
}

json GazeboSimulator::get_drone_pose(const std::string& agent_id) {
    std::lock_guard<std::mutex> lock(_state_mtx);
    if (_drone_states.find(agent_id) != _drone_states.end()) {
        const auto& state = _drone_states[agent_id];
        return {
            {"x", state.position.X()},
            {"y", state.position.Y()},
            {"z", state.position.Z()},
            {"roll", 0.0},
            {"pitch", 0.0},
            {"yaw", 0.0},
            {"vx", state.linear_velocity.X()},
            {"vy", state.linear_velocity.Y()},
            {"vz", state.linear_velocity.Z()}
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
    if (_msg->has_sim_time()) {
        std::lock_guard<std::mutex> lock(_state_mtx);
        _sim_time = _msg->sim_time().sec() + _msg->sim_time().nsec() * 1e-9;
    }
}

void GazeboSimulator::step() {
    double current_sim_time = 0.0;
    {
        std::lock_guard<std::mutex> lock(_state_mtx);
        current_sim_time = _sim_time;
    }

    double dt = 0.0;
    if (_last_sim_time > 0.0 && current_sim_time > _last_sim_time) {
        dt = current_sim_time - _last_sim_time;
    } 
    
    if (_last_sim_time == 0.0 && current_sim_time > 0.0) {
        _last_sim_time = current_sim_time;
    }

    // Even if dt == 0 (simulator paused or no time elapsed), we can still publish sensors.

    for (const auto& [agent_id, spawned] : _spawned_agents) {
        if (spawned) {
            json sensor_data = {
                {"sim_time", current_sim_time},
                {"pose", get_drone_pose(agent_id)},
                {"lidar", simulate_lidar(agent_id)}
            };
            publish_sensor_data(agent_id, sensor_data);

            // Publish LinkData to force Gazebo to move the visual model
            if (_physics_pub && dt > 0.0) {
                std::lock_guard<std::mutex> lock(_state_mtx);
                if (_drone_states.find(agent_id) != _drone_states.end()) {
                    auto& state = _drone_states[agent_id];
                    
                    // Integrate position using gazebo simulation time
                    state.position += state.linear_velocity * dt;

                    gazebo::msgs::Model msg;
                    msg.set_name(agent_id);
                    
                    gazebo::msgs::Pose* pose_ptr = msg.mutable_pose();
                    gazebo::msgs::Vector3d* pos = pose_ptr->mutable_position();
                    pos->set_x(state.position.X());
                    pos->set_y(state.position.Y());
                    pos->set_z(state.position.Z());

                    gazebo::msgs::Quaternion* rot = pose_ptr->mutable_orientation();
                    rot->set_x(state.orientation.X());
                    rot->set_y(state.orientation.Y());
                    rot->set_z(state.orientation.Z());
                    rot->set_w(state.orientation.W());

                    _physics_pub->Publish(msg);
                }
            }
        }
    }
    
    if (dt > 0.0) {
        _last_sim_time = current_sim_time;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz real-time tick rate
}
