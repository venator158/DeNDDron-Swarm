#include "Agent.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace denddron;

Agent::Agent(const AgentConfig& config) 
    : _config(config), _running(false) 
{
    std::cout << "[Agent] Booting " << _config.agent_id << "..." << std::endl;

    _enlister = std::make_unique<JobSelectionHandler>(_config.agent_id);
    _talker = std::make_unique<ConsensusHandler>(_config.agent_id);
    _eyes = std::make_unique<ObstaclePerceptionHandler>(_config.agent_id);
    _reflexes = std::make_unique<PathFindingHandler>();

    setup_comms();
}

Agent::~Agent() {
    stop();
    if (_session) {
        z_close(_session);
    }
}

void Agent::setup_comms() {
    z_owned_config_t z_config = z_config_default();
    
    std::cout << "[Zenoh] Opening session..." << std::endl;
    _session = z_open(z_move(z_config));

    if (!_session) {
        throw std::runtime_error("Unable to open Zenoh session!");
    }

    // --- PUBLISHERS ---
    // Publishes Velocity Vectors (Reflex output)
    std::string key_cmd = "drone/" + _config.agent_id + "/cmd_vel";
    _pub_cmd_vel = z_declare_publisher(_session, z_keyexpr(key_cmd.c_str()), NULL);

    // Publishes Gossip (State/Position)
    _pub_gossip = z_declare_publisher(_session, z_keyexpr("swarm/gossip"), NULL);

    // --- SUBSCRIBERS ---
    // 1. Proprioception (Sensors from Gazebo/Hardware)
    std::string key_sensors = "drone/" + _config.agent_id + "/sensors";
    _sub_sensors = z_declare_subscriber(
        _session, 
        z_keyexpr(key_sensors.c_str()), 
        z_closure( +[](const z_sample_t* sample, void* ctx) {
            static_cast<Agent*>(ctx)->on_sensor_data(sample);
        }, this), 
        NULL
    );

    // 2. Global Threats (Ship Oracle)
    _sub_threats = z_declare_subscriber(
        _session, 
        z_keyexpr("ship/threats"), 
        z_closure( +[](const z_sample_t* sample, void* ctx) {
            static_cast<Agent*>(ctx)->on_threat_update(sample);
        }, this), 
        NULL
    );
    
    // 3. Swarm Gossip (Other Drones)
    _sub_gossip = z_declare_subscriber(
        _session, 
        z_keyexpr("swarm/gossip"), 
        z_closure( +[](const z_sample_t* sample, void* ctx) {
            static_cast<Agent*>(ctx)->on_gossip_received(sample);
        }, this), 
        NULL
    );
}

void Agent::run() {
    _running = true;
    auto tick_rate = std::chrono::milliseconds(static_cast<int>(1000.0f / _config.loop_rate_hz));

    std::cout << "[Agent] Online. Loop Rate: " << _config.loop_rate_hz << "Hz" << std::endl;

    while (_running) {
        auto start_time = std::chrono::steady_clock::now();

        // --- PHASE 1: BRAIN (Strategy & Consensus) ---
        // Decides "Where should I go?"
        // Only updates if we have new threats or consensus conflicts
        auto [has_job, target_pos] = _talker->get_assigned_target();
        
        // --- PHASE 2: REFLEXES (Perception & Path Finding) ---
        // Decides "How do I move *right now* to stay alive?"
        tick_reflexes();

        // Maintain Loop Rate
        std::this_thread::sleep_until(start_time + tick_rate);
    }
}

void Agent::tick_reflexes() {
    // 1. Get the current Goal (from Brain)
    auto [has_target, target] = _talker->get_assigned_target();
    
    // 2. Get local obstacles (from Nervous System / Voxel Map)
    // The Nervous System has already merged Lidar + Gossip into "Bubbles"
    auto obstacles = _eyes->get_nearby_obstacles();

    // 3. Calculate Physics Force (APF)
    // returns a velocity vector {vx, vy, vz, yaw_rate}
    VelocityCommand cmd = _reflexes->calculate_next_step(target, obstacles);

    // 4. Actuate (Publish to Zenoh)
    // Convert Struct -> JSON -> Zenoh Bytes
    json j_cmd = {
        {"linear", {{"x", cmd.vx}, {"y", cmd.vy}, {"z", cmd.vz}}},
        {"angular", {{"z", cmd.yaw_rate}}}
    };
    
    std::string payload = j_cmd.dump();
    z_publisher_put(_pub_cmd_vel, (const uint8_t*)payload.data(), payload.size(), NULL);
}

// --- Event Handlers ---

void Agent::on_sensor_data(const z_sample_t* sample) {
    // Deserialize and feed the Nervous System
    // This updates the drone's OWN position/velocity perception
    std::string payload(reinterpret_cast<const char*>(sample->payload.start), sample->payload.len);
    auto data = json::parse(payload);
    _eyes->update_proprioception(data);
}

void Agent::on_gossip_received(const z_sample_t* sample) {
    // Feed the Nervous System with Peer data
    // This runs the "Uncertainty Engine" updates internally
    std::string payload(reinterpret_cast<const char*>(sample->payload.start), sample->payload.len);
    auto data = json::parse(payload);
    
    // Don't listen to yourself
    if (data["id"] == _config.agent_id) return;

    _eyes->process_rumor(data);
}

void Agent::on_threat_update(const z_sample_t* sample) {
    // Feed the Strategist
    std::string payload(reinterpret_cast<const char*>(sample->payload.start), sample->payload.len);
    auto data = json::parse(payload);
    _enlister->update_threat_landscape(data);
    
    // Strategist might trigger a Consensus update here
    auto desired_job = _enlister->select_best_job();
    _talker->volunteer_for(desired_job);
}

void Agent::stop() {
    _running = false;
}