#include "GazeboSimulator.hpp"
#include <iostream>
#include <unistd.h>
#include <thread>
#include <chrono>

using json = nlohmann::json;

GazeboSimulator::GazeboSimulator() {
}

GazeboSimulator::~GazeboSimulator() {
}

void GazeboSimulator::init() {
    zenoh::Config config;
    std::cout << "[Gazebo] Initializing Zenoh..." << std::endl;
    _session = zenoh::Session::open(std::move(config));

    std::cout << "[Gazebo] Registering publisher..." << std::endl;
    auto pub_opt = zenoh::Session::PublisherOptions::create_default();
    _pub_state = _session.declare_publisher(
        zenoh::KeyExpr("swarm/+/state"), 
        std::move(pub_opt)
    );

    std::cout << "[Gazebo] Registering subscriber..." << std::endl;
    auto sub_opt = zenoh::Session::SubscriberOptions::create_default();
    _sub_motors = _session.declare_subscriber(
        zenoh::KeyExpr("swarm/+/cmd_vel"),
        std::bind(&GazeboSimulator::on_motor_cmd, this, std::placeholders::_1),
        [](){},
        std::move(sub_opt)
    );

    _drone_states["drone_1"] = {0.0, 0.0};
    _drone_states["drone_2"] = {1.0, 1.0};
}

void GazeboSimulator::on_motor_cmd(const zenoh::Sample& sample) {
    std::string key = sample.get_keyexpr().as_string();
    std::string payload = sample.get_payload().as_string(); 
    
    // Extract drone ID from key expr (e.g., swarm/drone_1/cmd_vel)
    size_t first_slash = key.find('/');
    size_t second_slash = key.find('/', first_slash + 1);
    if(first_slash != std::string::npos && second_slash != std::string::npos) {
        std::string drone_id = key.substr(first_slash + 1, second_slash - first_slash - 1);
        std::lock_guard<std::mutex> lock(_state_mtx);
        
        try {
            json data = json::parse(payload);
            _drone_states[drone_id].first += data.value("vx", 0.0) * 0.1;
            _drone_states[drone_id].second += data.value("vy", 0.0) * 0.1;
            std::cout << "[Gazebo] Applied cmd to " << drone_id << "\n";
        } catch(...) {
            // Drop invalid commands
        }
    }
}

void GazeboSimulator::publish_state() {
    std::lock_guard<std::mutex> lock(_state_mtx);
    for (const auto& [drone_id, pos] : _drone_states) {
        json state;
        state["x"] = pos.first;
        state["y"] = pos.second;
        
        std::string payload = state.dump();
        std::string topic = "swarm/" + drone_id + "/state";
        
        auto put_opt = zenoh::Session::PutOptions::create_default();
        _session.put(
            zenoh::KeyExpr(topic),
            zenoh::Bytes(payload),
            std::move(put_opt)
        );
    }
}

void GazeboSimulator::step() {
    publish_state();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
}
