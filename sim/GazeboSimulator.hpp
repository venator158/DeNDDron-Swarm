#pragma once

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <nlohmann/json.hpp>
#include <zenoh.hxx>
#include <optional>
#include <map>
#include <string>
#include <vector>
#include <mutex>

using json = nlohmann::json;

class GazeboSimulator {
public:
    GazeboSimulator();
    ~GazeboSimulator();

    void init();
    void step();
    void disconnect();

private:
    std::mutex _state_mtx;

    // Gazebo transport layer
    gazebo::transport::NodePtr _gznode;
    gazebo::transport::PublisherPtr _factory_pub;  // For spawning models
    gazebo::transport::PublisherPtr _physics_pub;  // For applying forces to models
    gazebo::transport::SubscriberPtr _stats_sub;   // For receiving simulation time
    double _sim_time = 0.0;
    double _last_sim_time = 0.0;

    void on_world_stats(ConstWorldStatisticsPtr &_msg);
    
    // Cache drone state (position, velocity, orientation)
    struct DroneState {
        ignition::math::Vector3d position;
        ignition::math::Vector3d linear_velocity;
        ignition::math::Vector3d angular_velocity;
        ignition::math::Quaterniond orientation;
    };
    std::map<std::string, DroneState> _drone_states;

    // Zenoh C++ objects
    std::optional<zenoh::Session> _session;
    std::optional<zenoh::Subscriber<void>> _sub_agent_join;
    std::optional<zenoh::Subscriber<void>> _sub_cmd_vel;
    std::optional<zenoh::Publisher> _pub_metrics;

    std::map<std::string, bool> _spawned_agents;

    struct SpawnPoint {
        double x;
        double y;
        double z;
    };
    std::map<std::string, SpawnPoint> _spawn_config;
    std::string _spawn_config_path;

    void on_agent_join(const zenoh::Sample& sample);
    void on_cmd_vel(const zenoh::Sample& sample);
    void load_spawn_config();

    std::string generate_drone_sdf(const std::string& agent_id, double x, double y, double z);
    void spawn_drone(const std::string& agent_id, const json& initial_pos);
    void update_drone_velocity(const std::string& agent_id, const json& cmd_msg);
    
    json simulate_lidar(const std::string& agent_id);
    json get_drone_pose(const std::string& agent_id);
    void publish_sensor_data(const std::string& agent_id, const json& sensor_data);
};
