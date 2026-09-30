#pragma once

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <nlohmann/json.hpp>
#include <zenoh.hxx>
#include <optional>
#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include <deque>
#include <random>

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
    gazebo::transport::PublisherPtr _request_pub;  // Persistent ~/request publisher (entity_delete)
    void delete_model(const std::string& name);
    gazebo::transport::SubscriberPtr _stats_sub;   // For receiving simulation time
    // Gazebo publishes world_stats at only ~5 Hz, so sim time is extrapolated
    // between messages with the observed real-time factor (guarded by _state_mtx).
    double _sim_time = 0.0;                 // sim time of the last world_stats
    double _stats_real_time = 0.0;          // real time of the last world_stats
    std::chrono::steady_clock::time_point _stats_wall;
    double _rtf = 1.0;
    // Loop period: 20 ms of sim time at the target real-time factor (SIM_RTF), so drones
    // get the same sensor and integration rate per simulated second at any speed-up.
    std::chrono::microseconds _tick_period{20000};
    bool _paused = false;
    double _clock_out = 0.0;                // last value handed out (monotonic)
    double estimated_sim_time();            // requires _state_mtx
    double _last_lidar_pub_time = -1.0;
    double _last_sim_time = 0.0;
    double _last_sensor_pub_time = 0.0;

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
    std::optional<zenoh::Subscriber<void>> _sub_agent_despawn;
    std::optional<zenoh::Subscriber<void>> _sub_threat_tracks;
    std::optional<zenoh::Publisher> _pub_clock;
    double _last_clock_pub_time = 0.0;

    // false = despawned (expended drones stay false so a rejoin cannot respawn them)
    std::map<std::string, bool> _spawned_agents;
    std::vector<std::string> _pending_despawns;   // guarded by _state_mtx, applied in step()

    // Threat markers (visual only), moved along the ship's straight-line tracks
    struct ThreatMarker {
        ignition::math::Vector3d p0;   // position at t0
        ignition::math::Vector3d v;    // velocity
        double t0;                     // sim time of p0
    };
    std::map<std::string, ThreatMarker> _threat_markers;          // guarded by _state_mtx
    std::vector<std::string> _pending_model_deletes;              // guarded by _state_mtx
    std::map<std::string, bool> _seen_cmd_vel;

    struct SpawnPoint {
        double x;
        double y;
        double z;
    };
    std::map<std::string, SpawnPoint> _spawn_config;
    std::map<std::string, SpawnPoint> _goal_config;
    std::string _spawn_config_path;

    void on_agent_join(const zenoh::Sample& sample);
    void on_cmd_vel(const zenoh::Sample& sample);
    void on_agent_despawn(const zenoh::Sample& sample);
    void on_threat_track(const zenoh::Sample& sample);
    void move_threat_markers(double sim_time);
    std::string generate_threat_sdf(const std::string& threat_id, const std::string& type,
                                    int level, double x, double y, double z);
    void apply_pending_despawns();
    void load_spawn_config();

    std::string generate_drone_sdf(const std::string& agent_id, double x, double y, double z);
    std::string generate_goal_sdf(const std::string& agent_id, double x, double y, double z);
    void spawn_drone(const std::string& agent_id, const json& initial_pos);
    void update_drone_velocity(const std::string& agent_id, const json& cmd_msg);
    
    json simulate_lidar(const std::string& agent_id);

    // Proximity fuze (drone/{id}/fuze): unlabelled 3D positions, relative to the drone, of every
    // object within range (threats, other drones, the ship's hull), with noise and latency.
    // Env: FUZE (1), FUZE_HZ (50), FUZE_RANGE_M (10), FUZE_NOISE_M (0.1), FUZE_LATENCY_S (0), FUZE_SEED (0).
    bool _fuze_on = true;
    double _fuze_period = 0.02;              // sim seconds between scans
    double _fuze_range = 10.0;
    double _fuze_noise = 0.1;                // sd per axis, m
    double _fuze_latency = 0.0;              // sim seconds between measurement and delivery
    double _last_fuze_pub_time = 0.0;
    std::mt19937 _fuze_rng{0};
    struct PendingFuze { double release; std::string topic; std::string payload; };
    std::deque<PendingFuze> _fuze_queue;     // scans held back by the latency (step() thread only)
    void configure_fuze();
    void publish_fuze(double sim_time, const std::vector<std::string>& agents);
    void flush_fuze(double sim_time);
    json get_drone_pose(const std::string& agent_id);
    void publish_sensor_data(const std::string& agent_id, const json& sensor_data);
};
