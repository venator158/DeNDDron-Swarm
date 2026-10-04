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
#include <set>

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
    // PERCEPTION=radar: the contact sensor becomes the hardware record's mmWave radar (range, rate,
    // range/angle noise; topic drone/{id}/radar) and the planar lidar is no longer computed.
    bool _radar_mode = false;
    double _radar_range_sigma = 0.05, _radar_az_sigma = 0.0349, _radar_el_sigma = 0.0349;   // m, rad, rad
    // Degradation (env, for sweeps): the environment is worse than the record; drones keep the record's figures.
    double _radar_miss_p = 0.0;     // RADAR_MISS_P: each real contact missed per scan
    double _radar_clutter = 0.0;    // RADAR_CLUTTER: mean false contacts per scan (Poisson, uniform in range)
    std::string _contact_topic = "fuze";
    json _hw = json::object();               // hardware record from the runtime config
    double _last_fuze_pub_time = 0.0;
    std::mt19937 _fuze_rng{0};
    struct PendingFuze { double release; std::string topic; std::string payload; };
    std::deque<PendingFuze> _fuze_queue;     // scans held back by the latency (step() thread only)
    void configure_fuze();

    // Physics owned by the simulator (truth): a drone asks to detonate (drone/{id}/detonate); the
    // blast is placed at its true position (sim/detonation) and destroys every other drone within
    // the warhead's kill radius, except drones on the same job (they detonate together by design;
    // drones report their current or last job on drone/{id}/job).  sim/truth carries true poses for
    // evaluation only; drones never subscribe to it.
    std::optional<zenoh::Subscriber<void>> _sub_detonate;
    std::optional<zenoh::Subscriber<void>> _sub_job;
    std::map<std::string, std::string> _drone_job;              // guarded by _state_mtx
    struct PendingDetonation { std::string agent_id; json msg; };
    std::vector<PendingDetonation> _pending_detonations;        // guarded by _state_mtx
    double _kill_radius = 8.0;                                  // hardware.warhead.kill_radius_m
    double _last_truth_pub_time = 0.0;
    void on_detonate(const zenoh::Sample& sample);
    void on_job(const zenoh::Sample& sample);
    void apply_detonations(double sim_time);
    void publish_truth(double sim_time);
    void put(const std::string& key, const std::string& payload);

    // Localization (LOCALIZATION=truth|anchors|coop).  Not truth: sensor frames lose x, y and the
    // horizontal velocity (altitude and attitude stay: barometer/IMU/compass, taken as perfect), and
    // each drone gets UWB ranges (drone/{id}/uwb) to the ship's anchors (hardware record: noise,
    // range limit, dropouts, channel airtime), plus, with coop, to the peers it asks for
    // (drone/{id}/uwb_tx), each with that peer's state payload.  UWB_JAM jams anchors/all UWB
    // for sim-time windows: "anchors:t0:t1,all:t0:t1", optionally only for drones within r m of
    // a jammer at (x, y): "anchors:t0:t1:x:y:r".
    std::string _loc_mode = "truth";
    std::vector<ignition::math::Vector3d> _anchors;
    double _uwb_sigma = 0.1, _uwb_range = 250.0, _uwb_dropout = 0.02, _uwb_capacity = 1000.0;
    // Degradation (env, for sweeps; drones keep the record's sigma): extra Gaussian noise, and
    // non-line-of-sight ranges: with probability _uwb_nlos_p a positive bias, exponential with mean _uwb_nlos_bias.
    double _uwb_extra_sigma = 0.0, _uwb_nlos_p = 0.0, _uwb_nlos_bias = 0.0;
    double _uwb_anchor_period = 0.5, _uwb_peer_period = 0.5;
    int _uwb_max_peers = 6;
    double _last_uwb_anchor_time = 0.0, _last_uwb_peer_time = 0.0;
    struct JamWindow { std::string what; double t0, t1; bool local = false; double x = 0, y = 0, r = 0; };
    std::vector<JamWindow> _uwb_jam;
    std::mt19937 _uwb_rng{1};
    std::map<std::string, json> _uwb_tx;                       // agent -> latest uwb_tx (guarded)
    std::optional<zenoh::Subscriber<void>> _sub_uwb_tx;
    void configure_localization();

    // Environment (hardware record "environment", real values x speed_scale; WIND_MPS="x,y" and
    // GUST_SIGMA_MPS override): a steady wind plus per-drone gusts (Ornstein-Uhlenbeck, gust_tau_s)
    // push the drones' true positions; drones cannot sense it.  Default calm.
    ignition::math::Vector3d _wind{0, 0, 0};
    double _gust_sigma = 0.0, _gust_tau = 5.0;
    std::map<std::string, ignition::math::Vector3d> _gust;     // per drone (step thread only)
    std::mt19937 _env_rng{7};
    void configure_environment();

    // IMU (hardware record "imu" + "ahrs"; not with LOCALIZATION=truth): each sensor frame carries the
    // flight controller's horizontal delta-velocity since the previous frame ("imu": {dt, dv}),
    // measured over the ground (so gusts are in it): (1 + scale factor) dv_true + (accelerometer bias
    // + tilt-equivalent bias g sin(tilt)) dt + white noise.  Both biases are Gauss-Markov per drone.
    // Error scaling follows the simulation's time stretch (IMU_ERROR_SCALING=dilated|real, as
    // hardware.imu_errors): biases x s^2, noise density x s^1.5, correlation times / s.
    // IMU_EXTRA_BIAS_MPS2 (real m/s^2, scaled the same way) adds a constant bias, for sweeps.
    struct ImuState {
        ignition::math::Vector3d vg_last{0, 0, 0};    // ground velocity at the previous frame
        double t_last = -1.0;
        double ba[2] = {0, 0}, bt[2] = {0, 0}, sf[2] = {0, 0};
    };
    bool _imu_on = false;
    double _imu_bias = 0.0, _imu_bias_tau = 3000.0, _imu_tilt = 0.0, _imu_tilt_tau = 200.0;
    double _imu_noise = 0.0, _imu_sf = 0.0, _imu_extra = 0.0;
    std::map<std::string, ImuState> _imu;                       // per drone (step thread only)
    std::mt19937 _imu_rng{11};
    void configure_imu();
    json imu_frame(const std::string& agent_id, double sim_time);
    void on_uwb_tx(const zenoh::Sample& sample);
    bool uwb_jammed(const std::string& what, double t, const ignition::math::Vector3d& at) const;
    void publish_uwb(double sim_time, const std::vector<std::string>& agents, bool anchors, bool peers);
    void publish_fuze(double sim_time, const std::vector<std::string>& agents);
    void flush_fuze(double sim_time);
    json get_drone_pose(const std::string& agent_id);
    void publish_sensor_data(const std::string& agent_id, const json& sensor_data);
};
