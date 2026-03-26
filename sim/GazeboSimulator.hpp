#pragma once

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <nlohmann/json.hpp>
#include <zenoh.h>
#include <memory>
#include <map>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace denddron {

/**
 * @class GazeboSimulator
 * @brief Bridge between Gazebo physics engine and Zenoh messaging
 * 
 * This component:
 * 1. Connects to gzserver (Gazebo simulator running in same container)
 * 2. Listens for agent join events via Zenoh (swarm/agents/join)
 * 3. Dynamically spawns drone models in Gazebo when agents join
 * 4. Reads velocity commands from agents (drone/*/cmd_vel) and updates physics
 * 5. Publishes sensor data (lidar simulation) back to agents (drone/*/sensors)
 * 6. Publishes collision metrics to dedicated metrics node (swarm/metrics)
 */
class GazeboSimulator {
public:
    /**
     * @brief Constructor - initializes Gazebo and Zenoh connections
     */
    GazeboSimulator();

    /**
     * @brief Destructor - cleans up Gazebo and Zenoh resources
     */
    ~GazeboSimulator();

    /**
     * @brief Connect to gzserver and initialize Zenoh session
     * @return true if connection successful, false otherwise
     */
    bool connect();

    /**
     * @brief Disconnect and shutdown
     */
    void disconnect();

    /**
     * @brief Main event loop - processes Zenoh messages and publishes sensor data
     */
    void run();

    /**
     * @brief Spawn a drone sphere model in Gazebo
     * @param agent_id - unique identifier for the agent
     * @param initial_pos - JSON with x, y, z coordinates
     */
    void spawn_drone(const std::string& agent_id, const json& initial_pos);

    /**
     * @brief Update drone velocity based on command from agent
     * @param agent_id - drone to update
     * @param cmd_msg - JSON with linear {x,y,z} and angular {z} velocity
     */
    void update_drone_velocity(const std::string& agent_id, const json& cmd_msg);

    /**
     * @brief Check if simulator is connected and ready
     */
    bool is_connected() const { return _connected; }

private:
    // Connection state
    bool _connected;
    z_session_t _session;

    // Gazebo transport layer
    gazebo::transport::NodePtr _gznode;
    gazebo::transport::PublisherPtr _factory_pub;  // For spawning models
    gazebo::transport::PublisherPtr _physics_pub;  // For applying forces to models
    
    // Per-drone publishers for velocity commands to Gazebo
    std::map<std::string, gazebo::transport::PublisherPtr> _model_pubs;
    
    // Cache drone state (position, velocity, orientation)
    struct DroneState {
        ignition::math::Vector3d position;
        ignition::math::Vector3d linear_velocity;
        ignition::math::Vector3d angular_velocity;
        ignition::math::Quaterniond orientation;
    };
    std::map<std::string, DroneState> _drone_states;

    /**
     * @brief Collision detection and response
     * Tracks collisions between drones and despawns on high-velocity impacts
     */
    struct CollisionEvent {
        std::string agent_a;
        std::string agent_b;
        double relative_velocity;  // m/s
        ignition::math::Vector3d contact_point;
        uint64_t timestamp;
    };
    
    // Collision history for this frame
    std::vector<CollisionEvent> _frame_collisions;
    
    // Despawn threshold: drones destroyed if relative velocity > this
    // Default: 5.0 m/s (meaningful impact speed)
    static constexpr double DRONE_DESPAWN_VELOCITY_THRESHOLD = 5.0;
    
    // Zenoh subscribers
    z_owned_subscriber_t _sub_agent_join;   // Listen for new agents
    z_owned_subscriber_t _sub_cmd_vel;      // Listen for velocity commands

    // Track which agents we've spawned to avoid duplicates
    std::map<std::string, bool> _spawned_agents;

    /**
     * @brief Callback when agent join event received
     * Parses JSON and calls spawn_drone
     */
    static void on_agent_join(const z_sample_t* sample, void* ctx);

    /**
     * @brief Callback when velocity command received
     * Parses JSON and calls update_drone_velocity
     */
    static void on_cmd_vel(const z_sample_t* sample, void* ctx);

    /**
     * @brief Generate SDF model string for a drone sphere
     * @param agent_id - name of the model
     * @param x, y, z - initial position
     * @return SDF XML as string
     * 
     * Creates a sphere drone with:
     * - Mass: 1.0 kg
     * - Radius: 0.25 meters
     * - Inertia: simple sphere inertia tensor
     * - Color: blue
     */
    std::string generate_drone_sdf(const std::string& agent_id,
                                   double x, double y, double z);

    /**
     * @brief Publish sensor data to agent
     * @param agent_id - which agent to send data to
     * @param sensor_data - JSON with lidar rays, position, velocity, etc.
     */
    void publish_sensor_data(const std::string& agent_id, const json& sensor_data);

    /**
     * @brief Publish metrics to the dedicated metrics node
     * @param metrics - JSON with collision counts, contact forces, etc.
     */
    void publish_metrics(const json& metrics);

    /**
     * @brief Simulate lidar sensor for a drone
     * @param agent_id - which drone to get lidar for
     * @return JSON with lidar ray data (distances, angles, etc.)
     * 
     * This performs raycasts from the drone position to detect nearby obstacles.
     * For now, uses simple sphere-based obstacle detection.
     * Future: integrate with Gazebo raytracing for full lidar simulation.
     */
    json simulate_lidar(const std::string& agent_id);

    /**
     * @brief Get current position and orientation of a drone in Gazebo
     * @param agent_id - drone to query
     * @return JSON with x, y, z, roll, pitch, yaw
     */
    json get_drone_pose(const std::string& agent_id);

    /**
     * @brief Query collision contacts and generate metrics
     * @return JSON with collision data, contact forces, etc.
     */
    json gather_metrics();

    /**
     * @brief Detect drone-to-drone collisions and check velocity threshold
     * Queries Gazebo for contact information and simulates collision effects
     */
    void process_drone_collisions();

    /**
     * @brief Remove a drone from simulation
     * @param agent_id - drone to despawn
     * @param reason - cause of despawn (e.g., "collision", "command")
     * Publishes despawn event to swarm/agents/despawn for notification
     */
    void despawn_drone(const std::string& agent_id, const std::string& reason);

    /**
     * @brief Calculate relative velocity between two drones
     * @param agent_a - first drone
     * @param agent_b - second drone
     * @return relative velocity magnitude in m/s
     */
    double calculate_relative_velocity(const std::string& agent_a, 
                                       const std::string& agent_b) const;
};

}

#endif
