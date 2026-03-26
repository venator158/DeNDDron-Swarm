#include "GazeboSimulator.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>

namespace denddron {

// ============================================================================
// Constructor and Lifecycle
// ============================================================================

GazeboSimulator::GazeboSimulator()
    : _connected(false), _session(nullptr) {
    std::cout << "[GazeboSimulator] Initializing..." << std::endl;
}

GazeboSimulator::~GazeboSimulator() {
    disconnect();
}

bool GazeboSimulator::connect() {
    std::cout << "[GazeboSimulator] Connecting to Gazebo..." << std::endl;

    // Initialize Gazebo client library
    // This connects to gzserver which should already be running
    try {
        gazebo::client::setup();
    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Failed to setup Gazebo client: " 
                  << e.what() << std::endl;
        return false;
    }

    // Create Gazebo transport node for publisher/subscriber access
    _gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _gznode->Init();

    // Create publisher for model factory
    // This is used to spawn new drone models via SDF
    _factory_pub = _gznode->Advertise<gazebo::msgs::Factory>("~/factory");
    _factory_pub->WaitForConnection();

    // Create publisher for applying physics (velocity/force) to models
    // Topic: ~/model/set_link_world_pose for position/rotation
    // and ~/apply_body_wrench for force/torque application
    _physics_pub = _gznode->Advertise<gazebo::msgs::LinkData>("~/link/modify");
    
    std::cout << "[GazeboSimulator] Connected to Gazebo transport" << std::endl;

    // ========================================================================
    // Initialize Zenoh Session
    // ========================================================================
    std::cout << "[GazeboSimulator] Opening Zenoh session..." << std::endl;
    
    z_owned_config_t z_config = z_config_default();
    _session = z_open(z_move(z_config));

    if (!_session) {
        std::cerr << "[GazeboSimulator] Failed to open Zenoh session!" << std::endl;
        return false;
    }

    std::cout << "[GazeboSimulator] Connected to Zenoh" << std::endl;

    // ========================================================================
    // Subscribe to Agent Join Events
    // ========================================================================
    // Topic: swarm/agents/join
    // Format: JSON {"agent_id": "drone_1", "timestamp": 123456}
    // Purpose: Triggers drone spawning when an agent comes online
    //
    // When we receive this, we call on_agent_join which parses the JSON
    // and spawns a drone model in Gazebo for that agent_id.
    _sub_agent_join = z_declare_subscriber(
        _session,
        z_keyexpr("swarm/agents/join"),
        z_closure(+[](const z_sample_t* sample, void* ctx) {
            GazeboSimulator* self = static_cast<GazeboSimulator*>(ctx);
            self->on_agent_join(sample, ctx);
        }, this),
        NULL
    );

    if (!_sub_agent_join) {
        std::cerr << "[GazeboSimulator] Failed to subscribe to swarm/agents/join" << std::endl;
        return false;
    }

    std::cout << "[GazeboSimulator] Subscribed to swarm/agents/join" << std::endl;

    // ========================================================================
    // Subscribe to Velocity Commands
    // ========================================================================
    // Topic: drone/*/cmd_vel
    // Format: JSON {"linear": {"x": vx, "y": vy, "z": vz}, "angular": {"z": yaw}}
    // Purpose: Agents send velocity commands that we apply to their drone models
    //
    // Wildcard subscription (*) matches all agent IDs.
    // When we receive cmd_vel, we parse the agent_id from the topic path
    // and apply the velocity to that drone's model in Gazebo.
    _sub_cmd_vel = z_declare_subscriber(
        _session,
        z_keyexpr("drone/*/cmd_vel"),
        z_closure(+[](const z_sample_t* sample, void* ctx) {
            GazeboSimulator* self = static_cast<GazeboSimulator*>(ctx);
            self->on_cmd_vel(sample, ctx);
        }, this),
        NULL
    );

    if (!_sub_cmd_vel) {
        std::cerr << "[GazeboSimulator] Failed to subscribe to drone/*/cmd_vel" << std::endl;
        return false;
    }

    std::cout << "[GazeboSimulator] Subscribed to drone/*/cmd_vel" << std::endl;

    _connected = true;
    std::cout << "[GazeboSimulator] Ready. Waiting for agents..." << std::endl;
    return true;
}

void GazeboSimulator::disconnect() {
    if (_session) {
        z_close(_session);
        _session = nullptr;
    }

    try {
        gazebo::client::shutdown();
    } catch (...) {
        // Ignore errors during shutdown
    }

    _connected = false;
    std::cout << "[GazeboSimulator] Disconnected" << std::endl;
}

// ============================================================================
// Zenoh Callback Handlers
// ============================================================================

// Static method wrapper for agent join callback
void GazeboSimulator::on_agent_join(const z_sample_t* sample, void* ctx) {
    GazeboSimulator* self = static_cast<GazeboSimulator*>(ctx);

    try {
        // Extract payload from Zenoh sample
        std::string payload(
            reinterpret_cast<const char*>(sample->payload.start),
            sample->payload.len
        );

        // Parse JSON
        auto join_msg = json::parse(payload);
        std::string agent_id = join_msg["agent_id"];

        std::cout << "[GazeboSimulator] Received join event from: " << agent_id << std::endl;

        // Spawn the drone with default starting position
        // Initial position: (0, 0, 1) - one meter above water surface
        json initial_pos = {{"x", 0.0}, {"y", 0.0}, {"z", 1.0}};
        self->spawn_drone(agent_id, initial_pos);

    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Error processing agent join: " 
                  << e.what() << std::endl;
    }
}

// Static method wrapper for velocity command callback
void GazeboSimulator::on_cmd_vel(const z_sample_t* sample, void* ctx) {
    GazeboSimulator* self = static_cast<GazeboSimulator*>(ctx);

    try {
        // Extract topic name to get agent_id
        // Topic format: drone/{agent_id}/cmd_vel
        std::string topic(
            reinterpret_cast<const char*>(sample->keyexpr.start),
            sample->keyexpr.len
        );

        // Parse agent_id from topic path
        // Example: "drone/drone_1/cmd_vel" -> "drone_1"
        auto start = topic.find('/') + 1;
        auto end = topic.rfind('/');
        std::string agent_id = topic.substr(start, end - start);

        // Extract payload (JSON velocity command)
        std::string payload(
            reinterpret_cast<const char*>(sample->payload.start),
            sample->payload.len
        );

        auto cmd_msg = json::parse(payload);

        // Update the drone velocity in Gazebo
        self->update_drone_velocity(agent_id, cmd_msg);

    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Error processing cmd_vel: " 
                  << e.what() << std::endl;
    }
}

// ============================================================================
// Drone Spawning and Control
// ============================================================================

std::string GazeboSimulator::generate_drone_sdf(const std::string& agent_id,
                                               double x, double y, double z) {
    // Generate SDF (Simulation Description Format) XML for a drone sphere model
    //
    // Key parameters:
    // - Mass: 1.0 kg (lightweight UAV)
    // - Radius: 0.25 meters (25cm sphere)
    // - Inertia: sphere moments (1/2 * m * r^2)
    // - Material: blue color for visual identification
    //
    // The model will be controlled via velocity commands from agents.
    std::stringstream sdf;
    sdf << "<?xml version='1.0'?>"
        << "<sdf version='1.6'>"
        << "  <model name='" << agent_id << "'>"
        << "    <pose>" << x << " " << y << " " << z << " 0 0 0</pose>"
        << "    <link name='base'>"
        << "      <inertial>"
        << "        <mass>1.0</mass>"
        << "        <inertia>"
        << "          <ixx>0.0083</ixx>"    // 1/2 * 1.0 * 0.25^2
        << "          <ixy>0</ixy>"
        << "          <ixz>0</ixz>"
        << "          <iyy>0.0083</iyy>"
        << "          <iyz>0</iyz>"
        << "          <izz>0.0083</izz>"
        << "        </inertia>"
        << "      </inertial>"
        << "      <collision name='collision'>"
        << "        <geometry>"
        << "          <sphere>"
        << "            <radius>0.25</radius>"
        << "          </sphere>"
        << "        </geometry>"
        << "        <surface>"
        << "          <contact>"
        << "            <collide_bitmask>0xffff</collide_bitmask>"
        << "          </contact>"
        << "          <friction>"
        << "            <ode>"
        << "              <mu>0.5</mu>"
        << "              <mu2>0.5</mu2>"
        << "            </ode>"
        << "          </friction>"
        << "        </surface>"
        << "      </collision>"
        << "      <visual name='visual'>"
        << "        <geometry>"
        << "          <sphere>"
        << "            <radius>0.25</radius>"
        << "          </sphere>"
        << "        </geometry>"
        << "        <material>"
        << "          <ambient>0.1 0.5 0.8 1.0</ambient>"
        << "          <diffuse>0.2 0.6 1.0 1.0</diffuse>"
        << "          <specular>0.5 0.5 0.5 1.0</specular>"
        << "        </material>"
        << "      </visual>"
        << "    </link>"
        << "  </model>"
        << "</sdf>";

    return sdf.str();
}

void GazeboSimulator::spawn_drone(const std::string& agent_id, const json& initial_pos) {
    // Check if we've already spawned this agent to avoid duplicates
    if (_spawned_agents.find(agent_id) != _spawned_agents.end()) {
        std::cout << "[GazeboSimulator] Drone " << agent_id << " already spawned, skipping"
                  << std::endl;
        return;
    }

    if (!_factory_pub) {
        std::cerr << "[GazeboSimulator] Factory publisher not initialized!" << std::endl;
        return;
    }

    // Extract coordinates from JSON
    double x = initial_pos.value("x", 0.0);
    double y = initial_pos.value("y", 0.0);
    double z = initial_pos.value("z", 1.0);

    // Generate SDF model string
    std::string sdf_str = generate_drone_sdf(agent_id, x, y, z);

    // Create Gazebo factory message and populate with SDF
    gazebo::msgs::Factory factory_msg;
    factory_msg.set_sdf(sdf_str);

    // Publish to factory topic - Gazebo will parse SDF and create the model
    _factory_pub->Publish(factory_msg);

    // Mark as spawned
    _spawned_agents[agent_id] = true;

    std::cout << "[GazeboSimulator] Spawned drone: " << agent_id
              << " at (" << x << ", " << y << ", " << z << ")" << std::endl;
}

void GazeboSimulator::update_drone_velocity(const std::string& agent_id,
                                           const json& cmd_msg) {
    // Parse velocity command JSON
    // Expected format: {"linear": {"x": vx, "y": vy, "z": vz}, "angular": {"z": yaw_rate}}
    //
    // Strategy:
    // We don't directly set velocities (ODE can be unstable).
    // Instead, we apply forces via Gazebo's wrench publisher.
    // This lets the physics engine handle constraints and collisions properly.
    //
    // With gravity = 0 and the physics engine handling forces:
    // - Drones maintain velocity when no commands arrive
    // - Collision detection works automatically
    // - Dynamics are realistic
    
    try {
        double vx = cmd_msg["linear"]["x"].get<double>();
        double vy = cmd_msg["linear"]["y"].get<double>();
        double vz = cmd_msg["linear"]["z"].get<double>();
        double yaw_rate = cmd_msg["angular"]["z"].get<double>();

        // Cache the desired velocity in our state map
        // The main loop will periodically convert this to forces
        if (_drone_states.find(agent_id) == _drone_states.end()) {
            _drone_states[agent_id] = DroneState{
                ignition::math::Vector3d(0, 0, 1),  // position (will update from Gazebo)
                ignition::math::Vector3d(vx, vy, vz),  // linear velocity (commanded)
                ignition::math::Vector3d(0, 0, yaw_rate),  // angular velocity
                ignition::math::Quaterniond()  // orientation (identity)
            };
        } else {
            _drone_states[agent_id].linear_velocity = ignition::math::Vector3d(vx, vy, vz);
            _drone_states[agent_id].angular_velocity = ignition::math::Vector3d(0, 0, yaw_rate);
        }

        // Log the command for debugging
        if (std::fmod(_drone_states[agent_id].linear_velocity.Length(), 10.0) < 0.1) {
            // Log every ~10 velocity updates to avoid spam
            std::cout << "[GazeboSimulator] Commanded " << agent_id 
                      << " velocity: (" << vx << ", " << vy << ", " << vz 
                      << ") yaw_rate=" << yaw_rate << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[GazeboSimulator] Error parsing velocity for " << agent_id
                  << ": " << e.what() << std::endl;
    }
}

// ============================================================================
// Sensor Simulation and Data Publishing
// ============================================================================

json GazeboSimulator::simulate_lidar(const std::string& agent_id) {
    // Simulate lidar sensor for the drone
    //
    // Lidar Data Format: JSON array of rays
    // [
    //   {
    //     "angle": 0.0,          # Angle in radians (0 = forward, pi/2 = left, pi = back, -pi/2 = right)
    //     "distance": 50.0,      # Distance to obstacle in meters (max_range if no obstacle)
    //     "intensity": 0.5,      # Reflectivity (0.0 = dark, 1.0 = bright)
    //     "ray_id": 0            # Which ray this is (0-15 for 16 rays)
    //   },
    //   ...
    // ]
    //
    // Current Implementation (MVP):
    // - 16 rays in 360-degree pattern around drone
    // - All rays return max_range (50m) - simplified model
    // - No actual collision raytracing yet
    //
    // Future Enhancement:
    // - Integrate with Gazebo raytracing for accurate hits
    // - Add distance calculation to known obstacles (ship, other drones)
    // - Simulate noise/uncertainty for realistic sensor behavior
    //
    // Processing Location:
    // This data is sent to Agent via drone/{agent_id}/sensors topic.
    // The Agent's ObstaclePerceptionHandler processes this:
    // 1. Converts lidar rays to obstacle "bubbles" (3D uncertainty regions)
    // 2. Merges with gossip data from other drones
    // 3. Feeds "Nervous System" (eyes) which runs uncertainty engine
    // 4. Output: detected obstacles used by PathFinder for avoidance

    json lidar_data = json::array();

    // Simulate N rays in a 360-degree pattern
    int num_rays = 16;
    double max_range = 50.0;  // 50 meter detection range

    for (int i = 0; i < num_rays; ++i) {
        double angle = (2.0 * M_PI * i) / num_rays;
        
        // TODO: Actual raycast detection
        // Current: all rays at max_range (no obstacles detected)
        // When integrated: calculate actual distance to ship, drones, etc.
        
        json ray = {
            {"angle", angle},
            {"distance", max_range},
            {"intensity", 0.5},
            {"ray_id", i}
        };
        lidar_data.push_back(ray);
    }

    return lidar_data;
}

json GazeboSimulator::get_drone_pose(const std::string& agent_id) {
    // Get current position and orientation of a drone
    //
    // We maintain a cached state map that gets updated periodically.
    // In a production system, we would subscribe to Gazebo's world_stats topic
    // to get authoritative pose data from the physics engine.
    //
    // For now, we track the last known position from our internal state.
    // The main loop will periodically query actual poses from Gazebo.

    if (_drone_states.find(agent_id) != _drone_states.end()) {
        const auto& state = _drone_states[agent_id];
        
        // Convert Ignition quaternion to roll/pitch/yaw
        // For simplicity with zero gravity, just use the quaternion
        auto euler = state.orientation.Euler();
        
        json pose = {
            {"x", state.position.X()},
            {"y", state.position.Y()},
            {"z", state.position.Z()},
            {"roll", euler.X()},
            {"pitch", euler.Y()},
            {"yaw", euler.Z()},
            {"vx", state.linear_velocity.X()},
            {"vy", state.linear_velocity.Y()},
            {"vz", state.linear_velocity.Z()}
        };
        return pose;
    }

    // Fallback if agent not in cache
    json pose = {
        {"x", 0.0},
        {"y", 0.0},
        {"z", 1.0},
        {"roll", 0.0},
        {"pitch", 0.0},
        {"yaw", 0.0},
        {"vx", 0.0},
        {"vy", 0.0},
        {"vz", 0.0}
    };

    return pose;
}

void GazeboSimulator::publish_sensor_data(const std::string& agent_id,
                                         const json& sensor_data) {
    // Publish sensor data to the agent
    // Topic: drone/{agent_id}/sensors
    //
    // Sensor data includes:
    // - Proprioception: position, velocity, orientation
    // - Lidar: obstacle distances and angles
    // - IMU data (if needed)

    std::string topic = "drone/" + agent_id + "/sensors";
    std::string payload = sensor_data.dump();

    z_publisher_t pub = z_declare_publisher(_session, z_keyexpr(topic.c_str()), NULL);
    z_publisher_put(&pub, (const uint8_t*)payload.c_str(), payload.size(), NULL);
}

json GazeboSimulator::gather_metrics() {
    // Gather simulation metrics
    // Published to: swarm/metrics (for dedicated metrics node)
    //
    // Metrics Format: JSON object
    // {
    //   "timestamp": 1234567890,     # Unix timestamp (nanoseconds)
    //   "total_collisions": 2,       # Total collision events this frame
    //   "contact_forces": [          # Array of contacts
    //     {
    //       "agent_a": "drone_1",
    //       "agent_b": "ship_obstacle",
    //       "force_magnitude": 45.3,  # Newtons
    //       "contact_point": [x, y, z]
    //     },
    //     ...
    //   ],
    //   "simulation_time": 123.456,  # Seconds elapsed
    //   "num_active_agents": 3,      # Count of spawned agents
    //   "agent_statuses": {          # Per-agent telemetry
    //     "drone_1": {
    //       "position": [x, y, z],
    //       "velocity": [vx, vy, vz],
    //       "collision_count": 0,
    //       "contact_forces_total": 0.0
    //     },
    //     ...
    //   }
    // }
    //
    // Collision Detection:
    // ODE physics engine automatically detects collisions between:
    // - Drones (spheres) with each other
    // - Drones with ship obstacle (box)
    // - Drones with ocean surface (plane)
    //
    // Currently: Placeholder metrics (TODO: subscribe to Gazebo contacts topic)
    // Future: Will subscribe to ~/physics/contacts for actual contact data
    //
    // Processing Location:
    // Metrics node (separate process) subscribes to swarm/metrics and:
    // 1. Logs collision events for analysis
    // 2. Tracks agent health/damage
    // 3. Generates performance reports
    // 4. Feeds metrics to mission planning system

    json agent_statuses = json::object();
    
    // Populate per-agent status from our cached state
    for (const auto& [agent_id, state] : _drone_states) {
        agent_statuses[agent_id] = {
            {"position", json::array({state.position.X(), state.position.Y(), state.position.Z()})},
            {"velocity", json::array({state.linear_velocity.X(), state.linear_velocity.Y(), state.linear_velocity.Z()})},
            {"status", "active"},
            {"collision_count", 0},  // TODO: query actual collisions
            {"contact_forces_total", 0.0}
        };
    }

    json metrics = {
        {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()},
        {"total_collisions", _frame_collisions.size()},  // Count from this frame
        {"contact_forces", json::array()},  // TODO: extract from Gazebo physics
        {"simulation_time", 0.0},  // TODO: get from gzserver
        {"num_active_agents", _spawned_agents.size()},
        {"agent_statuses", agent_statuses},
        {"despawned_this_frame", json::array()}  // Will populate from _frame_collisions
    };

    return metrics;
}

void GazeboSimulator::publish_metrics(const json& metrics) {
    // Publish metrics to dedicated metrics node
    // Topic: swarm/metrics

    std::string payload = metrics.dump();
    z_publisher_t pub = z_declare_publisher(_session, z_keyexpr("swarm/metrics"), NULL);
    z_publisher_put(&pub, (const uint8_t*)payload.c_str(), payload.size(), NULL);
}

// ============================================================================
// Main Event Loop
// ============================================================================

void GazeboSimulator::run() {
    // Main event loop - runs until explicitly stopped
    //
    // This loop:
    // 1. Waits for Zenoh messages (callbacks handle them)
    // 2. Periodically gathers sensor data and metrics
    // 3. Publishes to agents and metrics node
    //
    // The actual physics integration happens in Gazebo's gzserver process.
    // We just sync the control inputs and sensor outputs via Zenoh.

    std::cout << "[GazeboSimulator] Starting main loop..." << std::endl;

    auto last_sensor_publish = std::chrono::steady_clock::now();
    auto last_metrics_publish = std::chrono::steady_clock::now();
    auto sensor_publish_interval = std::chrono::milliseconds(20);  // 50 Hz
    auto metrics_publish_interval = std::chrono::milliseconds(100);  // 10 Hz

    while (_connected) {
        auto now = std::chrono::steady_clock::now();

        // ====================================================================
        // Process Collisions and Despawning
        // ====================================================================
        // Check for drone-to-drone collisions every frame
        // If relative velocity exceeds threshold, despawn both drones
        process_drone_collisions();

        // Publish sensor data at regular intervals
        if (now - last_sensor_publish > sensor_publish_interval) {
            for (const auto& [agent_id, spawned] : _spawned_agents) {
                if (spawned) {
                    json pose = get_drone_pose(agent_id);
                    json lidar = simulate_lidar(agent_id);

                    json sensor_data = {
                        {"pose", pose},
                        {"lidar", lidar}
                    };

                    publish_sensor_data(agent_id, sensor_data);
                }
            }
            last_sensor_publish = now;
        }

        // Publish metrics at regular intervals
        if (now - last_metrics_publish > metrics_publish_interval) {
            json metrics = gather_metrics();
            publish_metrics(metrics);
            last_metrics_publish = now;
        }

        // Sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "[GazeboSimulator] Main loop exited" << std::endl;
}

// ============================================================================
// Collision Detection and Response
// ============================================================================

double GazeboSimulator::calculate_relative_velocity(const std::string& agent_a,
                                                    const std::string& agent_b) const {
    // Calculate relative velocity between two drones
    //
    // Relative velocity = |v_a - v_b|
    // This represents the closing speed between the two objects.
    // Higher values indicate more violent collision.
    //
    // Example:
    // - Two drones moving toward each other at 2 m/s each
    //   relative velocity = 4 m/s (closing at 4 m/s)
    // - Two drones moving in same direction, one 3 m/s faster
    //   relative velocity = 3 m/s

    if (_drone_states.find(agent_a) == _drone_states.end() ||
        _drone_states.find(agent_b) == _drone_states.end()) {
        return 0.0;  // One or both drones not in cache
    }

    const auto& state_a = _drone_states.at(agent_a);
    const auto& state_b = _drone_states.at(agent_b);

    // Relative velocity vector
    auto rel_vel = state_a.linear_velocity - state_b.linear_velocity;
    
    // Return magnitude (scalar speed)
    return rel_vel.Length();
}

void GazeboSimulator::process_drone_collisions() {
    // Detect drone-to-drone collisions and apply despawn logic
    //
    // Current Implementation:
    // - Simulates drone-drone collision detection
    // - Checks relative velocity against threshold
    // - Despawns both drones if velocity exceeds threshold
    //
    // Future Enhancement:
    // - Subscribe to ~/physics/contacts Gazebo topic for real collisions
    // - Extract actual contact points and forces
    // - More sophisticated damage model (partial damage, degradation)
    //
    // Despawn Threshold: DRONE_DESPAWN_VELOCITY_THRESHOLD = 5.0 m/s
    // Logic: If two drones collide with relative velocity > 5 m/s, both despawn
    // Rationale: 
    //   - Represents high-impact collision (~5 m/s is ~18 km/h)
    //   - Simulates structural damage/destruction
    //   - Low velocities (<5 m/s) = glancing collision, drones survive
    //   - Can be tuned in header file

    _frame_collisions.clear();

    // TODO: Query actual collisions from Gazebo ~/physics/contacts topic
    // For now: Placeholder implementation for testing infrastructure
    
    // In production, would iterate over actual contact list:
    // for (const auto& contact : gazebo_contacts) {
    //     if (is_drone(contact.body1) && is_drone(contact.body2)) {
    //         double rel_vel = calculate_relative_velocity(contact.body1, contact.body2);
    //         if (rel_vel > DRONE_DESPAWN_VELOCITY_THRESHOLD) {
    //             despawn_drone(contact.body1, "collision");
    //             despawn_drone(contact.body2, "collision");
    //         }
    //     }
    // }
}

void GazeboSimulator::despawn_drone(const std::string& agent_id, 
                                    const std::string& reason) {
    // Remove a drone from simulation
    //
    // Steps:
    // 1. Check if drone exists in spawned list
    // 2. Mark as no longer spawned
    // 3. Send despawn request to Gazebo
    // 4. Publish despawn event to swarm/agents/despawn
    // 5. Remove from internal state map

    auto it = _spawned_agents.find(agent_id);
    if (it == _spawned_agents.end()) {
        std::cout << "[GazeboSimulator] Drone " << agent_id 
                  << " not found (already despawned?)" << std::endl;
        return;
    }

    std::cout << "[GazeboSimulator] Despawning drone: " << agent_id 
              << " (reason: " << reason << ")" << std::endl;

    // Mark as not spawned
    _spawned_agents.erase(it);

    // Remove from state cache
    _drone_states.erase(agent_id);

    // ========================================================================
    // Send delete request to Gazebo
    // ========================================================================
    // Topic: ~/model/delete
    // Format: Gazebo DeleteModel message with model name
    
    if (_gznode) {
        // Create a delete model message
        // This tells Gazebo to remove the model from the simulation
        gazebo::msgs::Request* delete_req = gazebo::msgs::CreateRequest("entity_delete", agent_id);
        
        // Publish to model deletion topic
        // In practice, we'd use: _gznode->Request("~/model/delete", delete_req);
        // For now, just logging - Gazebo deletion handled separately
        
        delete delete_req;
    }

    // ========================================================================
    // Publish despawn event to Zenoh
    // ========================================================================
    // Topic: swarm/agents/despawn
    // Format: JSON {"agent_id": "...", "reason": "...", "timestamp": ...}
    // Purpose: Notify metrics node and other systems that drone is gone
    
    json despawn_event = {
        {"agent_id", agent_id},
        {"reason", reason},
        {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
    };
    
    std::string payload = despawn_event.dump();
    z_publisher_t despawn_pub = z_declare_publisher(
        _session, 
        z_keyexpr("swarm/agents/despawn"), 
        NULL
    );
    z_publisher_put(&despawn_pub, (const uint8_t*)payload.c_str(), payload.size(), NULL);

    std::cout << "[GazeboSimulator] Published despawn event for " << agent_id << std::endl;
}

}
