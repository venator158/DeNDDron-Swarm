#include "MetricsNode.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>

namespace denddron {

// ============================================================================
// Constructor and Lifecycle
// ============================================================================

MetricsNode::MetricsNode()
    : _connected(false), _session(nullptr) {
    std::cout << "[MetricsNode] Initializing..." << std::endl;
    
    // Initialize swarm statistics
    _swarm_stats.start_time = std::chrono::system_clock::now().time_since_epoch().count();
    _swarm_stats.total_spawned = 0;
    _swarm_stats.total_despawned = 0;
    _swarm_stats.current_active_count = 0;
    _swarm_stats.total_collisions = 0;
    _swarm_stats.last_update_time = _swarm_stats.start_time;
}

MetricsNode::~MetricsNode() {
    disconnect();
}

bool MetricsNode::connect() {
    std::cout << "[MetricsNode] Connecting to Zenoh..." << std::endl;
    
    z_owned_config_t z_config = z_config_default();
    _session = z_open(z_move(z_config));

    if (!_session) {
        std::cerr << "[MetricsNode] Failed to open Zenoh session!" << std::endl;
        return false;
    }

    std::cout << "[MetricsNode] Connected to Zenoh" << std::endl;

    // ========================================================================
    // Subscribe to Metrics Topic
    // ========================================================================
    // Topic: swarm/metrics
    // Published by: GazeboSimulator (every 100ms)
    // Format: JSON with collision data, per-agent status, etc.
    // 
    // This is the primary data source for swarm telemetry.
    // Every update tells us:
    // - Active agent count
    // - Position/velocity of each agent
    // - Collision count for each agent
    
    _sub_metrics = z_declare_subscriber(
        _session,
        z_keyexpr("swarm/metrics"),
        z_closure(+[](const z_sample_t* sample, void* ctx) {
            MetricsNode* self = static_cast<MetricsNode*>(ctx);
            self->on_metrics_update(sample, ctx);
        }, this),
        NULL
    );

    if (!_sub_metrics) {
        std::cerr << "[MetricsNode] Failed to subscribe to swarm/metrics" << std::endl;
        return false;
    }

    std::cout << "[MetricsNode] Subscribed to swarm/metrics" << std::endl;

    // ========================================================================
    // Subscribe to Despawn Events
    // ========================================================================
    // Topic: swarm/agents/despawn
    // Published by: GazeboSimulator when drone collides at high velocity
    // Format: JSON {"agent_id": "...", "reason": "collision", "timestamp": ...}
    // 
    // Tells us when drones are destroyed.
    // Allows tracking of drone lifecycle (birth to death).

    _sub_despawn = z_declare_subscriber(
        _session,
        z_keyexpr("swarm/agents/despawn"),
        z_closure(+[](const z_sample_t* sample, void* ctx) {
            MetricsNode* self = static_cast<MetricsNode*>(ctx);
            self->on_agent_despawn(sample, ctx);
        }, this),
        NULL
    );

    if (!_sub_despawn) {
        std::cerr << "[MetricsNode] Failed to subscribe to swarm/agents/despawn" << std::endl;
        return false;
    }

    std::cout << "[MetricsNode] Subscribed to swarm/agents/despawn" << std::endl;

    // ========================================================================
    // Subscribe to Join Events
    // ========================================================================
    // Topic: swarm/agents/join
    // Published by: Agent when it starts up
    // Format: JSON {"agent_id": "...", "timestamp": ...}
    // 
    // Tells us when drones spawn.
    // Allows tracking of drone lifecycle from birth.

    _sub_join = z_declare_subscriber(
        _session,
        z_keyexpr("swarm/agents/join"),
        z_closure(+[](const z_sample_t* sample, void* ctx) {
            MetricsNode* self = static_cast<MetricsNode*>(ctx);
            self->on_agent_join(sample, ctx);
        }, this),
        NULL
    );

    if (!_sub_join) {
        std::cerr << "[MetricsNode] Failed to subscribe to swarm/agents/join" << std::endl;
        return false;
    }

    std::cout << "[MetricsNode] Subscribed to swarm/agents/join" << std::endl;

    _connected = true;
    std::cout << "[MetricsNode] Ready. Listening for metrics..." << std::endl;
    return true;
}

void MetricsNode::disconnect() {
    if (_session) {
        z_close(_session);
        _session = nullptr;
    }

    _connected = false;
    std::cout << "[MetricsNode] Disconnected" << std::endl;
}

// ============================================================================
// Zenoh Callback Handlers
// ============================================================================

void MetricsNode::on_metrics_update(const z_sample_t* sample, void* ctx) {
    MetricsNode* self = static_cast<MetricsNode*>(ctx);

    try {
        std::string payload(
            reinterpret_cast<const char*>(sample->payload.start),
            sample->payload.len
        );

        auto metrics_msg = json::parse(payload);
        self->process_metrics(metrics_msg);

    } catch (const std::exception& e) {
        std::cerr << "[MetricsNode] Error processing metrics: " << e.what() << std::endl;
    }
}

void MetricsNode::on_agent_despawn(const z_sample_t* sample, void* ctx) {
    MetricsNode* self = static_cast<MetricsNode*>(ctx);

    try {
        std::string payload(
            reinterpret_cast<const char*>(sample->payload.start),
            sample->payload.len
        );

        auto despawn_msg = json::parse(payload);
        self->process_despawn(despawn_msg);

    } catch (const std::exception& e) {
        std::cerr << "[MetricsNode] Error processing despawn: " << e.what() << std::endl;
    }
}

void MetricsNode::on_agent_join(const z_sample_t* sample, void* ctx) {
    MetricsNode* self = static_cast<MetricsNode*>(ctx);

    try {
        std::string payload(
            reinterpret_cast<const char*>(sample->payload.start),
            sample->payload.len
        );

        auto join_msg = json::parse(payload);
        self->process_join(join_msg);

    } catch (const std::exception& e) {
        std::cerr << "[MetricsNode] Error processing join: " << e.what() << std::endl;
    }
}

// ============================================================================
// Metrics Processing
// ============================================================================

void MetricsNode::process_metrics(const json& metrics_msg) {
    // Process incoming metrics update from Gazebo
    //
    // Expected format:
    // {
    //   "timestamp": 123456789,
    //   "total_collisions": 2,
    //   "num_active_agents": 3,
    //   "agent_statuses": {
    //     "drone_1": {
    //       "position": [x, y, z],
    //       "velocity": [vx, vy, vz],
    //       "collision_count": 0,
    //       "contact_forces_total": 12.5
    //     },
    //     ...
    //   }
    // }

    try {
        uint64_t timestamp = metrics_msg["timestamp"].get<uint64_t>();
        int total_collisions = metrics_msg["total_collisions"].get<int>();
        int num_active = metrics_msg["num_active_agents"].get<int>();

        // Update global stats
        _swarm_stats.last_update_time = timestamp;
        _swarm_stats.total_collisions = total_collisions;
        _swarm_stats.current_active_count = num_active;

        // Process per-agent data
        if (metrics_msg.contains("agent_statuses")) {
            const auto& statuses = metrics_msg["agent_statuses"];
            
            for (auto it = statuses.begin(); it != statuses.end(); ++it) {
                std::string agent_id = it.key();
                const auto& status = it.value();

                // Initialize agent if not in history
                if (_agent_history.find(agent_id) == _agent_history.end()) {
                    AgentMetrics agent_metrics;
                    agent_metrics.agent_id = agent_id;
                    agent_metrics.birth_time = timestamp;
                    agent_metrics.death_time = 0;
                    agent_metrics.is_alive = true;
                    agent_metrics.collision_count = 0;
                    agent_metrics.despawn_count = 0;
                    _agent_history[agent_id] = agent_metrics;
                    
                    std::cout << "[MetricsNode] Tracking new agent: " << agent_id << std::endl;
                }

                // Update agent metrics
                auto& agent = _agent_history[agent_id];
                
                if (status.contains("position")) {
                    const auto& pos = status["position"];
                    AgentMetrics::Position position;
                    position.x = pos[0].get<double>();
                    position.y = pos[1].get<double>();
                    position.z = pos[2].get<double>();
                    position.timestamp = timestamp;
                    
                    // Sample position history (every 10th update to save space)
                    if (agent.position_history.empty() || 
                        (timestamp - agent.position_history.back().timestamp) > 100000000) {  // ~100ms
                        agent.position_history.push_back(position);
                    }
                }

                if (status.contains("velocity")) {
                    const auto& vel = status["velocity"];
                    agent.last_velocity_x = vel[0].get<double>();
                    agent.last_velocity_y = vel[1].get<double>();
                    agent.last_velocity_z = vel[2].get<double>();
                }

                if (status.contains("collision_count")) {
                    agent.collision_count = status["collision_count"].get<int>();
                }
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[MetricsNode] Error parsing metrics JSON: " << e.what() << std::endl;
    }
}

void MetricsNode::process_despawn(const json& despawn_msg) {
    // Process drone despawn event
    //
    // Expected format:
    // {
    //   "agent_id": "drone_1",
    //   "reason": "collision",
    //   "timestamp": 123456789
    // }

    try {
        std::string agent_id = despawn_msg["agent_id"].get<std::string>();
        std::string reason = despawn_msg["reason"].get<std::string>();
        uint64_t timestamp = despawn_msg["timestamp"].get<uint64_t>();

        // Log despawn event
        DespawnEvent event;
        event.timestamp = timestamp;
        event.agent_id = agent_id;
        event.reason = reason;
        _despawn_log.push_back(event);

        // Update agent metrics
        if (_agent_history.find(agent_id) != _agent_history.end()) {
            auto& agent = _agent_history[agent_id];
            agent.death_time = timestamp;
            agent.is_alive = false;
            agent.despawn_reason = reason;
            agent.despawn_count++;

            std::cout << "[MetricsNode] Agent despawned: " << agent_id 
                      << " (reason: " << reason << ")" << std::endl;
        } else {
            std::cout << "[MetricsNode] Despawn for unknown agent: " << agent_id << std::endl;
        }

        // Update global stats
        _swarm_stats.total_despawned++;

    } catch (const std::exception& e) {
        std::cerr << "[MetricsNode] Error parsing despawn JSON: " << e.what() << std::endl;
    }
}

void MetricsNode::process_join(const json& join_msg) {
    // Process drone join event
    //
    // Expected format:
    // {
    //   "agent_id": "drone_1",
    //   "timestamp": 123456789
    // }

    try {
        std::string agent_id = join_msg["agent_id"].get<std::string>();
        uint64_t timestamp = join_msg["timestamp"].get<uint64_t>();

        // Initialize agent metrics
        AgentMetrics agent_metrics;
        agent_metrics.agent_id = agent_id;
        agent_metrics.birth_time = timestamp;
        agent_metrics.death_time = 0;
        agent_metrics.is_alive = true;
        agent_metrics.collision_count = 0;
        agent_metrics.despawn_count = 0;

        _agent_history[agent_id] = agent_metrics;
        _swarm_stats.total_spawned++;

        std::cout << "[MetricsNode] Agent joined: " << agent_id << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[MetricsNode] Error parsing join JSON: " << e.what() << std::endl;
    }
}

// ============================================================================
// Summary and Reporting
// ============================================================================

json MetricsNode::get_summary_stats() const {
    // Calculate elapsed time
    uint64_t elapsed = _swarm_stats.last_update_time - _swarm_stats.start_time;
    double elapsed_seconds = elapsed / 1e9;

    json summary = {
        {"elapsed_seconds", elapsed_seconds},
        {"total_spawned", _swarm_stats.total_spawned},
        {"total_despawned", _swarm_stats.total_despawned},
        {"current_active_agents", _swarm_stats.current_active_count},
        {"total_collisions_detected", _swarm_stats.total_collisions},
        {"survival_rate_percent", 
         (_swarm_stats.total_spawned > 0) ? 
         (100.0 * _swarm_stats.current_active_count / _swarm_stats.total_spawned) : 0.0},
        {"despawn_events", _despawn_log.size()},
        {"collision_events", _collision_log.size()}
    };

    return summary;
}

json MetricsNode::get_agent_history() const {
    json history = json::object();

    for (const auto& [agent_id, metrics] : _agent_history) {
        json agent_data = {
            {"agent_id", metrics.agent_id},
            {"birth_timestamp", metrics.birth_time},
            {"death_timestamp", metrics.death_time},
            {"is_alive", metrics.is_alive},
            {"despawn_reason", metrics.despawn_reason},
            {"collision_count", metrics.collision_count},
            {"despawn_count", metrics.despawn_count},
            {"last_known_velocity", {
                {"vx", metrics.last_velocity_x},
                {"vy", metrics.last_velocity_y},
                {"vz", metrics.last_velocity_z}
            }},
            {"position_samples", json::array()}
        };

        // Add position history
        for (const auto& pos : metrics.position_history) {
            agent_data["position_samples"].push_back({
                {"timestamp", pos.timestamp},
                {"x", pos.x},
                {"y", pos.y},
                {"z", pos.z}
            });
        }

        history[agent_id] = agent_data;
    }

    return history;
}

void MetricsNode::print_summary() const {
    auto summary = get_summary_stats();

    std::cout << "\n";
    std::cout << "========================================" << std::endl;
    std::cout << "SWARM METRICS SUMMARY" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Elapsed Time: " << summary["elapsed_seconds"].get<double>() << "s" << std::endl;
    std::cout << "Total Spawned: " << summary["total_spawned"].get<int>() << std::endl;
    std::cout << "Total Despawned: " << summary["total_despawned"].get<int>() << std::endl;
    std::cout << "Active Agents: " << summary["current_active_agents"].get<int>() << std::endl;
    std::cout << "Survival Rate: " << std::fixed << std::setprecision(1) 
              << summary["survival_rate_percent"].get<double>() << "%" << std::endl;
    std::cout << "Total Collisions: " << summary["total_collisions_detected"].get<int>() << std::endl;
    std::cout << "Despawn Events: " << summary["despawn_events"].get<int>() << std::endl;
    std::cout << "========================================\n" << std::endl;
}

void MetricsNode::write_metrics_log(const std::string& filename) const {
    // Write detailed metrics to file for post-analysis
    std::ofstream log_file(filename);

    if (!log_file.is_open()) {
        std::cerr << "[MetricsNode] Failed to open log file: " << filename << std::endl;
        return;
    }

    // Write summary
    log_file << "# DeNDDron Swarm Simulation Metrics" << std::endl;
    log_file << "# Generated: " << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
    log_file << std::endl;

    auto summary = get_summary_stats();
    log_file << "## Summary" << std::endl;
    log_file << summary.dump(2) << std::endl;
    log_file << std::endl;

    // Write despawn log
    log_file << "## Despawn Events" << std::endl;
    for (const auto& event : _despawn_log) {
        log_file << "- Agent: " << event.agent_id 
                 << ", Reason: " << event.reason 
                 << ", Time: " << event.timestamp << std::endl;
    }
    log_file << std::endl;

    // Write agent history
    log_file << "## Agent History" << std::endl;
    auto history = get_agent_history();
    log_file << history.dump(2) << std::endl;

    log_file.close();
    std::cout << "[MetricsNode] Metrics written to: " << filename << std::endl;
}

// ============================================================================
// Main Event Loop
// ============================================================================

void MetricsNode::run() {
    std::cout << "[MetricsNode] Starting metrics collection..." << std::endl;

    auto last_print = std::chrono::steady_clock::now();
    auto print_interval = std::chrono::seconds(5);  // Print summary every 5 seconds

    while (_connected) {
        auto now = std::chrono::steady_clock::now();

        // Print summary periodically
        if (now - last_print > print_interval) {
            print_summary();
            last_print = now;
        }

        // Sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Write final metrics to file on exit
    write_metrics_log("/tmp/denddron_metrics.json");
    std::cout << "[MetricsNode] Event loop exited" << std::endl;
}

}
