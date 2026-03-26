#pragma once

#include <nlohmann/json.hpp>
#include <zenoh.h>
#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <chrono>

using json = nlohmann::json;

namespace denddron {

/**
 * @class MetricsNode
 * @brief Centralized metrics collection and analysis for swarm simulation
 * 
 * Responsibilities:
 * 1. Subscribe to swarm/metrics - collision and status data from Gazebo
 * 2. Subscribe to swarm/agents/despawn - drone death notifications
 * 3. Subscribe to swarm/agents/join - drone birth notifications
 * 4. Maintain swarm-wide telemetry:
 *    - Active agent count
 *    - Total collisions (cumulative)
 *    - Per-agent collision history
 *    - Despawned agent log
 * 5. Publish analysis to swarm/metrics/analysis (optional)
 * 6. Log events to file for post-simulation analysis
 * 
 * Data Flow:
 *   Gazebo publishes metrics every 100ms
 *        ↓
 *   MetricsNode receives and processes
 *        ↓
 *   Updates internal state and statistics
 *        ↓
 *   Logs to file for analysis
 */
class MetricsNode {
public:
    /**
     * @brief Constructor - initializes metrics collection
     */
    MetricsNode();

    /**
     * @brief Destructor - cleanup
     */
    ~MetricsNode();

    /**
     * @brief Connect to Zenoh and start subscribing to metrics topics
     * @return true if connection successful
     */
    bool connect();

    /**
     * @brief Disconnect and shutdown
     */
    void disconnect();

    /**
     * @brief Main event loop - process incoming metrics
     */
    void run();

    /**
     * @brief Check if node is connected
     */
    bool is_connected() const { return _connected; }

    /**
     * @brief Get summary statistics
     * @return JSON with swarm-wide metrics
     */
    json get_summary_stats() const;

    /**
     * @brief Get per-agent detailed history
     * @return JSON with individual agent telemetry
     */
    json get_agent_history() const;

private:
    // Connection state
    bool _connected;
    z_session_t _session;

    // Zenoh subscribers
    z_owned_subscriber_t _sub_metrics;      // swarm/metrics
    z_owned_subscriber_t _sub_despawn;      // swarm/agents/despawn
    z_owned_subscriber_t _sub_join;         // swarm/agents/join

    // ========================================================================
    // Metrics State Tracking
    // ========================================================================

    /**
     * @brief Per-agent telemetry history
     */
    struct AgentMetrics {
        std::string agent_id;
        uint64_t birth_time;         // When spawned
        uint64_t death_time;         // When despawned (0 if alive)
        bool is_alive;
        
        struct Position {
            double x, y, z;
            uint64_t timestamp;
        };
        std::vector<Position> position_history;  // Sample every few frames
        
        int collision_count;
        int despawn_count;
        std::string despawn_reason;
        
        double last_velocity_x, last_velocity_y, last_velocity_z;
    };

    std::map<std::string, AgentMetrics> _agent_history;

    /**
     * @brief Global swarm statistics
     */
    struct SwarmStats {
        uint64_t start_time;
        int total_spawned;          // Cumulative agents spawned
        int total_despawned;        // Cumulative agents despawned
        int current_active_count;
        int total_collisions;       // Cumulative drone-drone collisions
        uint64_t last_update_time;
    };
    SwarmStats _swarm_stats;

    // Event logs for analysis
    struct CollisionEvent {
        uint64_t timestamp;
        std::string agent_a;
        std::string agent_b;
        double relative_velocity;
    };
    std::vector<CollisionEvent> _collision_log;

    struct DespawnEvent {
        uint64_t timestamp;
        std::string agent_id;
        std::string reason;
    };
    std::vector<DespawnEvent> _despawn_log;

    // ========================================================================
    // Callback Handlers
    // ========================================================================

    /**
     * @brief Callback when metrics data received
     * Parses swarm/metrics and updates internal state
     */
    static void on_metrics_update(const z_sample_t* sample, void* ctx);

    /**
     * @brief Callback when agent despawns
     * Records despawn event and updates agent status
     */
    static void on_agent_despawn(const z_sample_t* sample, void* ctx);

    /**
     * @brief Callback when agent joins
     * Records spawn event and initializes agent metrics
     */
    static void on_agent_join(const z_sample_t* sample, void* ctx);

    // ========================================================================
    // Internal Processing
    // ========================================================================

    /**
     * @brief Process incoming metrics update
     * Extracts per-agent status and collision data
     */
    void process_metrics(const json& metrics_msg);

    /**
     * @brief Process despawn event
     * Updates agent status and logs death
     */
    void process_despawn(const json& despawn_msg);

    /**
     * @brief Process join event
     * Initializes new agent in metrics tracking
     */
    void process_join(const json& join_msg);

    /**
     * @brief Write metrics to log file
     * For post-simulation analysis
     */
    void write_metrics_log(const std::string& filename) const;

    /**
     * @brief Print summary to console
     * Human-readable status update
     */
    void print_summary() const;
};

}
