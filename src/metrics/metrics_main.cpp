#include "MetricsNode.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

static std::atomic<bool> running(true);

void signal_handler(int signal) {
    std::cout << "\n[metrics_main] Received signal " << signal << ", shutting down..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    // =========================================================================
    // Metrics Node - System Observer for DeNDDron Swarm
    // =========================================================================
    //
    // This node subscribes to three Zenoh topics:
    // 1. swarm/metrics       - Periodic telemetry from GazeboSimulator
    // 2. swarm/agents/join   - When agents spawn
    // 3. swarm/agents/despawn - When agents are destroyed
    //
    // It aggregates this data into:
    // - Per-agent metrics (position history, collision count, lifetime)
    // - Swarm-wide statistics (total spawned, despawned, active count, survival rate)
    // - Event logs (all despawn events with reasons and timestamps)
    //
    // Every 5 seconds it prints a summary. On exit, it writes full metrics to:
    // /tmp/denddron_metrics.json
    //
    // This is essential for:
    // - Debugging swarm behavior
    // - Analyzing collision patterns
    // - Validating physics simulation
    // - Measuring drone survivability
    // - Post-sim analysis and visualization
    //
    // =========================================================================

    std::cout << "========================================" << std::endl;
    std::cout << "DeNDDron Swarm - Metrics Node" << std::endl;
    std::cout << "========================================" << std::endl;

    // Register signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Create metrics node
    denddron::MetricsNode metrics_node;

    // Connect to Zenoh
    if (!metrics_node.connect()) {
        std::cerr << "[metrics_main] Failed to connect metrics node!" << std::endl;
        return 1;
    }

    std::cout << "[metrics_main] Metrics node ready" << std::endl;
    std::cout << "[metrics_main] Listening for events from:" << std::endl;
    std::cout << "  - swarm/metrics" << std::endl;
    std::cout << "  - swarm/agents/join" << std::endl;
    std::cout << "  - swarm/agents/despawn" << std::endl;
    std::cout << "[metrics_main] Press Ctrl+C to exit" << std::endl;
    std::cout << std::endl;

    // Run metrics collection loop
    // This will:
    // - Receive Zenoh events asynchronously via subscribers
    // - Print summary every 5 seconds
    // - Write metrics log on exit
    // - Handle Ctrl+C gracefully
    while (running) {
        metrics_node.run();
        break;  // run() loops internally until !_connected
    }

    metrics_node.disconnect();
    std::cout << "[metrics_main] Metrics node shutdown complete" << std::endl;

    return 0;
}
