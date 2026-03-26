#include "GazeboSimulator.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

// Global flag for graceful shutdown
std::atomic<bool> g_running(true);

// Signal handler for Ctrl+C
void signal_handler(int signum) {
    std::cout << "\n[Main] Received signal " << signum << ", shutting down..." << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "DeNDDron Swarm - Gazebo Simulator" << std::endl;
    std::cout << "========================================" << std::endl;

    // Register signal handlers for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // ========================================================================
    // Initialize Gazebo Simulator Bridge
    // ========================================================================
    denddron::GazeboSimulator simulator;

    // Connect to Gazebo and Zenoh
    if (!simulator.connect()) {
        std::cerr << "[Main] Failed to connect to Gazebo simulator!" << std::endl;
        std::cerr << "[Main] Make sure gzserver is running in this container" << std::endl;
        return 1;
    }

    std::cout << "[Main] Simulator bridge initialized successfully" << std::endl;
    std::cout << "[Main] Waiting for agents to join..." << std::endl;

    // ========================================================================
    // Main Event Loop
    // ========================================================================
    // The simulator runs in the main thread.
    // Zenoh callbacks will fire when messages arrive on subscribed topics.
    // The simulator publishes sensor data and metrics at fixed intervals.
    simulator.run();

    // ========================================================================
    // Shutdown
    // ========================================================================
    std::cout << "[Main] Shutting down..." << std::endl;
    simulator.disconnect();

    std::cout << "[Main] Goodbye!" << std::endl;
    return 0;
}
