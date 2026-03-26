#include "core/Agent.hpp"
#include <iostream>
#include <csignal>
#include <memory>

using namespace denddron;

static std::unique_ptr<Agent> g_agent = nullptr;

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\n[Main] Shutdown signal received" << std::endl;
        if (g_agent) {
            g_agent->stop();
        }
    }
}

int main(int argc, char** argv) {
    // Parse command line arguments
    AgentConfig config;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--agent-id" && i + 1 < argc) {
            config.agent_id = argv[++i];
        } else if (arg == "--zenoh-router" && i + 1 < argc) {
            config.zenoh_router_ip = argv[++i];
        } else if (arg == "--loop-rate" && i + 1 < argc) {
            config.loop_rate_hz = std::stof(argv[++i]);
        }
    }

    // Validate configuration
    if (config.agent_id.empty()) {
        std::cerr << "[Main] Error: --agent-id required" << std::endl;
        return 1;
    }

    std::cout << "[Main] Starting DeNDDron Agent" << std::endl;
    std::cout << "[Main]   Agent ID: " << config.agent_id << std::endl;
    std::cout << "[Main]   Zenoh Router: " << config.zenoh_router_ip << std::endl;
    std::cout << "[Main]   Loop Rate: " << config.loop_rate_hz << " Hz" << std::endl;

    // Setup signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        // Create and run agent
        g_agent = std::make_unique<Agent>(config);
        g_agent->run();
    } catch (const std::exception& e) {
        std::cerr << "[Main] Fatal error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "[Main] Agent stopped cleanly" << std::endl;
    return 0;
}
