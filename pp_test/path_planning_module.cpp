#include "path_planning_interface.hpp"
#include "path_planning_strategy.hpp"
#include "strategies/rrt_strategy.hpp"
#include "strategies/astar_strategy.hpp"
#include "strategies/apf_strategy.hpp"
#include "strategies/apf_mapf_strategy.hpp"
#include <iostream>
#include <memory>

namespace PathPlanning {
    
    static Config currentConfig;
    static std::unique_ptr<PathPlanningStrategy> currentStrategy;
    
    // Forward declaration of helper function
    bool shouldReplan(const Environment& environment, const std::vector<glm::vec3>& currentPath, float deltaTime);
    
    void initialize() {
        std::cout << "Path Planning Module: Initialized" << std::endl;
        std::cout << "Available algorithms: APF, APF-MAPF, A*, RRT, Custom" << std::endl;
        
        // Initialize with default strategy (APF MAPF)
        currentStrategy = std::make_unique<APFStrategy>();
        currentStrategy->initialize();
    }
    
    void shutdown() {
        std::cout << "Path Planning Module: Shutdown" << std::endl;
        if (currentStrategy) {
            currentStrategy->shutdown();
            currentStrategy.reset();
        }
    }
    
    void setStrategy(Config::Algorithm algorithm) {
        if (currentStrategy) {
            currentStrategy->shutdown();
        }
        
        switch (algorithm) {
            case Config::Algorithm::RRT:
                currentStrategy = std::make_unique<RRTStrategy>();
                break;
            case Config::Algorithm::AStar:
                currentStrategy = std::make_unique<AStarStrategy>();
                break;
            case Config::Algorithm::APF:
                currentStrategy = std::make_unique<APFStrategy>();
                break;
            case Config::Algorithm::APF_MAPF:
                currentStrategy = std::make_unique<APFMAPFStrategy>();
                break;
            default:
                currentStrategy = std::make_unique<APFStrategy>();
                break;
        }
        
        currentStrategy->initialize();
        currentStrategy->configure(currentConfig);
        std::cout << "Switched to algorithm: " << currentStrategy->getAlgorithmName() << std::endl;
    }
    
    std::vector<glm::vec3> planPath(const Environment& environment) {
        std::cout << "=== PATH PLANNING ENTRY POINT ===" << std::endl;
        std::cout << "Using algorithm: " << (currentStrategy ? currentStrategy->getAlgorithmName() : "None") << std::endl;
        std::cout << "Agent Start: (" << environment.agentStart.x << ", " 
                  << environment.agentStart.y << ", " << environment.agentStart.z << ")" << std::endl;
        std::cout << "Goal Position: (" << environment.goalPosition.x << ", " 
                  << environment.goalPosition.y << ", " << environment.goalPosition.z << ")" << std::endl;
        std::cout << "Agent Radius: " << environment.agentRadius << std::endl;
        std::cout << "World Bounds Available: " << (environment.worldBounds.min.x != 0.0f ? "Yes" : "Check") << std::endl;
        std::cout << "Number of Obstacles: " << environment.obstacles.size() << std::endl;
        
        if (!currentStrategy) {
            std::cout << "Error: No strategy selected! Using fallback." << std::endl;
            // Fallback to direct path
            std::vector<glm::vec3> path;
            path.push_back(environment.goalPosition);
            return path;
        }
        
        return currentStrategy->planPath(environment);
    }
    
    
    std::vector<std::vector<glm::vec3>> planMultiplePaths(const std::vector<Environment>& environments) {
        std::cout << "Path Planning Module: Planning paths for " << environments.size() << " agents" << std::endl;
        
        std::vector<std::vector<glm::vec3>> paths;
        paths.reserve(environments.size());
        
        if (currentConfig.enableMAPF && environments.size() > 1 && currentStrategy) {
            // Multi-agent coordination using the selected strategy
            std::cout << "MAPF: Using " << currentStrategy->getAlgorithmName() << " with coordination" << std::endl;
            std::cout << "MAPF: Strategy supports multi-agent: " << (currentStrategy->supportsMultiAgent() ? "YES" : "NO") << std::endl;
            
            if (currentStrategy->supportsMultiAgent()) {
                // Use strategy's native multi-agent support
                std::cout << "MAPF: Using NATIVE multi-agent planning (all agents planned simultaneously)" << std::endl;
                paths = currentStrategy->planMultipleAgentPaths(environments);
            } else {
                // Plan paths with coordination using single-agent strategy
                // Simple sequential planning with obstacle avoidance
                std::cout << "MAPF: Using SEQUENTIAL planning (agents planned one after another)" << std::endl;
                for (size_t i = 0; i < environments.size(); ++i) {
                    std::cout << "MAPF: Planning for agent " << i << " (considering " << paths.size() << " previous agents as obstacles)" << std::endl;
                    Environment modifiedEnv = environments[i];
                    
                    // Add previously planned agent paths as obstacles
                    for (size_t j = 0; j < i; ++j) {
                        for (const auto& waypoint : paths[j]) {
                            // Add agent radius around each waypoint as obstacle
                            BoundingBox agentObstacle;
                            agentObstacle.min = waypoint - glm::vec3(environments[i].agentRadius);
                            agentObstacle.max = waypoint + glm::vec3(environments[i].agentRadius);
                            agentObstacle.center = waypoint;
                            agentObstacle.size = glm::vec3(environments[i].agentRadius * 2.0f);
                            modifiedEnv.obstacles.push_back(agentObstacle);
                        }
                    }
                    
                    paths.push_back(planPath(modifiedEnv));
                }
            }
        } else {
            // Independent planning using current strategy
            std::cout << "MAPF: MAPF disabled or single agent - using independent planning" << std::endl;
            for (const auto& env : environments) {
                paths.push_back(planPath(env));
            }
        }
        
        return paths;
    }
    
    std::vector<glm::vec3> updateDynamicPlanning(const Environment& environment,
                                                 const std::vector<glm::vec3>& currentPath,
                                                 float deltaTime) {
        
        // Check if replanning is needed
        if (!shouldReplan(environment, currentPath, deltaTime)) {
            return {}; // No replanning needed
        }
        
        std::cout << "Dynamic replanning triggered using " 
                  << (currentStrategy ? currentStrategy->getAlgorithmName() : "fallback") << std::endl;
        
        // Use current strategy for replanning
        if (currentStrategy) {
            if (currentStrategy->supportsDynamicReplanning()) {
                return currentStrategy->replanPath(environment, currentPath, deltaTime);
            } else {
                return currentStrategy->planPath(environment);
            }
        }
        
        return {};
    }
    
    bool isPathValid(const Environment& environment, const std::vector<glm::vec3>& path) {
        if (path.empty()) return false;
        
        // Check each waypoint for validity
        for (const auto& waypoint : path) {
            if (!environment.isPositionValid(waypoint)) {
                return false;
            }
        }
        
        return true;
    }
    
    void setConfig(const Config& config) {
        currentConfig = config;
        
        // Switch strategy if algorithm changed
        if (config.algorithm != Config::Algorithm::CUSTOM) {
            setStrategy(config.algorithm);
        }
        
        // Configure current strategy
        if (currentStrategy) {
            currentStrategy->configure(config);
        }
        
        std::cout << "Path Planning Module: Configuration updated for " 
                  << (currentStrategy ? currentStrategy->getAlgorithmName() : "no strategy") << std::endl;
    }
    
    const Config& getConfig() {
        return currentConfig;
    }

    // Helper function for replanning decisions
    bool shouldReplan(const Environment& environment, const std::vector<glm::vec3>& currentPath, float deltaTime) {
        // Check if path is still valid
        if (!isPathValid(environment, currentPath)) {
            return true;
        }
        
        // Check replanning interval
        static float timeSinceLastReplan = 0.0f;
        timeSinceLastReplan += deltaTime;
        
        if (currentConfig.enableDynamicReplanning && 
            timeSinceLastReplan >= currentConfig.replanningInterval) {
            timeSinceLastReplan = 0.0f;
            return true;
        }
        
        return false;
    }
}
