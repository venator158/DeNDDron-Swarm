#include "path_planning_interface.hpp"
#include <iostream>
#include <glm/glm.hpp>

int main() {
    std::cout << "=== Path Planning System Test ===" << std::endl;
    
    // Initialize the system
    PathPlanning::initialize();
    
    // Configure system
    PathPlanning::Config config;
    config.algorithm = PathPlanning::Config::Algorithm::APF;
    config.stepSize = 0.1f;
    config.maxIterations = 1000;
    config.enableMAPF = true;
    
    PathPlanning::setConfig(config);
    
    // Create test environment
    PathPlanning::Environment env;
    env.agentStart = glm::vec3(0.0f, 0.0f, 0.0f);
    env.goalPosition = glm::vec3(10.0f, 10.0f, 0.0f);
    env.agentRadius = 0.5f;
    env.goalTolerance = 0.5f;
    
    // Set world bounds
    env.worldBounds.min = glm::vec3(-20.0f, -20.0f, -5.0f);
    env.worldBounds.max = glm::vec3(20.0f, 20.0f, 5.0f);
    
    // Add some obstacles
    PathPlanning::BoundingBox obstacle1;
    obstacle1.min = glm::vec3(4.0f, 4.0f, -1.0f);
    obstacle1.max = glm::vec3(6.0f, 6.0f, 1.0f);
    obstacle1.center = glm::vec3(5.0f, 5.0f, 0.0f);
    obstacle1.size = glm::vec3(2.0f, 2.0f, 2.0f);
    env.obstacles.push_back(obstacle1);
    
    PathPlanning::BoundingBox obstacle2;
    obstacle2.min = glm::vec3(7.0f, 2.0f, -1.0f);
    obstacle2.max = glm::vec3(9.0f, 4.0f, 1.0f);
    obstacle2.center = glm::vec3(8.0f, 3.0f, 0.0f);
    obstacle2.size = glm::vec3(2.0f, 2.0f, 2.0f);
    env.obstacles.push_back(obstacle2);
    
    std::cout << "\n=== Testing Single Agent Path Planning ===" << std::endl;
    
    // Test APF
    std::cout << "\n--- Testing APF Algorithm ---" << std::endl;
    PathPlanning::setStrategy(PathPlanning::Config::Algorithm::APF);
    auto apfPath = PathPlanning::planPath(env);
    std::cout << "APF Path length: " << apfPath.size() << " waypoints" << std::endl;
    
    // Test A*
    std::cout << "\n--- Testing A* Algorithm ---" << std::endl;
    PathPlanning::setStrategy(PathPlanning::Config::Algorithm::AStar);
    auto astarPath = PathPlanning::planPath(env);
    std::cout << "A* Path length: " << astarPath.size() << " waypoints" << std::endl;
    
    // Test RRT
    std::cout << "\n--- Testing RRT Algorithm ---" << std::endl;
    PathPlanning::setStrategy(PathPlanning::Config::Algorithm::RRT);
    auto rrtPath = PathPlanning::planPath(env);
    std::cout << "RRT Path length: " << rrtPath.size() << " waypoints" << std::endl;
    
    std::cout << "\n=== Testing Multi-Agent Path Planning (MAPF) ===" << std::endl;
    
    // Create multiple agents
    std::vector<PathPlanning::Environment> environments;
    
    // Agent 1
    PathPlanning::Environment env1 = env;
    env1.agentStart = glm::vec3(0.0f, 0.0f, 0.0f);
    env1.goalPosition = glm::vec3(10.0f, 10.0f, 0.0f);
    environments.push_back(env1);
    
    // Agent 2
    PathPlanning::Environment env2 = env;
    env2.agentStart = glm::vec3(10.0f, 0.0f, 0.0f);
    env2.goalPosition = glm::vec3(0.0f, 10.0f, 0.0f);
    environments.push_back(env2);
    
    // Agent 3
    PathPlanning::Environment env3 = env;
    env3.agentStart = glm::vec3(5.0f, 0.0f, 0.0f);
    env3.goalPosition = glm::vec3(5.0f, 10.0f, 0.0f);
    environments.push_back(env3);
    
    // Test with APF MAPF strategy for true multi-agent coordination
    std::cout << "\n--- Testing APF MAPF (True Multi-Agent Coordination) ---" << std::endl;
    std::cout << "This strategy considers ALL agents simultaneously during planning!" << std::endl;
    PathPlanning::setStrategy(PathPlanning::Config::Algorithm::APF_MAPF);
    auto mapfPaths = PathPlanning::planMultiplePaths(environments);
    
    std::cout << "APF MAPF planned paths for " << mapfPaths.size() << " agents:" << std::endl;
    for (size_t i = 0; i < mapfPaths.size(); ++i) {
        std::cout << "  Agent " << i << ": " << mapfPaths[i].size() << " waypoints" << std::endl;
        if (mapfPaths[i].size() > 1) {
            std::cout << "    Start: (" << mapfPaths[i][0].x << ", " << mapfPaths[i][0].y << ")" << std::endl;
            std::cout << "    End: (" << mapfPaths[i].back().x << ", " << mapfPaths[i].back().y << ")" << std::endl;
        }
    }
    
    // Compare with sequential APF planning
    std::cout << "\n--- Testing Regular APF (Sequential Planning) ---" << std::endl;
    std::cout << "This strategy plans each agent independently, one after another." << std::endl;
    PathPlanning::setStrategy(PathPlanning::Config::Algorithm::APF);
    auto multiPaths = PathPlanning::planMultiplePaths(environments);
    
    std::cout << "Regular APF planned paths for " << multiPaths.size() << " agents:" << std::endl;
    for (size_t i = 0; i < multiPaths.size(); ++i) {
        std::cout << "  Agent " << i << ": " << multiPaths[i].size() << " waypoints" << std::endl;
        if (multiPaths[i].size() > 1) {
            std::cout << "    Start: (" << multiPaths[i][0].x << ", " << multiPaths[i][0].y << ")" << std::endl;
            std::cout << "    End: (" << multiPaths[i].back().x << ", " << multiPaths[i].back().y << ")" << std::endl;
        }
    }
    
    std::cout << "\n=== Testing Path Validation ===" << std::endl;
    
    bool isValid = PathPlanning::isPathValid(env, apfPath);
    std::cout << "Path validation result: " << (isValid ? "VALID" : "INVALID") << std::endl;
    
    // Shutdown
    PathPlanning::shutdown();
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
