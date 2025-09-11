#pragma once

#include <vector>
#include <glm/glm.hpp>

// Forward declarations
class Agent;
class Obstacle;

/**
 * Interface for Path Planning Module
 * 
 * This header defines the interface that your path planning module should implement.
 * The module will be responsible for generating paths for agents in 3D space.
 */

namespace PathPlanning {
    
    /**
     * Initialize the path planning module
     * Called once at startup
     */
    void initialize();
    
    /**
     * Shutdown the path planning module
     * Called once at cleanup
     */
    void shutdown();
    
    /**
     * Plan a path for a single agent from start to goal, avoiding obstacles
     * 
     * @param start Starting position in 3D space
     * @param goal Target position in 3D space
     * @param obstacles List of obstacles to avoid
     * @return Vector of 3D waypoints representing the path (empty if no path found)
     */
    std::vector<glm::vec3> planPath(const glm::vec3& start, 
                                   const glm::vec3& goal, 
                                   const std::vector<Obstacle*>& obstacles);
    
    /**
     * Plan paths for multiple agents simultaneously (for swarm coordination)
     * 
     * @param agents List of agents that need paths
     * @param obstacles List of obstacles to avoid
     * @return Vector of paths, one for each agent (index corresponds to agent index)
     */
    std::vector<std::vector<glm::vec3>> planMultiplePaths(const std::vector<Agent*>& agents,
                                                          const std::vector<Obstacle*>& obstacles);
    
    /**
     * Update dynamic path planning (for moving obstacles or real-time replanning)
     * Called every frame if dynamic planning is enabled
     * 
     * @param agents List of active agents
     * @param obstacles List of current obstacles (may include moving ones)
     * @param deltaTime Time since last update
     */
    void updateDynamicPlanning(const std::vector<Agent*>& agents,
                              const std::vector<Obstacle*>& obstacles,
                              float deltaTime);
    
    /**
     * Check if a path is still valid (no new obstacles blocking it)
     * 
     * @param path The path to validate
     * @param obstacles Current list of obstacles
     * @return true if path is still valid, false if replanning is needed
     */
    bool isPathValid(const std::vector<glm::vec3>& path,
                     const std::vector<Obstacle*>& obstacles);
    
    /**
     * Configuration structure for path planning parameters
     */
    struct Config {
        float agentRadius = 0.5f;           // Agent collision radius
        float stepSize = 1.0f;              // Path planning step size
        int maxIterations = 1000;           // Maximum planning iterations
        float goalTolerance = 0.5f;         // Distance to goal considered "reached"
        bool enableDynamicReplanning = false; // Enable real-time replanning
        float replanningInterval = 1.0f;    // How often to replan (seconds)
    };
    
    /**
     * Set configuration parameters for the path planning module
     */
    void setConfig(const Config& config);
    
    /**
     * Get current configuration parameters
     */
    const Config& getConfig();
}
