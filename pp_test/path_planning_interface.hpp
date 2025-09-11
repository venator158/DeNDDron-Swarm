#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace PathPlanning {
    
    /**
     * Bounding box structure for obstacles and world bounds
     */
    struct BoundingBox {
        glm::vec3 min;  // Minimum corner (x, y, z)
        glm::vec3 max;  // Maximum corner (x, y, z)
        glm::vec3 center; // Center point (for convenience)
        glm::vec3 size;   // Size dimensions (for convenience)
        
        BoundingBox() : min(0.0f), max(0.0f), center(0.0f), size(0.0f) {}
        
        BoundingBox(const glm::vec3& centerPos, const glm::vec3& dimensions) {
            center = centerPos;
            size = dimensions;
            glm::vec3 halfSize = dimensions * 0.5f;
            min = centerPos - halfSize;
            max = centerPos + halfSize;
        }
        
        bool contains(const glm::vec3& point) const {
            return point.x >= min.x && point.x <= max.x &&
                   point.y >= min.y && point.y <= max.y &&
                   point.z >= min.z && point.z <= max.z;
        }
        
        bool intersects(const BoundingBox& other) const {
            return min.x <= other.max.x && max.x >= other.min.x &&
                   min.y <= other.max.y && max.y >= other.min.y &&
                   min.z <= other.max.z && max.z >= other.min.z;
        }
        
        // Distance from point to box (0 if inside)
        float distanceToPoint(const glm::vec3& point) const {
            glm::vec3 closest = glm::clamp(point, min, max);
            return glm::length(point - closest);
        }
        
        // Get volume of the bounding box
        float getVolume() const {
            return size.x * size.y * size.z;
        }
    };
    
    /**
     * Environment data structure containing all spatial information
     * Optimized for path planning algorithms like APF, A*, RRT, etc.
     */
    struct Environment {
        // World boundaries
        BoundingBox worldBounds;
        
        // All obstacles with their bounding boxes
        std::vector<BoundingBox> obstacles;
        
        // Agent information
        glm::vec3 agentStart;                      // Current agent position
        glm::vec3 goalPosition;                    // Target position
        float agentRadius;                         // Agent collision radius
        
        // Additional data for advanced algorithms
        float goalTolerance;                       // How close to goal counts as "reached"
        float stepSize;                           // Recommended step size for algorithms
        
        Environment() : agentRadius(0.5f), goalTolerance(0.5f), stepSize(0.5f) {}
        
        // Utility methods for path planning algorithms
        bool isPositionValid(const glm::vec3& position) const {
            // Check if position is within world bounds
            if (!worldBounds.contains(position)) {
                return false;
            }
            
            // Check collision with obstacles (considering agent radius)
            BoundingBox agentBox(position, glm::vec3(agentRadius * 2.0f));
            for (const auto& obstacle : obstacles) {
                if (agentBox.intersects(obstacle)) {
                    return false;
                }
            }
            return true;
        }
        
        // Get distance to nearest obstacle (useful for APF)
        float distanceToNearestObstacle(const glm::vec3& position) const {
            float minDistance = std::numeric_limits<float>::max();
            for (const auto& obstacle : obstacles) {
                float dist = obstacle.distanceToPoint(position);
                minDistance = std::min(minDistance, dist);
            }
            return minDistance;
        }
        
        // Get all obstacles within a certain radius (useful for local planning)
        std::vector<size_t> getObstaclesInRadius(const glm::vec3& position, float radius) const {
            std::vector<size_t> nearbyObstacles;
            for (size_t i = 0; i < obstacles.size(); ++i) {
                if (obstacles[i].distanceToPoint(position) <= radius) {
                    nearbyObstacles.push_back(i);
                }
            }
            return nearbyObstacles;
        }
    };

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
     * @param environment Complete environment data (bounds, obstacles, agent info)
     * @return Vector of 3D waypoints representing the path (empty if no path found)
     */
    std::vector<glm::vec3> planPath(const Environment& environment);
    
    /**
     * Plan paths for multiple agents simultaneously (for swarm coordination)
     * 
     * @param environments Vector of environments, one for each agent
     * @return Vector of paths, one for each agent (index corresponds to agent index)
     */
    std::vector<std::vector<glm::vec3>> planMultiplePaths(const std::vector<Environment>& environments);
    
    /**
     * Update dynamic path planning (for moving obstacles or real-time replanning)
     * Called every frame if dynamic planning is enabled
     * 
     * @param environment Current environment state
     * @param currentPath The agent's current path
     * @param deltaTime Time since last update
     * @return New path if replanning is needed, empty vector if current path is still valid
     */
    std::vector<glm::vec3> updateDynamicPlanning(const Environment& environment,
                                                 const std::vector<glm::vec3>& currentPath,
                                                 float deltaTime);
    
    /**
     * Check if a path is still valid (no new obstacles blocking it)
     * 
     * @param environment Current environment state
     * @param path The path to validate
     * @return true if path is still valid, false if replanning is needed
     */
    bool isPathValid(const Environment& environment, const std::vector<glm::vec3>& path);
    
    /**
     * Configuration structure for path planning parameters
     */
    struct Config {
        // APF (Artificial Potential Field) parameters
        float attractiveForceGain = 1.0f;      // Strength of attraction to goal
        float repulsiveForceGain = 10.0f;      // Strength of repulsion from obstacles
        float influenceRadius = 5.0f;          // Distance at which obstacles affect agent
        
        // General algorithm parameters
        float stepSize = 0.5f;                 // Path planning step size
        int maxIterations = 1000;              // Maximum planning iterations
        float goalTolerance = 0.5f;            // Distance to goal considered "reached"
        
        // Dynamic planning parameters
        bool enableDynamicReplanning = false;  // Enable real-time replanning
        float replanningInterval = 1.0f;       // How often to replan (seconds)
        
        // Algorithm selection
        enum Algorithm {
            APF,        // Artificial Potential Fields
            A_STAR,     // A* algorithm
            RRT,        // Rapidly-exploring Random Tree
            CUSTOM      // Your custom algorithm
        } selectedAlgorithm = APF;
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
