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
     * This provides all the data your path planning algorithm will need
     */
    struct Environment {
        // === WORLD BOUNDARIES ===
        BoundingBox worldBounds;                   // Complete world bounding box
        
        // === OBSTACLES ===
        std::vector<BoundingBox> obstacles;        // All static obstacles as bounding boxes
        
        // === AGENT INFORMATION ===
        glm::vec3 agentStart;                      // Current agent position (start point)
        glm::vec3 goalPosition;                    // Target position (end point)
        float agentRadius;                         // Agent collision radius for safety margin
        
        // === ALGORITHM PARAMETERS ===
        float goalTolerance;                       // How close to goal counts as "reached"
        float stepSize;                           // Recommended step size for your algorithm
        
        Environment() : agentRadius(0.5f), goalTolerance(0.5f), stepSize(0.5f) {}
        
        // === UTILITY METHODS FOR YOUR ALGORITHM ===
        
        /**
         * Check if a position is valid (within bounds and not colliding)
         * Use this to validate waypoints and intermediate positions
         */
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
        
        /**
         * Get distance to nearest obstacle (useful for potential field methods)
         * Returns the minimum distance from position to any obstacle surface
         */
        float distanceToNearestObstacle(const glm::vec3& position) const {
            float minDistance = std::numeric_limits<float>::max();
            for (const auto& obstacle : obstacles) {
                float dist = obstacle.distanceToPoint(position);
                minDistance = std::min(minDistance, dist);
            }
            return minDistance;
        }
        
        /**
         * Get indices of obstacles within a certain radius (useful for local planning)
         * Use this to optimize by only considering nearby obstacles
         */
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

    // ===== MAIN ENTRY POINTS FOR YOUR PATH PLANNING IMPLEMENTATION =====
    
    /**
     * Initialize the path planning module
     * Called once at startup - set up your algorithms here
     */
    void initialize();
    
    /**
     * Shutdown the path planning module
     * Called once at cleanup - clean up any resources here
     */
    void shutdown();
    
    /**
     * MAIN PATH PLANNING FUNCTION - IMPLEMENT YOUR ALGORITHM HERE
     * 
     * This is the primary function you need to implement. It receives complete
     * environment data and should return a collision-free path from start to goal.
     * 
     * @param environment Complete environment data including:
     *                   - agentStart: 3D starting position
     *                   - goalPosition: 3D target position  
     *                   - worldBounds: World boundaries (don't go outside these)
     *                   - obstacles: All obstacles as 3D bounding boxes
     *                   - agentRadius: Collision radius for safety margin
     *                   - Utility methods: isPositionValid(), distanceToNearestObstacle(), etc.
     * 
     * @return Vector of 3D waypoints representing the path
     *         - Return empty vector if no path found
     *         - First waypoint can be start position or first step
     *         - Last waypoint should be goal position (or close to it)
     */
    std::vector<glm::vec3> planPath(const Environment& environment);
    
    /**
     * MULTI-AGENT PATH PLANNING (OPTIONAL - ADVANCED)
     * 
     * Plan paths for multiple agents simultaneously. Useful for:
     * - Swarm coordination
     * - Avoiding inter-agent collisions
     * - Optimizing overall system performance
     * 
     * @param environments Vector of environments, one for each agent
     * @return Vector of paths, one for each agent (index corresponds to agent index)
     */
    std::vector<std::vector<glm::vec3>> planMultiplePaths(const std::vector<Environment>& environments);
    
    /**
     * DYNAMIC REPLANNING (OPTIONAL - ADVANCED)
     * 
     * Update path planning for dynamic environments. Called every frame if enabled.
     * Use this for:
     * - Moving obstacles
     * - Real-time replanning
     * - Adaptive path optimization
     * 
     * @param environment Current environment state
     * @param currentPath The agent's current planned path
     * @param deltaTime Time since last update (seconds)
     * @return New path if replanning is needed, empty vector if current path is still valid
     */
    std::vector<glm::vec3> updateDynamicPlanning(const Environment& environment,
                                                 const std::vector<glm::vec3>& currentPath,
                                                 float deltaTime);
    
    /**
     * PATH VALIDATION (OPTIONAL - UTILITY)
     * 
     * Check if a previously computed path is still valid.
     * Useful for determining when replanning is necessary.
     * 
     * @param environment Current environment state
     * @param path The path to validate
     * @return true if path is still valid, false if replanning is needed
     */
    bool isPathValid(const Environment& environment, const std::vector<glm::vec3>& path);
    
    // ===== CONFIGURATION SYSTEM =====
    
    /**
     * Configuration structure for path planning parameters
     * Customize these values to tune your algorithm's behavior
     */
    struct Config {
        // === ALGORITHM PARAMETERS ===
        float stepSize = 0.5f;                 // Path planning step size (smaller = more precise)
        int maxIterations = 1000;              // Maximum planning iterations (prevent infinite loops)
        float goalTolerance = 0.5f;            // Distance to goal considered "reached"
        
        // === POTENTIAL FIELD PARAMETERS (if using APF) ===
        float attractiveForceGain = 1.0f;      // Strength of attraction to goal (k parameter)
        float repulsiveForceGain = 10.0f;      // Strength of repulsion from obstacles
        float influenceRadius = 1.0f;          // Distance at which obstacles affect agent
        
        // === APF ADVANCED PARAMETERS ===
        float apfExponentialDecay = 0.8f;      // APF 'a' parameter for exponential decay
        float apfInverseSquareScale = 1.0f;    // APF 'b' parameter for inverse square scaling
        float apfStuckGrowthRate = 0.1f;       // APF 'alpha' parameter for local minima escape
        
        // === DYNAMIC PLANNING PARAMETERS ===
        bool enableDynamicReplanning = false;  // Enable real-time replanning
        float replanningInterval = 1.0f;       // How often to replan (seconds)
        
        // === ALGORITHM SELECTION ===
        enum Algorithm {
            APF,        // Artificial Potential Fields
            APF_MAPF,   // APF Multi-Agent Path Finding
            ORCA,
            AStar,      // A* algorithm  
            RRT,        // Rapidly-exploring Random Tree
            DIJKSTRA,   // Dijkstra's algorithm
            RRT_STAR,   // RRT* (optimal RRT)
            PRM,        // Probabilistic Roadmap
            CUSTOM      // Your custom algorithm
        } algorithm = APF;
        
        // === MULTI-AGENT PARAMETERS ===
        bool enableMAPF = true;               // Enable Multi-Agent Path Finding coordination
        
        // === ALGORITHM-SPECIFIC PARAMETERS ===
        // Add your own parameters here as needed for your specific algorithm
        // Examples:
        // float heuristicWeight = 1.0f;      // For A* weighting
        // int samplingAttempts = 100;        // For RRT sampling
        // float connectionRadius = 2.0f;     // For PRM connections
    };
    
    /**
     * Set the path planning strategy/algorithm
     * Call this to switch between different algorithms at runtime
     */
    void setStrategy(Config::Algorithm algorithm);
    
    /**
     * Set configuration parameters for the path planning module
     * Call this to update algorithm parameters at runtime
     */
    void setConfig(const Config& config);
    
    /**
     * Get current configuration parameters
     * Use this to read current settings
     */
    const Config& getConfig();
}
