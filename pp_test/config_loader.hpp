#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "path_planning_interface.hpp"

namespace ConfigLoader {
    
    /**
     * Agent configuration data loaded from YAML
     */
    struct AgentConfig {
        int id;
        glm::vec3 start;
        glm::vec3 goal;
        glm::vec3 color;
        float radius;
        float speed;
    };
    
    /**
     * Obstacle configuration data loaded from YAML
     */
    struct ObstacleConfig {
        glm::vec3 center;
        glm::vec3 size;
    };
    
    /**
     * World configuration data loaded from YAML
     */
    struct WorldConfig {
        glm::vec3 boundsMin;
        glm::vec3 boundsMax;
    };
    
    /**
     * Complete simulation configuration loaded from YAML
     */
    struct SimulationConfig {
        WorldConfig world;
        std::vector<ObstacleConfig> obstacles;
        std::vector<AgentConfig> agents;
    };
    
    /**
     * Load simulation configuration from YAML file
     * @param filepath Path to the YAML configuration file
     * @return SimulationConfig structure with all loaded data
     * @throws std::runtime_error if file cannot be loaded or parsed
     */
    SimulationConfig loadConfig(const std::string& filepath);
    
    /**
     * Generate random obstacles to fill the count requirement
     * @param existingObstacles Obstacles already defined in config
     * @param totalCount Total number of obstacles desired
     * @param defaultSize Default size for randomly generated obstacles
     * @param worldBounds World boundaries to generate obstacles within
     * @return Vector of all obstacles (existing + randomly generated)
     */
    std::vector<ObstacleConfig> generateObstacles(
        const std::vector<ObstacleConfig>& existingObstacles,
        int totalCount,
        const glm::vec3& defaultSize,
        const PathPlanning::BoundingBox& worldBounds
    );
}
