#include "config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdexcept>
#include <random>
#include <ctime>

namespace ConfigLoader {
    
    // Helper function to parse vec3 from YAML node
    glm::vec3 parseVec3(const YAML::Node& node) {
        if (!node.IsSequence() || node.size() != 3) {
            throw std::runtime_error("Invalid vec3 format - expected [x, y, z]");
        }
        return glm::vec3(
            node[0].as<float>(),
            node[1].as<float>(),
            node[2].as<float>()
        );
    }
    
    SimulationConfig loadConfig(const std::string& filepath) {
        SimulationConfig config;
        
        try {
            // Load YAML file
            YAML::Node root = YAML::LoadFile(filepath);
            
            // Parse world configuration
            if (!root["world"]) {
                throw std::runtime_error("Missing 'world' section in config");
            }
            
            const YAML::Node& world = root["world"];
            if (!world["bounds"]) {
                throw std::runtime_error("Missing 'world.bounds' in config");
            }
            
            config.world.boundsMin = parseVec3(world["bounds"]["min"]);
            config.world.boundsMax = parseVec3(world["bounds"]["max"]);
            
            std::cout << "Loaded world bounds: [" 
                      << config.world.boundsMin.x << ", " << config.world.boundsMin.y << ", " << config.world.boundsMin.z 
                      << "] to [" 
                      << config.world.boundsMax.x << ", " << config.world.boundsMax.y << ", " << config.world.boundsMax.z 
                      << "]" << std::endl;
            
            // Parse obstacles configuration
            if (!root["obstacles"]) {
                throw std::runtime_error("Missing 'obstacles' section in config");
            }
            
            const YAML::Node& obstacles = root["obstacles"];
            int obstacleCount = obstacles["count"].as<int>(0);
            glm::vec3 defaultSize = parseVec3(obstacles["default_size"]);
            
            std::cout << "Target obstacle count: " << obstacleCount << std::endl;
            
            // Load explicitly defined obstacles
            if (obstacles["positions"]) {
                for (const auto& obsNode : obstacles["positions"]) {
                    ObstacleConfig obs;
                    obs.center = parseVec3(obsNode["center"]);
                    
                    if (obsNode["size"]) {
                        obs.size = parseVec3(obsNode["size"]);
                    } else {
                        obs.size = defaultSize;
                    }
                    
                    config.obstacles.push_back(obs);
                }
            }
            
            std::cout << "Loaded " << config.obstacles.size() << " explicitly defined obstacles" << std::endl;
            
            // Generate random obstacles if needed
            if (static_cast<int>(config.obstacles.size()) < obstacleCount) {
                PathPlanning::BoundingBox worldBounds(
                    (config.world.boundsMin + config.world.boundsMax) * 0.5f,
                    config.world.boundsMax - config.world.boundsMin
                );
                config.obstacles = generateObstacles(config.obstacles, obstacleCount, defaultSize, worldBounds);
            }
            
            // Parse agents configuration
            if (!root["agents"]) {
                throw std::runtime_error("Missing 'agents' section in config");
            }
            
            const YAML::Node& agents = root["agents"];
            float defaultRadius = agents["default_radius"].as<float>(0.5f);
            float defaultSpeed = agents["default_speed"].as<float>(2.0f);
            
            if (!agents["agent_list"]) {
                throw std::runtime_error("Missing 'agents.agent_list' in config");
            }
            
            // Load each agent
            for (const auto& agentNode : agents["agent_list"]) {
                AgentConfig agent;
                
                agent.id = agentNode["id"].as<int>();
                agent.start = parseVec3(agentNode["start"]);
                agent.goal = parseVec3(agentNode["goal"]);
                
                // Optional fields with defaults
                if (agentNode["color"]) {
                    agent.color = parseVec3(agentNode["color"]);
                } else {
                    // Generate a default color based on ID
                    float hue = static_cast<float>(agent.id) * 0.618033988749895f; // Golden ratio
                    hue = hue - std::floor(hue); // Wrap to [0, 1]
                    agent.color = glm::vec3(
                        0.5f + 0.5f * std::cos(2.0f * 3.14159f * hue),
                        0.5f + 0.5f * std::cos(2.0f * 3.14159f * (hue + 0.333f)),
                        0.5f + 0.5f * std::cos(2.0f * 3.14159f * (hue + 0.666f))
                    );
                }
                
                agent.radius = agentNode["radius"].as<float>(defaultRadius);
                agent.speed = agentNode["speed"].as<float>(defaultSpeed);
                
                config.agents.push_back(agent);
                
                std::cout << "Loaded Agent " << agent.id 
                          << ": start=(" << agent.start.x << ", " << agent.start.y << ", " << agent.start.z << ") "
                          << "goal=(" << agent.goal.x << ", " << agent.goal.y << ", " << agent.goal.z << ")" 
                          << std::endl;
            }
            
            std::cout << "Successfully loaded " << config.agents.size() << " agents from config" << std::endl;
            
            if (config.agents.empty()) {
                throw std::runtime_error("No agents defined in configuration");
            }
            
        } catch (const YAML::Exception& e) {
            throw std::runtime_error(std::string("YAML parsing error: ") + e.what());
        }
        
        return config;
    }
    
    std::vector<ObstacleConfig> generateObstacles(
        const std::vector<ObstacleConfig>& existingObstacles,
        int totalCount,
        const glm::vec3& defaultSize,
        const PathPlanning::BoundingBox& worldBounds
    ) {
        std::vector<ObstacleConfig> allObstacles = existingObstacles;
        
        if (static_cast<int>(allObstacles.size()) >= totalCount) {
            return allObstacles;
        }
        
        int toGenerate = totalCount - static_cast<int>(allObstacles.size());
        std::cout << "Generating " << toGenerate << " random obstacles..." << std::endl;
        
        // Random number generator
        static std::mt19937 gen(static_cast<unsigned>(std::time(nullptr)));
        
        // Define safe generation bounds (slightly inside world bounds to avoid edge placement)
        glm::vec3 safeMin = worldBounds.min + defaultSize;
        glm::vec3 safeMax = worldBounds.max - defaultSize;
        
        std::uniform_real_distribution<float> distX(safeMin.x, safeMax.x);
        std::uniform_real_distribution<float> distY(safeMin.y, safeMax.y);
        std::uniform_real_distribution<float> distZ(safeMin.z, safeMax.z);
        
        // Size variation (80% to 120% of default size)
        std::uniform_real_distribution<float> sizeVariation(0.8f, 1.2f);
        
        for (int i = 0; i < toGenerate; ++i) {
            ObstacleConfig obs;
            
            // Generate random position
            obs.center = glm::vec3(
                distX(gen),
                distY(gen),
                distZ(gen)
            );
            
            // Vary size slightly
            float variation = sizeVariation(gen);
            obs.size = defaultSize * variation;
            
            // Check for overlap with existing obstacles (simple check)
            bool tooClose = false;
            float minSeparation = 3.0f; // Minimum distance between obstacle centers
            
            for (const auto& existing : allObstacles) {
                float dist = glm::length(obs.center - existing.center);
                if (dist < minSeparation) {
                    tooClose = true;
                    break;
                }
            }
            
            // If too close, try again (simple retry, max 3 attempts)
            int attempts = 0;
            while (tooClose && attempts < 3) {
                obs.center = glm::vec3(distX(gen), distY(gen), distZ(gen));
                tooClose = false;
                for (const auto& existing : allObstacles) {
                    float dist = glm::length(obs.center - existing.center);
                    if (dist < minSeparation) {
                        tooClose = true;
                        break;
                    }
                }
                attempts++;
            }
            
            allObstacles.push_back(obs);
            std::cout << "  Generated obstacle at (" << obs.center.x << ", " << obs.center.y << ", " << obs.center.z << ")" << std::endl;
        }
        
        return allObstacles;
    }
}
