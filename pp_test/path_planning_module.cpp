#include "path_planning_interface.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace PathPlanning {
    
    static Config currentConfig;
    
    // Forward declaration of APF implementation
    std::vector<glm::vec3> planPathAPF(const Environment& environment);
    
    void initialize() {
        std::cout << "Path Planning Module: Initialized" << std::endl;
        std::cout << "Available algorithms: APF, A*, RRT, Custom" << std::endl;
        std::cout << "Current algorithm: ";
        switch(currentConfig.selectedAlgorithm) {
            case Config::APF: std::cout << "Artificial Potential Fields (APF)"; break;
            case Config::A_STAR: std::cout << "A* Algorithm"; break;
            case Config::RRT: std::cout << "Rapidly-exploring Random Tree"; break;
            case Config::CUSTOM: std::cout << "Custom Algorithm"; break;
        }
        std::cout << std::endl;
    }
    
    void shutdown() {
        std::cout << "Path Planning Module: Shutdown" << std::endl;
    }
    
    std::vector<glm::vec3> planPath(const Environment& environment) {
        std::cout << "=== PATH PLANNING WITH COMPLETE ENVIRONMENT DATA ===" << std::endl;
        std::cout << "Agent Start: (" << environment.agentStart.x << ", " 
                  << environment.agentStart.y << ", " << environment.agentStart.z << ")" << std::endl;
        std::cout << "Goal Position: (" << environment.goalPosition.x << ", " 
                  << environment.goalPosition.y << ", " << environment.goalPosition.z << ")" << std::endl;
        std::cout << "Agent Radius: " << environment.agentRadius << std::endl;
        
        std::cout << "World Bounds: Min(" << environment.worldBounds.min.x << ", " 
                  << environment.worldBounds.min.y << ", " << environment.worldBounds.min.z << ") "
                  << "Max(" << environment.worldBounds.max.x << ", " 
                  << environment.worldBounds.max.y << ", " << environment.worldBounds.max.z << ")" << std::endl;
        
        std::cout << "Obstacles: " << environment.obstacles.size() << std::endl;
        for (size_t i = 0; i < environment.obstacles.size(); ++i) {
            const auto& box = environment.obstacles[i];
            std::cout << "  Obstacle " << i << ": Center(" << box.center.x << ", " << box.center.y << ", " << box.center.z 
                      << ") Size(" << box.size.x << ", " << box.size.y << ", " << box.size.z 
                      << ") Volume(" << box.getVolume() << ")" << std::endl;
        }
        
        // Implement path planning based on selected algorithm
        std::vector<glm::vec3> path;
        
        switch(currentConfig.selectedAlgorithm) {
            case Config::APF:
                path = planPathAPF(environment);
                break;
            case Config::A_STAR:
                std::cout << "A* algorithm not yet implemented, using APF" << std::endl;
                path = planPathAPF(environment);
                break;
            case Config::RRT:
                std::cout << "RRT algorithm not yet implemented, using APF" << std::endl;
                path = planPathAPF(environment);
                break;
            case Config::CUSTOM:
                std::cout << "Custom algorithm not yet implemented, using APF" << std::endl;
                path = planPathAPF(environment);
                break;
        }
        
        std::cout << "Generated path with " << path.size() << " waypoints" << std::endl;
        return path;
    }
    
    // Simple APF (Artificial Potential Fields) implementation
    std::vector<glm::vec3> planPathAPF(const Environment& environment) {
        std::vector<glm::vec3> path;
        glm::vec3 current = environment.agentStart;
        
        for (int iteration = 0; iteration < currentConfig.maxIterations; ++iteration) {
            // Check if we've reached the goal
            if (glm::length(current - environment.goalPosition) < currentConfig.goalTolerance) {
                path.push_back(environment.goalPosition);
                break;
            }
            
            // Calculate attractive force towards goal
            glm::vec3 attractiveForce = glm::normalize(environment.goalPosition - current) * currentConfig.attractiveForceGain;
            
            // Calculate repulsive forces from obstacles
            glm::vec3 repulsiveForce(0.0f);
            for (const auto& obstacle : environment.obstacles) {
                float distance = obstacle.distanceToPoint(current);
                if (distance < currentConfig.influenceRadius && distance > 0.01f) {
                    glm::vec3 direction = glm::normalize(current - obstacle.center);
                    float forceMagnitude = currentConfig.repulsiveForceGain / (distance * distance);
                    repulsiveForce += direction * forceMagnitude;
                }
            }
            
            // Calculate repulsive forces from world boundaries
            glm::vec3 boundaryRepulsiveForce(0.0f);
            const auto& bounds = environment.worldBounds;
            
            // Distance to each boundary wall
            float distToMinX = current.x - bounds.min.x;
            float distToMaxX = bounds.max.x - current.x;
            float distToMinY = current.y - bounds.min.y;
            float distToMaxY = bounds.max.y - current.y;
            float distToMinZ = current.z - bounds.min.z;
            float distToMaxZ = bounds.max.z - current.z;
            
            float boundaryInfluence = currentConfig.influenceRadius * 0.5f; // Boundary influence range
            
            // Repulsion from X boundaries
            if (distToMinX < boundaryInfluence && distToMinX > 0.01f) {
                float forceMagnitude = currentConfig.repulsiveForceGain * 0.5f / (distToMinX * distToMinX);
                boundaryRepulsiveForce += glm::vec3(forceMagnitude, 0.0f, 0.0f);
            }
            if (distToMaxX < boundaryInfluence && distToMaxX > 0.01f) {
                float forceMagnitude = currentConfig.repulsiveForceGain * 0.5f / (distToMaxX * distToMaxX);
                boundaryRepulsiveForce += glm::vec3(-forceMagnitude, 0.0f, 0.0f);
            }
            
            // Repulsion from Y boundaries (floor and ceiling)
            if (distToMinY < boundaryInfluence && distToMinY > 0.01f) {
                float forceMagnitude = currentConfig.repulsiveForceGain * 0.5f / (distToMinY * distToMinY);
                boundaryRepulsiveForce += glm::vec3(0.0f, forceMagnitude, 0.0f);
            }
            if (distToMaxY < boundaryInfluence && distToMaxY > 0.01f) {
                float forceMagnitude = currentConfig.repulsiveForceGain * 0.5f / (distToMaxY * distToMaxY);
                boundaryRepulsiveForce += glm::vec3(0.0f, -forceMagnitude, 0.0f);
            }
            
            // Repulsion from Z boundaries
            if (distToMinZ < boundaryInfluence && distToMinZ > 0.01f) {
                float forceMagnitude = currentConfig.repulsiveForceGain * 0.5f / (distToMinZ * distToMinZ);
                boundaryRepulsiveForce += glm::vec3(0.0f, 0.0f, forceMagnitude);
            }
            if (distToMaxZ < boundaryInfluence && distToMaxZ > 0.01f) {
                float forceMagnitude = currentConfig.repulsiveForceGain * 0.5f / (distToMaxZ * distToMaxZ);
                boundaryRepulsiveForce += glm::vec3(0.0f, 0.0f, -forceMagnitude);
            }
            
            // Combine all forces
            glm::vec3 totalForce = attractiveForce + repulsiveForce + boundaryRepulsiveForce;
            
            // Normalize and apply step size
            if (glm::length(totalForce) > 0.01f) {
                totalForce = glm::normalize(totalForce) * currentConfig.stepSize;
            }
            
            // Calculate next position
            glm::vec3 nextPosition = current + totalForce;
            
            // Check if next position is valid
            if (environment.isPositionValid(nextPosition)) {
                current = nextPosition;
                path.push_back(current);
            } else {
                // If invalid, try to move around obstacle
                std::cout << "Position invalid, trying alternative direction" << std::endl;
                // Simple obstacle avoidance: try perpendicular directions
                glm::vec3 perpendicular1 = glm::vec3(-totalForce.z, totalForce.y, totalForce.x) * 0.5f;
                glm::vec3 perpendicular2 = glm::vec3(totalForce.z, totalForce.y, -totalForce.x) * 0.5f;
                
                if (environment.isPositionValid(current + perpendicular1)) {
                    current = current + perpendicular1;
                    path.push_back(current);
                } else if (environment.isPositionValid(current + perpendicular2)) {
                    current = current + perpendicular2;
                    path.push_back(current);
                } else {
                    std::cout << "Stuck! No valid moves available." << std::endl;
                    break;
                }
            }
        }
        
        return path;
    }
    
    std::vector<std::vector<glm::vec3>> planMultiplePaths(const std::vector<Environment>& environments) {
        std::cout << "Path Planning Module: Planning paths for " << environments.size() << " agents" << std::endl;
        
        std::vector<std::vector<glm::vec3>> paths;
        paths.reserve(environments.size());
        
        for (const auto& env : environments) {
            paths.push_back(planPath(env));
        }
        
        return paths;
    }
    
    std::vector<glm::vec3> updateDynamicPlanning(const Environment& environment,
                                                 const std::vector<glm::vec3>& currentPath,
                                                 float deltaTime) {
        
        static float timeSinceLastReplan = 0.0f;
        timeSinceLastReplan += deltaTime;
        
        if (timeSinceLastReplan >= currentConfig.replanningInterval) {
            // Check if current path is still valid
            if (!isPathValid(environment, currentPath)) {
                std::cout << "Path invalidated, replanning..." << std::endl;
                timeSinceLastReplan = 0.0f;
                return planPath(environment);  // Return new path
            }
            timeSinceLastReplan = 0.0f;
        }
        
        return {};  // Return empty vector if no replanning needed
    }
    
    bool isPathValid(const Environment& environment, const std::vector<glm::vec3>& path) {
        if (path.empty()) return false;
        
        // Check each waypoint in the path
        for (const auto& waypoint : path) {
            if (!environment.isPositionValid(waypoint)) {
                return false;
            }
        }
        
        // Check line segments between waypoints for collision
        for (size_t i = 1; i < path.size(); ++i) {
            glm::vec3 start = path[i-1];
            glm::vec3 end = path[i];
            
            // Sample points along the line segment
            int samples = static_cast<int>(glm::length(end - start) / currentConfig.stepSize) + 1;
            for (int j = 0; j <= samples; ++j) {
                float t = static_cast<float>(j) / static_cast<float>(samples);
                glm::vec3 point = start + t * (end - start);
                
                if (!environment.isPositionValid(point)) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    void setConfig(const Config& config) {
        currentConfig = config;
        std::cout << "Path Planning Module: Configuration updated" << std::endl;
    }
    
    const Config& getConfig() {
        return currentConfig;
    }
}
