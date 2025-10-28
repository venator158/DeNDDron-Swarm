#include "apf_strategy.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace PathPlanning {
    
    void APFStrategy::initialize() {
        std::cout << "APF Strategy: Initialized" << std::endl;
    }
    
    void APFStrategy::shutdown() {
        std::cout << "APF Strategy: Shutdown" << std::endl;
    }
    
    void APFStrategy::configure(const Config& config) {
        // Basic parameters
        influenceRadius = config.influenceRadius;
        stepSize = config.stepSize > 0 ? config.stepSize : 0.1f;
        maxSteps = config.maxIterations;
        goalTolerance = config.goalTolerance;
        
        // APF force parameters
        attractiveForceGain = config.attractiveForceGain;
        repulsiveForceGain = config.repulsiveForceGain;
        
        // APF advanced parameters (if available)
        apfExponentialDecay = config.apfExponentialDecay;
        apfInverseSquareScale = config.apfInverseSquareScale;  
        apfStuckGrowthRate = config.apfStuckGrowthRate;
        
        std::cout << "APF Strategy: Configured with a=" << apfExponentialDecay 
                  << ", b=" << apfInverseSquareScale << ", k=" << attractiveForceGain << std::endl;
    }
    
    std::vector<glm::vec3> APFStrategy::planPath(const Environment& environment) {
        std::cout << "APF: Planning path using potential fields..." << std::endl;
        
        // APF Parameters (now configurable)
        const float a = apfExponentialDecay;      // Exponential decay rate
        const float b = apfInverseSquareScale;    // Inverse square scaling
        float k = attractiveForceGain;            // Attractive force gain (will grow if stuck)
        const float alpha = apfStuckGrowthRate;   // Stuck growth rate for k
        const float minMovement = 0.01f;          // Threshold for detecting stuck condition
        const int stuckThreshold = 5;            // Steps before considering stuck
        
        std::vector<glm::vec3> path;
        glm::vec3 currentPos = environment.agentStart;
        path.push_back(currentPos);
        
        int stuckCount = 0;
        glm::vec3 lastPos = currentPos;
        
        // Cost tracking variables
        float totalDistanceTraveled = 0.0f;
        glm::vec3 previousPos = currentPos;
        
        for (int step = 0; step < maxSteps; step++) {
            // Check if we've reached the goal
            float distanceToGoal = glm::length(environment.goalPosition - currentPos);
            if (distanceToGoal <= goalTolerance) {
                std::cout << "APF: Goal reached in " << step << " steps" << std::endl;
                std::cout << "APF: Total distance traveled: " << totalDistanceTraveled << " units" << std::endl;
                std::cout << "APF: Direct distance to goal: " << glm::length(environment.goalPosition - environment.agentStart) << " units" << std::endl;
                float efficiency = (totalDistanceTraveled > 0.0f) ? (glm::length(environment.goalPosition - environment.agentStart) / totalDistanceTraveled * 100.0f) : 0.0f;
                std::cout << "APF: Path efficiency: " << efficiency << "%" << std::endl;
                break;
            }
            
            // Calculate attractive force (constant magnitude towards goal)
            glm::vec3 toGoal = environment.goalPosition - currentPos;
            glm::vec3 attractiveForce = glm::vec3(0.0f);
            if (glm::length(toGoal) > 0.0f) {
                attractiveForce = k * glm::normalize(toGoal);
            }
            
            // Calculate repulsive forces from obstacles within influence radius
            glm::vec3 repulsiveForce = glm::vec3(0.0f);
            
            for (const auto& obstacle : environment.obstacles) {
                // Find closest point on obstacle surface to current position
                glm::vec3 closestPoint = getClosestPointOnBoundingBox(currentPos, obstacle);
                glm::vec3 toObstacle = closestPoint - currentPos;
                float distance = glm::length(toObstacle);
                
                // Only consider obstacles within influence radius
                if (distance <= influenceRadius && distance > 0.0f) {
                    // Apply repulsive force formula: z = (1/(b*d²)) * e^(-a*d)
                    float forceMagnitude = (1.0f / (b * distance * distance)) * std::exp(-a * distance);
                    
                    // Force direction is away from obstacle (opposite to toObstacle)
                    glm::vec3 forceDirection = -glm::normalize(toObstacle);
                    repulsiveForce += forceMagnitude * forceDirection;
                }
            }
            
            // Combine forces using simple vector addition
            glm::vec3 totalForce = attractiveForce + repulsiveForce;
            
            // Move in direction of total force with unit step size
            glm::vec3 nextPos = currentPos;
            if (glm::length(totalForce) > 0.0f) {
                glm::vec3 moveDirection = glm::normalize(totalForce);
                nextPos = currentPos + stepSize * moveDirection;
            }
            
            // Check for stuck condition (minimal movement)
            float movement = glm::length(nextPos - lastPos);
            if (movement < minMovement) {
                stuckCount++;
                if (stuckCount >= stuckThreshold) {
                    // Exponentially increase attractive force to escape local minima
                    k = std::exp(alpha * stuckCount);
                    std::cout << "APF: Stuck detected, increasing k to " << k << std::endl;
                }
            } else {
                stuckCount = 0; // Reset stuck counter if we're moving
                k = attractiveForceGain;       // Reset k to default
            }
            
            // Update position
            lastPos = currentPos;
            currentPos = nextPos;
            path.push_back(currentPos);
            
            // Calculate distance traveled for cost tracking
            float stepDistance = glm::length(currentPos - previousPos);
            totalDistanceTraveled += stepDistance;
            previousPos = currentPos;
            
            // Safety check for world bounds
            if (!environment.worldBounds.contains(currentPos)) {
                std::cout << "APF: Agent moved outside world bounds, stopping" << std::endl;
                std::cout << "APF: Total distance traveled: " << totalDistanceTraveled << " units" << std::endl;
                break;
            }
        }
        
        // Display final mission cost summary
        float directDistance = glm::length(environment.goalPosition - environment.agentStart);
        float pathEfficiency = (totalDistanceTraveled > 0.0f) ? (directDistance / totalDistanceTraveled * 100.0f) : 0.0f;
        
        std::cout << "APF: Generated path with " << path.size() << " waypoints" << std::endl;
        std::cout << "APF: Mission Cost Summary:" << std::endl;
        std::cout << "  - Total distance traveled: " << totalDistanceTraveled << " units" << std::endl;
        std::cout << "  - Direct distance to goal: " << directDistance << " units" << std::endl;
        std::cout << "  - Path efficiency: " << pathEfficiency << "%" << std::endl;
        std::cout << "  - Path overhead: " << (totalDistanceTraveled - directDistance) << " units" << std::endl;
        std::cout << "  - Waypoints generated: " << path.size() << std::endl;
        return path;
    }
    
    // Helper function to find closest point on bounding box surface
    glm::vec3 APFStrategy::getClosestPointOnBoundingBox(const glm::vec3& point, const BoundingBox& box) const {
        glm::vec3 closest;
        closest.x = std::max(box.min.x, std::min(point.x, box.max.x));
        closest.y = std::max(box.min.y, std::min(point.y, box.max.y));
        closest.z = std::max(box.min.z, std::min(point.z, box.max.z));
        return closest;
    }
    
    std::vector<glm::vec3> APFStrategy::replanPath(const Environment& environment,
                                                   const std::vector<glm::vec3>& currentPath,
                                                   float deltaTime) {
        // APF can replan very efficiently from current position
        Environment replanEnv = environment;
        replanEnv.agentStart = currentPath.empty() ? environment.agentStart : currentPath[0];
        
        return planPath(replanEnv);
    }
}
