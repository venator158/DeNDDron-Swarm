#include "apf_mapf_strategy.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace PathPlanning {
    
    void APFMAPFStrategy::initialize() {
        std::cout << "APF MAPF Strategy: Initialized" << std::endl;
    }
    
    void APFMAPFStrategy::shutdown() {
        std::cout << "APF MAPF Strategy: Shutdown" << std::endl;
    }
    
    void APFMAPFStrategy::configure(const Config& config) {
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
        
        // MAPF-specific parameters
        agentInfluenceRadius = influenceRadius * 1.5f;  // Agents have slightly larger influence
        agentRepulsiveGain = repulsiveForceGain * 2.0f; // Stronger agent-agent repulsion
        
        std::cout << "APF MAPF Strategy: Configured with a=" << apfExponentialDecay 
                  << ", b=" << apfInverseSquareScale << ", k=" << attractiveForceGain 
                  << ", agent_influence=" << agentInfluenceRadius << std::endl;
    }
    
    std::vector<glm::vec3> APFMAPFStrategy::planPath(const Environment& environment) {
        std::cout << "APF MAPF: Single agent fallback - using standard APF..." << std::endl;
        
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
        bool usingHexagonMode = false;
        glm::vec3 hexagonWaypoint = glm::vec3(0.0f);
        
        // Cost tracking variables
        float totalDistanceTraveled = 0.0f;
        glm::vec3 previousPos = currentPos;
        
        for (int step = 0; step < maxSteps; step++) {
            // Check if we've reached the goal
            float distanceToGoal = glm::length(environment.goalPosition - currentPos);
            if (distanceToGoal <= goalTolerance) {
                std::cout << "APF MAPF: Goal reached in " << step << " steps" << std::endl;
                std::cout << "APF MAPF: Total distance traveled: " << totalDistanceTraveled << " units" << std::endl;
                std::cout << "APF MAPF: Direct distance to goal: " << glm::length(environment.goalPosition - environment.agentStart) << " units" << std::endl;
                float efficiency = (totalDistanceTraveled > 0.0f) ? (glm::length(environment.goalPosition - environment.agentStart) / totalDistanceTraveled * 100.0f) : 0.0f;
                std::cout << "APF MAPF: Path efficiency: " << efficiency << "%" << std::endl;
                break;
            }
            
            // Check if we've reached the hexagon waypoint
            if (usingHexagonMode) {
                float distanceToWaypoint = glm::length(hexagonWaypoint - currentPos);
                if (distanceToWaypoint <= goalTolerance) {
                    std::cout << "APF MAPF: Hexagon waypoint reached, switching back to normal APF" << std::endl;
                    usingHexagonMode = false;
                    stuckCount = 0; // Reset stuck counter
                    k = attractiveForceGain; // Reset force gain
                }
            }
            
            // Calculate attractive force (towards goal or hexagon waypoint)
            glm::vec3 targetPos = usingHexagonMode ? hexagonWaypoint : environment.goalPosition;
            glm::vec3 toTarget = targetPos - currentPos;
            glm::vec3 attractiveForce = glm::vec3(0.0f);
            if (glm::length(toTarget) > 0.0f) {
                attractiveForce = k * glm::normalize(toTarget);
            }
            
            // Calculate repulsive forces from obstacles
            glm::vec3 repulsiveForce = calculateObstacleRepulsiveForce(currentPos, environment.obstacles);
            
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
                    if (!usingHexagonMode) {
                        // Local minima detected - render virtual hexagon and select waypoint
                        std::cout << "APF MAPF: Local minima detected (stuck for " << stuckCount << " ticks)" << std::endl;
                        hexagonWaypoint = findHexagonWaypoint(currentPos, environment.goalPosition, environment.obstacles);
                        usingHexagonMode = true;
                        std::cout << "APF MAPF: Virtual hexagon rendered, waypoint selected at (" << hexagonWaypoint.x << ", " 
                                  << hexagonWaypoint.y << ", " << hexagonWaypoint.z << ")" << std::endl;
                    }
                    
                    // Also apply force ramping to overcome GNRON (Goal Non-Reachable Obstacle Nearby)
                    k = attractiveForceGain * std::exp(alpha * (stuckCount - stuckThreshold));
                    if (stuckCount % 10 == 0) { // Log every 10 ticks to avoid spam
                        std::cout << "APF MAPF: Force ramping active (k = " << k << ") to overcome GNRON" << std::endl;
                    }
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
                std::cout << "APF MAPF: Agent moved outside world bounds, stopping" << std::endl;
                std::cout << "APF MAPF: Total distance traveled: " << totalDistanceTraveled << " units" << std::endl;
                break;
            }
        }
        
        // Display final mission cost summary
        float directDistance = glm::length(environment.goalPosition - environment.agentStart);
        float pathEfficiency = (totalDistanceTraveled > 0.0f) ? (directDistance / totalDistanceTraveled * 100.0f) : 0.0f;
        
        std::cout << "APF MAPF: Generated path with " << path.size() << " waypoints" << std::endl;
        std::cout << "APF MAPF: Mission Cost Summary:" << std::endl;
        std::cout << "  - Total distance traveled: " << totalDistanceTraveled << " units" << std::endl;
        std::cout << "  - Direct distance to goal: " << directDistance << " units" << std::endl;
        std::cout << "  - Path efficiency: " << pathEfficiency << "%" << std::endl;
        std::cout << "  - Path overhead: " << (totalDistanceTraveled - directDistance) << " units" << std::endl;
        std::cout << "  - Waypoints generated: " << path.size() << std::endl;
        return path;
    }
    
    std::vector<std::vector<glm::vec3>> APFMAPFStrategy::planMultipleAgentPaths(
        const std::vector<Environment>& environments) {
        
        std::cout << "APF MAPF: Planning paths for " << environments.size() << " agents simultaneously..." << std::endl;
        
        int numAgents = environments.size();
        if (numAgents == 0) return {};
        
        // Extract agent data
        std::vector<glm::vec3> startPositions, goalPositions;
        std::vector<float> agentRadii;
        std::vector<BoundingBox> allObstacles;
        
        for (const auto& env : environments) {
            startPositions.push_back(env.agentStart);
            goalPositions.push_back(env.goalPosition);
            agentRadii.push_back(env.agentRadius);
            
            // Combine all obstacles (assuming same world for all agents)
            if (allObstacles.empty()) {
                allObstacles = env.obstacles;
            }
        }
        
        // Initialize paths and current positions
        std::vector<std::vector<glm::vec3>> paths(numAgents);
        std::vector<glm::vec3> currentPositions = startPositions;
        std::vector<float> totalDistances(numAgents, 0.0f);
        std::vector<bool> reached(numAgents, false);
        
        for (int i = 0; i < numAgents; i++) {
            paths[i].push_back(currentPositions[i]);
        }
        
        // MAPF main loop
        for (int step = 0; step < maxSteps; step++) {
            std::vector<glm::vec3> nextPositions(numAgents);
            bool allReached = true;
            
            // Calculate next position for each agent
            for (int i = 0; i < numAgents; i++) {
                if (reached[i]) {
                    nextPositions[i] = currentPositions[i];
                    continue;
                }
                
                // Check if agent reached goal
                float distanceToGoal = glm::length(goalPositions[i] - currentPositions[i]);
                if (distanceToGoal <= goalTolerance) {
                    reached[i] = true;
                    nextPositions[i] = currentPositions[i];
                    std::cout << "APF MAPF: Agent " << i << " reached goal in " << step << " steps" << std::endl;
                    continue;
                }
                
                allReached = false;
                
                // Calculate total force for this agent
                // Note: For multi-agent, we use force ramping for dynamic collisions (other agents)
                // but static obstacles could use hexagon method if stuck (not implemented in MAPF yet)
                glm::vec3 totalForce = calculateTotalForceForAgent(i, currentPositions, 
                                                                  goalPositions, allObstacles, agentRadii);
                
                // Move agent
                if (glm::length(totalForce) > 0.0f) {
                    glm::vec3 moveDirection = glm::normalize(totalForce);
                    nextPositions[i] = currentPositions[i] + stepSize * moveDirection;
                } else {
                    nextPositions[i] = currentPositions[i];
                }
                
                // Track distance
                float stepDistance = glm::length(nextPositions[i] - currentPositions[i]);
                totalDistances[i] += stepDistance;
            }
            
            // Update all positions simultaneously
            currentPositions = nextPositions;
            for (int i = 0; i < numAgents; i++) {
                paths[i].push_back(currentPositions[i]);
            }
            
            if (allReached) {
                std::cout << "APF MAPF: All agents reached their goals in " << step << " steps" << std::endl;
                break;
            }
        }
        
        // Display cost summary for all agents
        std::cout << "APF MAPF: Multi-Agent Mission Cost Summary:" << std::endl;
        float totalSystemDistance = 0.0f;
        for (int i = 0; i < numAgents; i++) {
            float directDistance = glm::length(goalPositions[i] - startPositions[i]);
            float efficiency = (totalDistances[i] > 0.0f) ? (directDistance / totalDistances[i] * 100.0f) : 0.0f;
            
            std::cout << "  Agent " << i << ":" << std::endl;
            std::cout << "    - Distance traveled: " << totalDistances[i] << " units" << std::endl;
            std::cout << "    - Direct distance: " << directDistance << " units" << std::endl;
            std::cout << "    - Efficiency: " << efficiency << "%" << std::endl;
            std::cout << "    - Waypoints: " << paths[i].size() << std::endl;
            
            totalSystemDistance += totalDistances[i];
        }
        std::cout << "  Total system distance: " << totalSystemDistance << " units" << std::endl;
        
        return paths;
    }
    
    // Helper function to find closest point on bounding box surface
    glm::vec3 APFMAPFStrategy::getClosestPointOnBoundingBox(const glm::vec3& point, const BoundingBox& box) const {
        glm::vec3 closest;
        closest.x = std::max(box.min.x, std::min(point.x, box.max.x));
        closest.y = std::max(box.min.y, std::min(point.y, box.max.y));
        closest.z = std::max(box.min.z, std::min(point.z, box.max.z));
        return closest;
    }
    
    glm::vec3 APFMAPFStrategy::calculateTotalForceForAgent(int agentIndex,
                                                          const std::vector<glm::vec3>& allAgentPositions,
                                                          const std::vector<glm::vec3>& goalPositions,
                                                          const std::vector<BoundingBox>& obstacles,
                                                          const std::vector<float>& agentRadii) const {
        
        glm::vec3 currentPos = allAgentPositions[agentIndex];
        glm::vec3 goalPos = goalPositions[agentIndex];
        
        // 1. Attractive force to goal
        glm::vec3 attractiveForce = glm::vec3(0.0f);
        glm::vec3 toGoal = goalPos - currentPos;
        if (glm::length(toGoal) > 0.0f) {
            attractiveForce = attractiveForceGain * glm::normalize(toGoal);
        }
        
        // 2. Repulsive force from obstacles
        glm::vec3 obstacleRepulsiveForce = calculateObstacleRepulsiveForce(currentPos, obstacles);
        
        // 3. Repulsive force from other agents
        glm::vec3 agentRepulsiveForce = calculateAgentRepulsiveForce(agentIndex, allAgentPositions, agentRadii);
        
        return attractiveForce + obstacleRepulsiveForce + agentRepulsiveForce;
    }
    
    glm::vec3 APFMAPFStrategy::calculateAgentRepulsiveForce(int agentIndex,
                                                           const std::vector<glm::vec3>& allAgentPositions,
                                                           const std::vector<float>& agentRadii) const {
        
        glm::vec3 currentPos = allAgentPositions[agentIndex];
        glm::vec3 totalAgentRepulsive = glm::vec3(0.0f);
        float currentAgentRadius = agentRadii[agentIndex];
        
        for (int j = 0; j < allAgentPositions.size(); j++) {
            if (j == agentIndex) continue; // Don't repel from self
            
            glm::vec3 otherPos = allAgentPositions[j];
            float otherAgentRadius = agentRadii[j];
            
            glm::vec3 toOtherAgent = otherPos - currentPos;
            float distance = glm::length(toOtherAgent);
            
            // Combined radius for collision avoidance
            float combinedRadius = currentAgentRadius + otherAgentRadius;
            float safetyMargin = 1.5f; // 50% safety margin
            float effectiveInfluenceRadius = std::max(agentInfluenceRadius, combinedRadius * safetyMargin);
            
            if (distance <= effectiveInfluenceRadius && distance > 0.0f) {
                // Use same exponential formula but with agent-specific parameters
                const float a = apfExponentialDecay;
                const float b = apfInverseSquareScale;
                float forceMagnitude = agentRepulsiveGain * 
                                     (1.0f / (b * distance * distance)) * 
                                     std::exp(-a * distance);
                
                // Force direction is away from other agent
                glm::vec3 forceDirection = -glm::normalize(toOtherAgent);
                totalAgentRepulsive += forceMagnitude * forceDirection;
            }
        }
        
        return totalAgentRepulsive;
    }
    
    glm::vec3 APFMAPFStrategy::calculateObstacleRepulsiveForce(const glm::vec3& currentPos,
                                                              const std::vector<BoundingBox>& obstacles) const {
        
        glm::vec3 repulsiveForce = glm::vec3(0.0f);
        const float a = apfExponentialDecay;
        const float b = apfInverseSquareScale;
        
        for (const auto& obstacle : obstacles) {
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
        
        return repulsiveForce;
    }
    
    // Virtual hexagon method for static obstacle avoidance
    glm::vec3 APFMAPFStrategy::findHexagonWaypoint(const glm::vec3& currentPos, const glm::vec3& goalPos,
                                                   const std::vector<BoundingBox>& obstacles) const {
        
        // Create a virtual regular hexagon around the current position
        // The hexagon vertices represent potential waypoints to escape local minima
        const int numVertices = 6;
        const float hexagonRadius = influenceRadius * 1.5f; // Radius slightly larger than obstacle influence
        
        std::vector<glm::vec3> hexagonVertices;
        hexagonVertices.reserve(numVertices);
        
        // Calculate hexagon vertices in the XY plane
        for (int i = 0; i < numVertices; i++) {
            float angle = (M_PI / 3.0f) * i; // 60-degree intervals
            glm::vec3 vertex = currentPos;
            vertex.x += hexagonRadius * std::cos(angle);
            vertex.y += hexagonRadius * std::sin(angle);
            // Keep Z coordinate same as current position
            hexagonVertices.push_back(vertex);
        }
        
        // Find the best vertex based on:
        // 1. Clear path from current position to vertex
        // 2. Clear path from vertex to goal
        // 3. Distance to goal (prefer vertices closer to goal)
        // 4. Distance from obstacles
        
        glm::vec3 bestWaypoint = currentPos;
        float bestScore = -1.0f;
        
        for (const auto& vertex : hexagonVertices) {
            // Check if paths are clear
            bool pathToClear = isPathClear(currentPos, vertex, obstacles);
            bool pathToGoalClear = isPathClear(vertex, goalPos, obstacles);
            
            if (!pathToClear) continue; // Must be able to reach the vertex
            
            // Calculate score
            float distanceToGoal = glm::length(goalPos - vertex);
            float distanceFromObstacles = getObstacleDistance(vertex, obstacles);
            
            // Score function: prefer points closer to goal and farther from obstacles
            // Normalize and weight the factors
            float goalScore = 1.0f / (1.0f + distanceToGoal); // Higher score for closer to goal
            float obstacleScore = distanceFromObstacles; // Higher score for farther from obstacles
            float clearPathBonus = pathToGoalClear ? 2.0f : 0.0f; // Bonus if direct path to goal is clear
            
            float totalScore = goalScore + obstacleScore * 0.5f + clearPathBonus;
            
            if (totalScore > bestScore) {
                bestScore = totalScore;
                bestWaypoint = vertex;
            }
        }
        
        // If no good waypoint found, try extending the hexagon radius
        if (bestScore < 0.0f) {
            float extendedRadius = hexagonRadius * 2.0f;
            for (int i = 0; i < numVertices; i++) {
                float angle = (M_PI / 3.0f) * i;
                glm::vec3 vertex = currentPos;
                vertex.x += extendedRadius * std::cos(angle);
                vertex.y += extendedRadius * std::sin(angle);
                
                if (isPathClear(currentPos, vertex, obstacles)) {
                    float distanceToGoal = glm::length(goalPos - vertex);
                    float score = 1.0f / (1.0f + distanceToGoal);
                    if (score > bestScore) {
                        bestScore = score;
                        bestWaypoint = vertex;
                    }
                }
            }
        }
        
        return bestWaypoint;
    }
    
    // Check if a straight-line path between two points is clear of obstacles
    bool APFMAPFStrategy::isPathClear(const glm::vec3& from, const glm::vec3& to,
                                     const std::vector<BoundingBox>& obstacles) const {
        const int numSamples = 10; // Sample points along the path
        glm::vec3 direction = to - from;
        float pathLength = glm::length(direction);
        
        if (pathLength < 0.001f) return true; // Same point
        
        glm::vec3 step = direction / static_cast<float>(numSamples);
        
        for (int i = 0; i <= numSamples; i++) {
            glm::vec3 samplePoint = from + step * static_cast<float>(i);
            
            // Check if sample point is inside any obstacle
            for (const auto& obstacle : obstacles) {
                if (obstacle.contains(samplePoint)) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    // Get minimum distance from a point to all obstacles
    float APFMAPFStrategy::getObstacleDistance(const glm::vec3& point,
                                               const std::vector<BoundingBox>& obstacles) const {
        float minDistance = std::numeric_limits<float>::max();
        
        for (const auto& obstacle : obstacles) {
            glm::vec3 closestPoint = getClosestPointOnBoundingBox(point, obstacle);
            float distance = glm::length(closestPoint - point);
            minDistance = std::min(minDistance, distance);
        }
        
        return minDistance;
    }
    
    std::vector<glm::vec3> APFMAPFStrategy::replanPath(const Environment& environment,
                                                       const std::vector<glm::vec3>& currentPath,
                                                       float deltaTime) {
        // APF MAPF can replan very efficiently from current position
        Environment replanEnv = environment;
        replanEnv.agentStart = currentPath.empty() ? environment.agentStart : currentPath[0];
        
        return planPath(replanEnv);
    }
}
