#include "orca_mapf_strategy.hpp"
#include <iostream>
#include <algorithm>
#include <limits>

namespace PathPlanning {
    
    void ORCAMAPFStrategy::initialize() {
        std::cout << "ORCA MAPF Strategy: Initialized" << std::endl;
        std::cout << "ORCA: Real-time velocity-based collision avoidance" << std::endl;
    }
    
    void ORCAMAPFStrategy::shutdown() {
        std::cout << "ORCA MAPF Strategy: Shutdown" << std::endl;
    }
    
    void ORCAMAPFStrategy::configure(const Config& config) {
        stepSize = config.stepSize > 0 ? config.stepSize : 0.1f;
        maxSteps = config.maxIterations;
        goalTolerance = config.goalTolerance;
        
        // ORCA-specific parameters (can be added to Config if needed)
        timeHorizon = 2.0f;
        timeHorizonObst = 1.0f;
        maxSpeed = 3.0f;
        neighborDist = 5.0f;
        
        std::cout << "ORCA Strategy: Configured with timeHorizon=" << timeHorizon 
                  << ", maxSpeed=" << maxSpeed << std::endl;
    }
    
    // ===== HELPER FUNCTIONS =====
    
    glm::vec3 ORCAMAPFStrategy::computePreferredVelocity(const glm::vec3& position,
                                                         const glm::vec3& goal,
                                                         float maxSpeed) const {
        glm::vec3 toGoal = goal - position;
        float distance = glm::length(toGoal);
        
        if (distance < 0.001f) {
            return glm::vec3(0.0f);  // Already at goal
        }
        
        // Move toward goal at max speed, or slow down if close
        float speed = std::min(maxSpeed, distance / stepSize);
        return glm::normalize(toGoal) * speed;
    }
    
    glm::vec3 ORCAMAPFStrategy::closestPointOnSegment(const glm::vec3& point,
                                                       const glm::vec3& segStart,
                                                       const glm::vec3& segEnd) const {
        glm::vec3 segment = segEnd - segStart;
        float segmentLengthSq = glm::dot(segment, segment);
        
        if (segmentLengthSq < 0.0001f) {
            return segStart;  // Degenerate segment
        }
        
        float t = glm::dot(point - segStart, segment) / segmentLengthSq;
        t = glm::clamp(t, 0.0f, 1.0f);
        
        return segStart + t * segment;
    }
    
    float ORCAMAPFStrategy::det(const glm::vec3& v1, const glm::vec3& v2) const {
        // 2D determinant in XZ plane (ignoring Y)
        return v1.x * v2.z - v1.z * v2.x;
    }
    
    void ORCAMAPFStrategy::computeAgentORCALines(const AgentState& agent,
                                                 const std::vector<AgentState>& allAgents,
                                                 std::vector<Line>& orcaLines) const {
        for (const auto& other : allAgents) {
            if (other.id == agent.id) continue;  // Skip self
            
            glm::vec3 relativePosition = other.position - agent.position;
            glm::vec3 relativeVelocity = agent.velocity - other.velocity;
            float distSq = glm::dot(relativePosition, relativePosition);
            float combinedRadius = agent.radius + other.radius;
            float combinedRadiusSq = combinedRadius * combinedRadius;
            
            Line line;
            
            if (distSq > combinedRadiusSq) {
                // No collision - compute ORCA line
                glm::vec3 w = relativeVelocity - (relativePosition / timeHorizon);
                float wLengthSq = glm::dot(w, w);
                float dotProduct = glm::dot(w, relativePosition);
                
                if (dotProduct < 0.0f && dotProduct * dotProduct > combinedRadiusSq * wLengthSq) {
                    // Project on cut-off circle
                    float wLength = std::sqrt(wLengthSq);
                    glm::vec3 unitW = w / wLength;
                    
                    line.direction = glm::vec3(unitW.z, 0.0f, -unitW.x);  // Perpendicular in XZ
                    float u = (combinedRadius / timeHorizon - wLength);
                    line.point = agent.velocity + unitW * u * 0.5f;
                } else {
                    // Project on legs
                    float leg = std::sqrt(distSq - combinedRadiusSq);
                    
                    if (det(relativePosition, w) > 0.0f) {
                        // Project on left leg
                        line.direction = glm::normalize(glm::vec3(
                            relativePosition.x * leg - relativePosition.z * combinedRadius,
                            0.0f,
                            relativePosition.x * combinedRadius + relativePosition.z * leg
                        ));
                    } else {
                        // Project on right leg
                        line.direction = -glm::normalize(glm::vec3(
                            relativePosition.x * leg + relativePosition.z * combinedRadius,
                            0.0f,
                            -relativePosition.x * combinedRadius + relativePosition.z * leg
                        ));
                    }
                    
                    float dotProduct2 = glm::dot(relativeVelocity, line.direction);
                    line.point = agent.velocity + line.direction * dotProduct2 * 0.5f;
                }
            } else {
                // Collision imminent - more aggressive avoidance
                float invTimeStep = 1.0f / stepSize;
                glm::vec3 w = relativeVelocity - invTimeStep * relativePosition;
                float wLength = glm::length(w);
                glm::vec3 unitW = (wLength > 0.0f) ? w / wLength : glm::vec3(0.0f, 0.0f, 1.0f);
                
                line.direction = glm::vec3(unitW.z, 0.0f, -unitW.x);
                line.point = agent.velocity + unitW * (combinedRadius * invTimeStep - wLength) * 0.5f;
            }
            
            orcaLines.push_back(line);
        }
    }
    
    void ORCAMAPFStrategy::computeObstacleORCALines(const AgentState& agent,
                                                    const std::vector<BoundingBox>& obstacles,
                                                    std::vector<Line>& orcaLines) const {
        for (const auto& obstacle : obstacles) {
            // Find closest point on obstacle to agent
            glm::vec3 closestPoint;
            closestPoint.x = glm::clamp(agent.position.x, obstacle.min.x, obstacle.max.x);
            closestPoint.y = glm::clamp(agent.position.y, obstacle.min.y, obstacle.max.y);
            closestPoint.z = glm::clamp(agent.position.z, obstacle.min.z, obstacle.max.z);
            
            glm::vec3 relativePosition = closestPoint - agent.position;
            float distSq = glm::dot(relativePosition, relativePosition);
            
            if (distSq < agent.radius * agent.radius * 4.0f) {  // Within influence range
                Line line;
                float dist = std::sqrt(distSq);
                
                if (dist > 0.001f) {
                    glm::vec3 direction = relativePosition / dist;
                    
                    // Create ORCA line perpendicular to obstacle direction
                    line.direction = glm::vec3(-direction.z, 0.0f, direction.x);
                    line.point = agent.velocity - direction * ((agent.radius - dist) / timeHorizonObst);
                    
                    orcaLines.push_back(line);
                }
            }
        }
    }
    
    bool ORCAMAPFStrategy::isValidVelocity(const glm::vec3& velocity, const Line& line) const {
        // Check if velocity is on the allowed side of the line
        glm::vec3 diff = velocity - line.point;
        return det(line.direction, diff) >= 0.0f;
    }
    
    glm::vec3 ORCAMAPFStrategy::projectOnLine(const glm::vec3& velocity, const Line& line) const {
        // Project velocity onto the line
        glm::vec3 diff = velocity - line.point;
        float dotProduct = glm::dot(diff, line.direction);
        return line.point + line.direction * dotProduct;
    }
    
    glm::vec3 ORCAMAPFStrategy::linearProgram(const std::vector<Line>& lines,
                                              const glm::vec3& prefVelocity,
                                              float maxSpeed) const {
        // Start with preferred velocity
        glm::vec3 result = prefVelocity;
        
        // Clip to max speed
        float speedSq = glm::dot(result, result);
        if (speedSq > maxSpeed * maxSpeed) {
            result = glm::normalize(result) * maxSpeed;
        }
        
        // Iteratively project onto each ORCA line if violated
        for (size_t i = 0; i < lines.size(); ++i) {
            if (!isValidVelocity(result, lines[i])) {
                // Velocity violates this line - project it
                glm::vec3 projected = projectOnLine(result, lines[i]);
                
                // Clip to max speed again
                speedSq = glm::dot(projected, projected);
                if (speedSq > maxSpeed * maxSpeed) {
                    float t = maxSpeed / std::sqrt(speedSq);
                    projected = projected * t;
                }
                
                // Check if projection satisfies previous lines
                bool satisfiesAll = true;
                for (size_t j = 0; j < i; ++j) {
                    if (!isValidVelocity(projected, lines[j])) {
                        satisfiesAll = false;
                        break;
                    }
                }
                
                if (satisfiesAll) {
                    result = projected;
                } else {
                    // Linear programming failed - use fallback (stop or minimize violation)
                    result = glm::vec3(0.0f);
                    break;
                }
            }
        }
        
        return result;
    }
    
    // ===== MAIN ORCA COMPUTATION =====
    
    glm::vec3 ORCAMAPFStrategy::computeNewVelocity(int agentIndex,
                                                   const std::vector<glm::vec3>& positions,
                                                   const std::vector<glm::vec3>& velocities,
                                                   const std::vector<glm::vec3>& goals,
                                                   const std::vector<float>& radii,
                                                   const std::vector<BoundingBox>& obstacles) const {
        // Create agent states
        std::vector<AgentState> agents;
        for (size_t i = 0; i < positions.size(); ++i) {
            AgentState state;
            state.position = positions[i];
            state.velocity = velocities[i];
            state.prefVelocity = computePreferredVelocity(positions[i], goals[i], maxSpeed);
            state.radius = radii[i];
            state.id = i;
            agents.push_back(state);
        }
        
        // Compute preferred velocity for this agent
        AgentState& currentAgent = agents[agentIndex];
        
        // Compute ORCA lines
        std::vector<Line> orcaLines;
        computeAgentORCALines(currentAgent, agents, orcaLines);
        computeObstacleORCALines(currentAgent, obstacles, orcaLines);
        
        // Solve linear program to find optimal velocity
        return linearProgram(orcaLines, currentAgent.prefVelocity, maxSpeed);
    }
    
    // ===== PATH PLANNING INTERFACE =====
    
    std::vector<glm::vec3> ORCAMAPFStrategy::planPath(const Environment& environment) {
        std::cout << "ORCA: Single agent fallback (using simplified velocity-based planning)" << std::endl;
        
        std::vector<glm::vec3> path;
        glm::vec3 currentPos = environment.agentStart;
        glm::vec3 currentVel(0.0f);
        path.push_back(currentPos);
        
        float totalDistance = 0.0f;
        
        for (int step = 0; step < maxSteps; ++step) {
            float distToGoal = glm::length(environment.goalPosition - currentPos);
            if (distToGoal <= goalTolerance) {
                std::cout << "ORCA: Goal reached in " << step << " steps" << std::endl;
                std::cout << "ORCA: Total distance: " << totalDistance << " units" << std::endl;
                break;
            }
            
            // Create single agent state
            AgentState agent;
            agent.position = currentPos;
            agent.velocity = currentVel;
            agent.prefVelocity = computePreferredVelocity(currentPos, environment.goalPosition, maxSpeed);
            agent.radius = environment.agentRadius;
            agent.id = 0;
            
            // Compute ORCA lines (only obstacles, no other agents)
            std::vector<Line> orcaLines;
            computeObstacleORCALines(agent, environment.obstacles, orcaLines);
            
            // Compute new velocity
            currentVel = linearProgram(orcaLines, agent.prefVelocity, maxSpeed);
            
            // Update position
            glm::vec3 oldPos = currentPos;
            currentPos += currentVel * stepSize;
            totalDistance += glm::length(currentPos - oldPos);
            path.push_back(currentPos);
        }
        
        std::cout << "ORCA: Generated path with " << path.size() << " waypoints" << std::endl;
        return path;
    }
    
    std::vector<std::vector<glm::vec3>> ORCAMAPFStrategy::planMultipleAgentPaths(
        const std::vector<Environment>& environments) {
        
        std::cout << "ORCA: Planning paths for " << environments.size() << " agents..." << std::endl;
        
        int numAgents = environments.size();
        if (numAgents == 0) return {};
        
        // Extract agent data
        std::vector<glm::vec3> positions, goals, velocities;
        std::vector<float> radii;
        std::vector<BoundingBox> obstacles;
        
        for (const auto& env : environments) {
            positions.push_back(env.agentStart);
            goals.push_back(env.goalPosition);
            velocities.push_back(glm::vec3(0.0f));
            radii.push_back(env.agentRadius);
            
            if (obstacles.empty()) {
                obstacles = env.obstacles;
            }
        }
        
        // Initialize paths
        std::vector<std::vector<glm::vec3>> paths(numAgents);
        std::vector<bool> reached(numAgents, false);
        std::vector<float> totalDistances(numAgents, 0.0f);
        
        for (int i = 0; i < numAgents; ++i) {
            paths[i].push_back(positions[i]);
        }
        
        // Main ORCA simulation loop
        for (int step = 0; step < maxSteps; ++step) {
            bool allReached = true;
            std::vector<glm::vec3> newVelocities(numAgents);
            
            // Compute new velocity for each agent
            for (int i = 0; i < numAgents; ++i) {
                if (reached[i]) {
                    newVelocities[i] = glm::vec3(0.0f);
                    continue;
                }
                
                float distToGoal = glm::length(goals[i] - positions[i]);
                if (distToGoal <= goalTolerance) {
                    reached[i] = true;
                    newVelocities[i] = glm::vec3(0.0f);
                    std::cout << "ORCA: Agent " << i << " reached goal at step " << step << std::endl;
                    continue;
                }
                
                allReached = false;
                
                // Compute ORCA velocity for this agent
                newVelocities[i] = computeNewVelocity(i, positions, velocities, goals, radii, obstacles);
            }
            
            // Update all agents simultaneously
            for (int i = 0; i < numAgents; ++i) {
                if (!reached[i]) {
                    velocities[i] = newVelocities[i];
                    glm::vec3 oldPos = positions[i];
                    positions[i] += velocities[i] * stepSize;
                    totalDistances[i] += glm::length(positions[i] - oldPos);
                    paths[i].push_back(positions[i]);
                }
            }
            
            if (allReached) {
                std::cout << "ORCA: All agents reached goals at step " << step << std::endl;
                break;
            }
        }
        
        // Print summary
        std::cout << "ORCA: Multi-Agent Mission Summary:" << std::endl;
        float totalSystemDistance = 0.0f;
        for (int i = 0; i < numAgents; ++i) {
            float directDist = glm::length(goals[i] - environments[i].agentStart);
            float efficiency = (totalDistances[i] > 0.0f) ? (directDist / totalDistances[i] * 100.0f) : 0.0f;
            
            std::cout << "  Agent " << i << ":" << std::endl;
            std::cout << "    Distance: " << totalDistances[i] << " units" << std::endl;
            std::cout << "    Efficiency: " << efficiency << "%" << std::endl;
            std::cout << "    Waypoints: " << paths[i].size() << std::endl;
            
            totalSystemDistance += totalDistances[i];
        }
        std::cout << "  Total system distance: " << totalSystemDistance << " units" << std::endl;
        
        return paths;
    }
    
    std::vector<glm::vec3> ORCAMAPFStrategy::replanPath(const Environment& environment,
                                                        const std::vector<glm::vec3>& currentPath,
                                                        float deltaTime) {
        // ORCA is inherently dynamic - just replan from current position
        Environment replanEnv = environment;
        replanEnv.agentStart = currentPath.empty() ? environment.agentStart : currentPath[0];
        return planPath(replanEnv);
    }
}