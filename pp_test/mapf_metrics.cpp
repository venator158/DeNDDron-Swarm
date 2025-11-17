#include "mapf_metrics.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

namespace MAPFMetrics {

// ========== SystemMetrics Implementation ==========

void SystemMetrics::calculateFromAgents() {
    if (agentMetrics.empty()) {
        return;
    }
    
    numAgents = agentMetrics.size();
    
    // Reset aggregate values
    makespan = 0.0f;
    sumOfCosts = 0.0f;
    avgPathLength = 0.0f;
    agentsReachedGoal = 0;
    avgEfficiency = 0.0f;
    minEfficiency = 100.0f;
    maxEfficiency = 0.0f;
    avgPathSmoothness = 0.0f;
    totalDirectionChanges = 0;
    totalAgentCollisions = 0;
    totalObstacleCollisions = 0;
    totalNearMisses = 0;
    
    // Calculate aggregates
    for (const auto& agent : agentMetrics) {
        // Makespan is the maximum completion time
        makespan = std::max(makespan, agent.completionTime);
        
        // Sum of costs
        sumOfCosts += agent.pathLength;
        
        // Success tracking
        if (agent.reachedGoal) {
            agentsReachedGoal++;
        }
        
        // Efficiency stats
        if (agent.efficiency > 0.0f) {
            avgEfficiency += agent.efficiency;
            minEfficiency = std::min(minEfficiency, agent.efficiency);
            maxEfficiency = std::max(maxEfficiency, agent.efficiency);
        }
        
        // Path quality
        avgPathSmoothness += agent.pathSmoothness;
        totalDirectionChanges += agent.directionChanges;
        
        // Collisions
        totalAgentCollisions += agent.agentCollisions;
        totalObstacleCollisions += agent.obstacleCollisions;
        totalNearMisses += agent.nearMisses;
    }
    
    // Calculate averages
    avgPathLength = sumOfCosts / numAgents;
    avgEfficiency /= numAgents;
    avgPathSmoothness /= numAgents;
    successRate = (static_cast<float>(agentsReachedGoal) / numAgents) * 100.0f;
    
    // Check collision-free status
    collisionFree = (totalAgentCollisions == 0 && totalObstacleCollisions == 0);
}

void SystemMetrics::print() const {
    std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║               MAPF METRICS REPORT                              ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Algorithm: " << std::left << std::setw(48) << algorithmName << "║\n";
    std::cout << "║ Agents: " << std::setw(7) << numAgents 
              << " | Obstacles: " << std::setw(7) << numObstacles << "                     ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    
    // Essential Metrics
    std::cout << "║ ESSENTIAL METRICS                                              ║\n";
    std::cout << "║ Makespan:           " << std::fixed << std::setprecision(3) << std::setw(10) 
              << makespan << " units                              ║\n";
    std::cout << "║ Sum of Costs:       " << std::setw(10) << sumOfCosts 
              << " units                              ║\n";
    std::cout << "║ Avg Path Length:    " << std::setw(10) << avgPathLength 
              << " units                              ║\n";
    std::cout << "║ Planning Time:      " << std::setprecision(6) << std::setw(10) 
              << planningTime << " seconds                           ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    
    // Success Metrics
    std::cout << "║ SUCCESS METRICS                                                ║\n";
    std::cout << "║ Agents Reached Goal: " << std::setw(3) << agentsReachedGoal 
              << " / " << std::setw(3) << numAgents << " (" << std::setprecision(1) 
              << std::setw(5) << successRate << "%)                    ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    
    // Quality Metrics
    std::cout << "║ QUALITY METRICS                                                ║\n";
    std::cout << "║ Avg Efficiency:     " << std::setprecision(2) << std::setw(6) 
              << avgEfficiency << "% (Min: " << std::setw(6) << minEfficiency 
              << "% Max: " << std::setw(6) << maxEfficiency << "%) ║\n";
    std::cout << "║ Direction Changes:  " << std::setw(10) << totalDirectionChanges 
              << " total                              ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    
    // Collision Metrics
    std::cout << "║ COLLISION METRICS                                              ║\n";
    std::cout << "║ Collision-Free:     " << (collisionFree ? "YES" : "NO ") 
              << "                                             ║\n";
    std::cout << "║ Agent Collisions:   " << std::setw(10) << totalAgentCollisions 
              << "                                   ║\n";
    std::cout << "║ Obstacle Collisions:" << std::setw(10) << totalObstacleCollisions 
              << "                                   ║\n";
    std::cout << "║ Near Misses:        " << std::setw(10) << totalNearMisses 
              << "                                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";
    
    // Per-Agent Details
    if (!agentMetrics.empty()) {
        std::cout << "Per-Agent Details:\n";
        std::cout << "┌──────┬──────────┬────────┬──────────┬───────────┐\n";
        std::cout << "│ Agt  │ Path Len │ Effic. │ Complet. │ Collision │\n";
        std::cout << "├──────┼──────────┼────────┼──────────┼───────────┤\n";
        
        for (const auto& agent : agentMetrics) {
            std::cout << "│ " << std::setw(4) << agent.agentId << " │ "
                      << std::fixed << std::setprecision(2) << std::setw(8) << agent.pathLength << " │ "
                      << std::setw(5) << agent.efficiency << "% │ "
                      << std::setw(8) << agent.completionTime << " │ "
                      << std::setw(9) << (agent.agentCollisions + agent.obstacleCollisions) << " │\n";
        }
        
        std::cout << "└──────┴──────────┴────────┴──────────┴───────────┘\n";
    }
}

void SystemMetrics::printComparisonHeader() {
    std::cout << "\n┌────────────────┬──────────┬──────────┬──────────┬──────────┬──────────┬───────────┐\n";
    std::cout << "│   Algorithm    │ Makespan │   SoC    │ Planning │ Success  │ Avg Eff  │ Collision │\n";
    std::cout << "│                │ (units)  │ (units)  │  (sec)   │   (%)    │   (%)    │   Free    │\n";
    std::cout << "├────────────────┼──────────┼──────────┼──────────┼──────────┼──────────┼───────────┤\n";
}

void SystemMetrics::printComparisonRow() const {
    std::cout << "│ " << std::left << std::setw(14) << algorithmName.substr(0, 14) << " │ "
              << std::right << std::fixed << std::setprecision(2)
              << std::setw(8) << makespan << " │ "
              << std::setw(8) << sumOfCosts << " │ "
              << std::setprecision(4) << std::setw(8) << planningTime << " │ "
              << std::setprecision(1) << std::setw(7) << successRate << "% │ "
              << std::setw(7) << avgEfficiency << "% │ "
              << std::setw(9) << (collisionFree ? "YES" : "NO") << " │\n";
}

void SystemMetrics::saveToCSV(const std::string& filename, bool append) const {
    std::ofstream file;
    bool fileExists = false;
    
    // Check if file exists
    if (append) {
        std::ifstream checkFile(filename);
        fileExists = checkFile.good();
        checkFile.close();
    }
    
    // Open file
    file.open(filename, append ? std::ios::app : std::ios::out);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open CSV file: " << filename << std::endl;
        return;
    }
    
    // Write header if new file
    if (!fileExists || !append) {
        file << "Timestamp,Algorithm,NumAgents,NumObstacles,Makespan,SumOfCosts,AvgPathLength,"
             << "PlanningTime,AgentsReachedGoal,SuccessRate,AvgEfficiency,MinEfficiency,"
             << "MaxEfficiency,AvgSmoothness,DirectionChanges,AgentCollisions,"
             << "ObstacleCollisions,NearMisses,CollisionFree\n";
    }
    
    // Get current timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&time_t_now);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H:%M:%S", tm_now);
    
    // Write data row
    file << timestamp << ","
         << algorithmName << ","
         << numAgents << ","
         << numObstacles << ","
         << std::fixed << std::setprecision(6) << makespan << ","
         << sumOfCosts << ","
         << avgPathLength << ","
         << planningTime << ","
         << agentsReachedGoal << ","
         << successRate << ","
         << avgEfficiency << ","
         << minEfficiency << ","
         << maxEfficiency << ","
         << avgPathSmoothness << ","
         << totalDirectionChanges << ","
         << totalAgentCollisions << ","
         << totalObstacleCollisions << ","
         << totalNearMisses << ","
         << (collisionFree ? 1 : 0) << "\n";
    
    file.close();
    std::cout << "Metrics saved to: " << filename << std::endl;
}

// ========== MetricsCalculator Implementation ==========

float MetricsCalculator::calculatePathLength(const std::vector<glm::vec3>& path) {
    if (path.size() < 2) {
        return 0.0f;
    }
    
    float totalLength = 0.0f;
    for (size_t i = 1; i < path.size(); ++i) {
        totalLength += glm::length(path[i] - path[i-1]);
    }
    
    return totalLength;
}

float MetricsCalculator::calculatePathSmoothness(const std::vector<glm::vec3>& path) {
    if (path.size() < 3) {
        return 0.0f;  // Perfectly smooth (straight line or single point)
    }
    
    float totalAngle = 0.0f;
    int angleCount = 0;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        glm::vec3 v1 = glm::normalize(path[i] - path[i-1]);
        glm::vec3 v2 = glm::normalize(path[i+1] - path[i]);
        
        float dotProduct = glm::dot(v1, v2);
        dotProduct = glm::clamp(dotProduct, -1.0f, 1.0f);
        
        float angle = std::acos(dotProduct) * (180.0f / M_PI);  // Convert to degrees
        totalAngle += angle;
        angleCount++;
    }
    
    return angleCount > 0 ? totalAngle / angleCount : 0.0f;
}

int MetricsCalculator::countDirectionChanges(const std::vector<glm::vec3>& path, 
                                              float thresholdDegrees) {
    if (path.size() < 3) {
        return 0;
    }
    
    int changes = 0;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        glm::vec3 v1 = glm::normalize(path[i] - path[i-1]);
        glm::vec3 v2 = glm::normalize(path[i+1] - path[i]);
        
        float dotProduct = glm::dot(v1, v2);
        dotProduct = glm::clamp(dotProduct, -1.0f, 1.0f);
        
        float angle = std::acos(dotProduct) * (180.0f / M_PI);
        
        if (angle > thresholdDegrees) {
            changes++;
        }
    }
    
    return changes;
}

AgentMetrics MetricsCalculator::calculateAgentMetrics(
    int agentId,
    const glm::vec3& start,
    const glm::vec3& goal,
    const std::vector<glm::vec3>& path,
    float completionTime,
    bool reachedGoal
) {
    AgentMetrics metrics(agentId);
    
    metrics.reachedGoal = reachedGoal;
    metrics.completionTime = completionTime;
    metrics.waypoints = path.size();
    
    // Distance metrics
    metrics.directDistance = glm::length(goal - start);
    metrics.pathLength = calculatePathLength(path);
    
    if (metrics.pathLength > 0.0f && metrics.directDistance > 0.0f) {
        metrics.efficiency = (metrics.directDistance / metrics.pathLength) * 100.0f;
    } else {
        metrics.efficiency = 0.0f;
    }
    
    // Path quality metrics
    metrics.pathSmoothness = calculatePathSmoothness(path);
    metrics.directionChanges = countDirectionChanges(path, 30.0f);
    
    return metrics;
}

int MetricsCalculator::detectAgentCollisions(
    const std::vector<glm::vec3>& agentPositions,
    const std::vector<float>& agentRadii,
    float safetyMargin,
    std::vector<std::pair<int, int>>* collisionPairs
) {
    if (collisionPairs) {
        collisionPairs->clear();
    }
    
    int collisionCount = 0;
    
    for (size_t i = 0; i < agentPositions.size(); ++i) {
        for (size_t j = i + 1; j < agentPositions.size(); ++j) {
            float distance = glm::length(agentPositions[i] - agentPositions[j]);
            float minDistance = (agentRadii[i] + agentRadii[j]) * safetyMargin;
            
            if (distance < minDistance) {
                collisionCount++;
                if (collisionPairs) {
                    collisionPairs->push_back({static_cast<int>(i), static_cast<int>(j)});
                }
            }
        }
    }
    
    return collisionCount;
}

bool MetricsCalculator::checkObstacleCollision(
    const glm::vec3& position,
    float agentRadius,
    const std::vector<glm::vec3>& obstacles,
    const std::vector<glm::vec3>& obstacleSizes
) {
    for (size_t i = 0; i < obstacles.size(); ++i) {
        // Simple bounding box collision check
        glm::vec3 obstacleMin = obstacles[i] - obstacleSizes[i] / 2.0f;
        glm::vec3 obstacleMax = obstacles[i] + obstacleSizes[i] / 2.0f;
        
        // Expand bounding box by agent radius
        obstacleMin -= glm::vec3(agentRadius);
        obstacleMax += glm::vec3(agentRadius);
        
        if (position.x >= obstacleMin.x && position.x <= obstacleMax.x &&
            position.y >= obstacleMin.y && position.y <= obstacleMax.y &&
            position.z >= obstacleMin.z && position.z <= obstacleMax.z) {
            return true;
        }
    }
    
    return false;
}

// ========== MetricsTracker Implementation ==========

MetricsTracker::MetricsTracker() : planningTimerActive(false) {}

void MetricsTracker::startPlanningTimer() {
    planningStartTime = std::chrono::high_resolution_clock::now();
    planningTimerActive = true;
}

void MetricsTracker::stopPlanningTimer() {
    if (!planningTimerActive) {
        return;
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration = endTime - planningStartTime;
    currentMetrics.planningTime = duration.count();
    planningTimerActive = false;
}

void MetricsTracker::initialize(const std::string& algorithmName, int numAgents, int numObstacles) {
    currentMetrics = SystemMetrics();
    currentMetrics.algorithmName = algorithmName;
    currentMetrics.numAgents = numAgents;
    currentMetrics.numObstacles = numObstacles;
    currentMetrics.agentMetrics.resize(numAgents);
    
    // Initialize each agent metrics
    for (int i = 0; i < numAgents; ++i) {
        currentMetrics.agentMetrics[i] = AgentMetrics(i);
    }
}

void MetricsTracker::setAgentMetrics(const std::vector<AgentMetrics>& metrics) {
    currentMetrics.agentMetrics = metrics;
}

void MetricsTracker::recordCollision(int agentId, bool isAgentCollision) {
    if (agentId >= 0 && agentId < static_cast<int>(currentMetrics.agentMetrics.size())) {
        if (isAgentCollision) {
            currentMetrics.agentMetrics[agentId].agentCollisions++;
        } else {
            currentMetrics.agentMetrics[agentId].obstacleCollisions++;
        }
    }
}

void MetricsTracker::recordNearMiss(int agentId) {
    if (agentId >= 0 && agentId < static_cast<int>(currentMetrics.agentMetrics.size())) {
        currentMetrics.agentMetrics[agentId].nearMisses++;
    }
}

SystemMetrics MetricsTracker::finalize() {
    currentMetrics.calculateFromAgents();
    return currentMetrics;
}

} // namespace MAPFMetrics
