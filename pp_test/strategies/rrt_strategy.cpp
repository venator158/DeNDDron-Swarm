#include "rrt_strategy.hpp"
#include "../path_planning_interface.hpp" // For Environment, Config
#include <iostream>
#include <vector>
#include <cmath>       // For glm::distance, glm::normalize
#include <random>       // For C++11 random numbers
#include <algorithm>    // For std::reverse
#include <limits>       // For std::numeric_limits

namespace PathPlanning {

    // --- RRT Implementation Helpers ---
    // These are helper functions and structs just for this file.
    // They are not part of the RRTStrategy class itself.

    /**
     * @struct RRTNode
     * @brief A simple struct to represent a node in the RRT tree.
     */
    struct RRTNode {
        glm::vec3 position;
        int parentIndex; // Index of the parent node in the 'tree' vector

        RRTNode(const glm::vec3& pos, int parent = -1) 
            : position(pos), parentIndex(parent) {}
    };

    /**
     * @brief Finds the index of the node in the tree that is closest to a given point.
     */
    static int findNearestNode(const std::vector<RRTNode>& tree, const glm::vec3& point) {
        int nearestIndex = -1;
        float minDistance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < tree.size(); ++i) {
            float distance = glm::distance(tree[i].position, point);
            if (distance < minDistance) {
                minDistance = distance;
                nearestIndex = static_cast<int>(i);
            }
        }
        return nearestIndex;
    }

    /**
     * @brief Checks if a straight-line segment between two points is collision-free.
     */
    static bool isSegmentCollisionFree(const glm::vec3& start, const glm::vec3& end, const Environment& env) {
        // Check a few points along the line segment for collisions
        const int numSteps = 10; 
        glm::vec3 direction = end - start;
        float segmentLength = glm::length(direction);

        // If the segment is very short, just check the end point
        if (segmentLength < 1e-6) {
            return env.isPositionValid(start);
        }

        glm::vec3 stepVector = direction / static_cast<float>(numSteps);

        for (int i = 1; i <= numSteps; ++i) {
            glm::vec3 checkPoint = start + stepVector * static_cast<float>(i);
            // Use the environment's built-in collision checker
            if (!env.isPositionValid(checkPoint)) {
                return false; // Collision detected
            }
        }
        return true; // No collision
    }

    /**
     * @brief Reconstructs the path from the goal node back to the start node.
     */
    static std::vector<glm::vec3> reconstructPath(const std::vector<RRTNode>& tree, int goalNodeIndex, const glm::vec3& goalPos) {
        std::vector<glm::vec3> path;
        
        // Add the actual goal position first
        path.push_back(goalPos);

        int currentIndex = goalNodeIndex;
        while (currentIndex != -1) { // -1 is the parent of the root node
            path.push_back(tree[currentIndex].position);
            currentIndex = tree[currentIndex].parentIndex;
        }
        
        // The path is currently Goal -> Start, so we reverse it
        std::reverse(path.begin(), path.end()); 
        return path;
    }

    // --- End RRT Helpers ---


    // --- RRTStrategy Class Member Functions ---
    
    void RRTStrategy::initialize() {
        std::cout << "RRT Strategy: Initialized" << std::endl;
        // Set default values in case configure isn't called
        maxIterations = 5000;
        stepSize = 1.0f;
        goalTolerance = 1.5f;
        // goalBias = 0.1f; // <-- REMOVED: This variable isn't in the .hpp
    }
    
    void RRTStrategy::shutdown() {
        std::cout << "RRT Strategy: Shutdown" << std::endl;
    }
    
    void RRTStrategy::configure(const Config& config) {
        // Use values from config, with sane defaults
        maxIterations = config.maxIterations > 0 ? config.maxIterations : 5000;
        stepSize = config.stepSize > 0 ? config.stepSize : 1.0f;
        goalTolerance = config.goalTolerance > 0 ? config.goalTolerance : 1.5f;
        // goalBias = 0.1f; // <-- REMOVED: This variable isn't in the .hpp
        std::cout << "RRT Strategy: Configured" << std::endl;
        std::cout << "  - Max Iterations: " << maxIterations << std::endl;
        std::cout << "  - Step Size: " << stepSize << std::endl;
    }
    
    std::vector<glm::vec3> RRTStrategy::planPath(const Environment& environment) {
        std::cout << "RRT: Planning path..." << std::endl;
        std::cout << "Start: (" << environment.agentStart.x << ", " << environment.agentStart.y << ", " << environment.agentStart.z << ")" << std::endl;
        std::cout << "Goal: (" << environment.goalPosition.x << ", " << environment.goalPosition.y << ", " << environment.goalPosition.z << ")" << std::endl;

        // Get world bounds for random sampling
        glm::vec3 worldMin = environment.worldBounds.center - environment.worldBounds.size / 2.0f;
        glm::vec3 worldMax = environment.worldBounds.center + environment.worldBounds.size / 2.0f;

        // Setup Random Number Generation
        // We make this 'static' so it's only seeded once, not every time planPath is called
        static std::mt19937 gen(std::random_device{}()); 
        std::uniform_real_distribution<float> dis(0.0f, 1.0f); // For goal bias
        std::uniform_real_distribution<float> disX(worldMin.x, worldMax.x);
        std::uniform_real_distribution<float> disY(worldMin.y, worldMax.y);
        std::uniform_real_distribution<float> disZ(worldMin.z, worldMax.z);

        // --- RRT Algorithm ---
        std::vector<RRTNode> tree;
        tree.emplace_back(environment.agentStart, -1); // Root node

        // We use the *member variables* (maxIterations, goalBias, etc.) 
        // that were set by the initialize() or configure() functions.
        
        // --- ADDED: Define goalBias here as a local constant ---
        const float goalBias = 0.1f; 

        for (int i = 0; i < maxIterations; ++i) {
            
            // 1. Sample a random point (with goal bias)
            glm::vec3 randomPoint;
            if (dis(gen) < goalBias) { // <-- This will now work
                randomPoint = environment.goalPosition;
            } else {
                randomPoint = glm::vec3(disX(gen), disY(gen), disZ(gen));
            }

            // 2. Find nearest node in tree
            // (Uses the static helper function from above)
            int nearestNodeIndex = findNearestNode(tree, randomPoint);
            const RRTNode& nearestNode = tree[nearestNodeIndex];

            // 3. Steer from nearest towards random point
            glm::vec3 direction = glm::normalize(randomPoint - nearestNode.position);
            glm::vec3 newPoint = nearestNode.position + direction * stepSize; // Use member var stepSize

            // 4. Check collision, add to tree if valid
            // (Uses the static helper function from above)
            if (isSegmentCollisionFree(nearestNode.position, newPoint, environment)) {
                
                tree.emplace_back(newPoint, nearestNodeIndex);
                int newNodeIndex = static_cast<int>(tree.size()) - 1;

                // 5. Check if goal reached
                if (glm::distance(newPoint, environment.goalPosition) <= goalTolerance) { // Use member var goalTolerance
                    
                    if (isSegmentCollisionFree(newPoint, environment.goalPosition, environment)) {
                        std::cout << "RRT: Goal reached in " << i << " iterations." << std::endl;
                        // (Uses the static helper function from above)
                        return reconstructPath(tree, newNodeIndex, environment.goalPosition);
                    }
                }
            }
        }
        
        std::cout << "RRT: Failed to find a path after " << maxIterations << " iterations." << std::endl;
        return {}; // Return empty path
    }

} // namespace PathPlanning


