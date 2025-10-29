#include "rrt_strategy.hpp"
#include "../path_planning_interface.hpp" // For Environment, Config
#include <iostream>
#include <vector>
#include <cmath>       // For glm::distance, glm::normalize
#include <random>       // For C++11 random numbers
#include <algorithm>    // For std::reverse
#include <limits>       // For std::numeric_limits

namespace PathPlanning {

    // --- RRT* Implementation Helpers ---

    /**
     * @struct RRTNode
     * @brief Represents a node in the RRT* tree.
     * @note NEW: Added 'cost' member for RRT*.
     */
    struct RRTNode {
        glm::vec3 position;
        int parentIndex; // Index of the parent node in the 'tree' vector
        double cost;     // Cost from the root (start node) to this node

        RRTNode(const glm::vec3& pos, int parent = -1, double c = 0.0) 
            : position(pos), parentIndex(parent), cost(c) {}
    };

    /**
     * @brief Finds the index of the node in the tree that is closest to a given point.
     * (This function is the same as in RRT)
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
     * @brief NEW (for RRT*): Finds all nodes within a given radius of a point.
     * @return A vector of indices of the neighboring nodes.
     */
    static std::vector<int> findNeighbors(const std::vector<RRTNode>& tree, const glm::vec3& point, float radius) {
        std::vector<int> neighbors;
        for (size_t i = 0; i < tree.size(); ++i) {
            if (glm::distance(tree[i].position, point) <= radius) {
                neighbors.push_back(static_cast<int>(i));
            }
        }
        return neighbors;
    }

    /**
     * @brief Checks if a straight-line segment between two points is collision-free.
     * (This function is the same as in RRT)
     */
    static bool isSegmentCollisionFree(const glm::vec3& start, const glm::vec3& end, const Environment& env) {
        const int numSteps = 10; 
        glm::vec3 direction = end - start;
        float segmentLength = glm::length(direction);

        if (segmentLength < 1e-6) {
            return env.isPositionValid(start);
        }

        glm::vec3 stepVector = direction / static_cast<float>(numSteps);

        for (int i = 1; i <= numSteps; ++i) {
            glm::vec3 checkPoint = start + stepVector * static_cast<float>(i);
            if (!env.isPositionValid(checkPoint)) {
                return false; 
            }
        }
        return true; 
    }

    /**
     * @brief Reconstructs the path from the goal node back to the start node.
     * (This function is the same as in RRT)
     */
    static std::vector<glm::vec3> reconstructPath(const std::vector<RRTNode>& tree, int goalNodeIndex, const glm::vec3& goalPos) {
        std::vector<glm::vec3> path;
        path.push_back(goalPos); // Add the actual goal position first

        int currentIndex = goalNodeIndex;
        while (currentIndex != -1) { 
            path.push_back(tree[currentIndex].position);
            currentIndex = tree[currentIndex].parentIndex;
        }
        std::reverse(path.begin(), path.end()); 
        return path;
    }

    // --- End RRT* Helpers ---


    // --- RRTStrategy Class Member Functions ---
    
    void RRTStrategy::initialize() {
        std::cout << "RRT* Strategy: Initialized" << std::endl; // Changed to RRT*
        maxIterations = 5000;
        stepSize = 1.0f;
        goalTolerance = 1.5f;
        // searchRadius = 2.5f; // <-- REMOVED: This variable isn't in the .hpp
    }
    
    void RRTStrategy::shutdown() {
        std::cout << "RRT* Strategy: Shutdown" << std::endl;
    }
    
    void RRTStrategy::configure(const Config& config) {
        maxIterations = config.maxIterations > 0 ? config.maxIterations : 5000;
        stepSize = config.stepSize > 0 ? config.stepSize : 1.0f;
        goalTolerance = config.goalTolerance > 0 ? config.goalTolerance : 1.5f;
        // NEW: Set search radius. A good heuristic is a bit larger than stepSize.
        // searchRadius = stepSize * 2.5f; // <-- REMOVED: This variable isn't in the .hpp

        // Compute a local search radius for reporting since there is no member variable.
        float searchRadius = stepSize * 2.5f;

        std::cout << "RRT* Strategy: Configured" << std::endl;
        std::cout << "  - Max Iterations: " << maxIterations << std::endl;
        std::cout << "  - Step Size: " << stepSize << std::endl;
        std::cout << "  - Search Radius: " << searchRadius << std::endl;
    }
    
    std::vector<glm::vec3> RRTStrategy::planPath(const Environment& environment) {
        std::cout << "RRT*: Planning path..." << std::endl; // Changed to RRT*
        
        // ... (rest of setup is the same)
        glm::vec3 worldMin = environment.worldBounds.center - environment.worldBounds.size / 2.0f;
        glm::vec3 worldMax = environment.worldBounds.center + environment.worldBounds.size / 2.0f;
        static std::mt19937 gen(std::random_device{}()); 
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);
        std::uniform_real_distribution<float> disX(worldMin.x, worldMax.x);
        std::uniform_real_distribution<float> disY(worldMin.y, worldMax.y);
        std::uniform_real_distribution<float> disZ(worldMin.z, worldMax.z);

        // --- RRT* Algorithm ---
        std::vector<RRTNode> tree;
        tree.emplace_back(environment.agentStart, -1, 0.0); // Root node with cost 0

        const float goalBias = 0.1f; 
        
        // --- ADDED: Define searchRadius here as a local variable ---
        // We use the member variable 'stepSize' which was set by configure()
        const float searchRadius = stepSize * 2.5f; 

        for (int i = 0; i < maxIterations; ++i) {
            
            // 1. Sample a random point (with goal bias)
            glm::vec3 randomPoint;
            if (dis(gen) < goalBias) {
                randomPoint = environment.goalPosition;
            } else {
                randomPoint = glm::vec3(disX(gen), disY(gen), disZ(gen));
            }

            // 2. Find nearest node
            int nearestNodeIndex = findNearestNode(tree, randomPoint);
            const RRTNode& nearestNode = tree[nearestNodeIndex];

            // 3. Steer from nearest towards random point
            glm::vec3 direction = glm::normalize(randomPoint - nearestNode.position);
            glm::vec3 newPoint = nearestNode.position + direction * stepSize;

            // 4. --- NEW (RRT*): Choose Best Parent ---
            // We find the best parent (lowest cost) in the search radius.
            
            int bestParentIndex = nearestNodeIndex;
            double minCost = nearestNode.cost + glm::distance(nearestNode.position, newPoint);
            
            // Check if the default path is even valid
            if (!isSegmentCollisionFree(nearestNode.position, newPoint, environment)) {
                continue; // This path is blocked, try a new random point
            }

            std::vector<int> neighborIndices = findNeighbors(tree, newPoint, searchRadius); // <-- This will now work

            for (int neighborIndex : neighborIndices) {
                const RRTNode& neighbor = tree[neighborIndex];
                double newCost = neighbor.cost + glm::distance(neighbor.position, newPoint);
                
                // If this neighbor gives a shorter path AND is collision-free...
                if (newCost < minCost && isSegmentCollisionFree(neighbor.position, newPoint, environment)) {
                    minCost = newCost;
                    bestParentIndex = neighborIndex;
                }
            }

            // 5. Add the new node to the tree with the best parent
            tree.emplace_back(newPoint, bestParentIndex, minCost);
            int newNodeIndex = static_cast<int>(tree.size()) - 1;

            // 6. --- NEW (RRT*): Rewire the Tree ---
            // Check if this new node can be a better parent for any of its neighbors.
            for (int neighborIndex : neighborIndices) {
                if (neighborIndex == bestParentIndex) continue; // Skip the parent

                RRTNode& neighbor = tree[neighborIndex]; // Note: non-const!
                double newCost = minCost + glm::distance(newPoint, neighbor.position);

                // If the path through the new node is shorter AND collision-free...
                if (newCost < neighbor.cost && isSegmentCollisionFree(newPoint, neighbor.position, environment)) {
                    neighbor.parentIndex = newNodeIndex;
                    neighbor.cost = newCost;
                    // In full RRT*, we would also update all children of this neighbor.
                    // This simplified version is often good enough and much faster.
                }
            }
        } // --- End of main loop ---
        
        // 7. --- NEW (RRT*): Find Best Path to Goal After Loop ---
        // RRT* doesn't stop early. It runs all iterations to optimize.
        // Now, we find the node in the tree that is closest to the goal.
        
        std::cout << "RRT*: Ran " << maxIterations << " iterations. Finding best path to goal..." << std::endl;
        
        int bestGoalNodeIndex = -1;
        double minGoalCost = std::numeric_limits<double>::max();

        for (size_t i = 0; i < tree.size(); ++i) {
            float distToGoal = glm::distance(tree[i].position, environment.goalPosition);
            
            // Check if node is "close enough" and the final segment is clear
            if (distToGoal <= goalTolerance && isSegmentCollisionFree(tree[i].position, environment.goalPosition, environment)) {
                double totalCost = tree[i].cost + distToGoal;
                if (totalCost < minGoalCost) {
                    minGoalCost = totalCost;
                    bestGoalNodeIndex = static_cast<int>(i);
                }
            }
        }

        if (bestGoalNodeIndex != -1) {
            std::cout << "RRT*: Found optimal path to goal." << std::endl;
            return reconstructPath(tree, bestGoalNodeIndex, environment.goalPosition);
        }
        
        std::cout << "RRT*: Failed to find a path after " << maxIterations << " iterations." << std::endl;
        return {}; // Return empty path
    }

} // namespace PathPlanning


