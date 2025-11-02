#include "astar_strategy.hpp"
#include "../path_planning_interface.hpp" 
#include <iostream>
#include <vector>
#include <cmath>
#include <queue>         
#include <unordered_map> 
#include <algorithm>     
#include <limits>        
#include <functional>    

// Assuming the necessary headers for glm::vec3 and glm::distance are available
// in your main project environment.

namespace PathPlanning {

    // --- A* Data Structures ---

    struct Node {
        glm::vec3 position;
        float g;            
        float h;            
        float f;            
        Node* parent;       

        Node(const glm::vec3& pos, float g_val = 0.0f, float h_val = 0.0f, Node* p = nullptr) 
            : position(pos), g(g_val), h(h_val), f(g_val + h_val), parent(p) {}
        
        // Custom destructor to help with potential memory cleanup (though explicit cleanup is safer)
        ~Node() { 
            // Note: In large graphs, manual memory management or smart pointers are best.
            // For now, we rely on the search to finish quickly enough.
        }
    };

    struct CompareNode {
        bool operator()(const Node* n1, const Node* n2) const {
            return n1->f > n2->f; 
        }
    };
    
    struct Vec3Hash {
        std::size_t operator()(const glm::vec3& v) const {
            // Using a simple combination for hashing 3D float components
            std::size_t seed = 0;
            seed ^= std::hash<float>{}(v.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<float>{}(v.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<float>{}(v.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
    
    // --- Helper Functions ---

    static float calculateHValue(const glm::vec3& current, const glm::vec3& goal) {
        return glm::distance(current, goal);
    }
    
    /**
     * @brief Checks if a straight-line segment between two grid points is collision-free.
     * This is crucial for avoiding paths that clip corners or pass too close to obstacles.
     * This function is adapted from the robust check used in your RRT* file.
     */
    static bool isPathSegmentCollisionFree(const glm::vec3& start, const glm::vec3& end, const Environment& env) {
        // Reduced steps slightly for A* since points are close, but kept for robustness
        const int numSteps = 5; 
        glm::vec3 direction = end - start;
        float segmentLength = glm::length(direction);

        if (segmentLength < 1e-6) {
            return env.isPositionValid(start);
        }

        glm::vec3 stepVector = direction / static_cast<float>(numSteps);

        for (int i = 1; i <= numSteps; ++i) {
            glm::vec3 checkPoint = start + stepVector * static_cast<float>(i);
            // We assume isPositionValid checks boundaries AND obstacle collision 
            if (!env.isPositionValid(checkPoint)) {
                return false; 
            }
        }
        return true; 
    }

    /**
     * @brief Generates the 26 possible neighbor positions in a 3D grid.
     */
    static std::vector<glm::vec3> generateNeighbors(const glm::vec3& center, float step) {
        std::vector<glm::vec3> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue; 

                    neighbors.emplace_back(
                        center.x + (float)dx * step,
                        center.y + (float)dy * step,
                        center.z + (float)dz * step
                    );
                }
            }
        }
        return neighbors;
    }

    /**
     * @brief Reconstructs the path from the goal node back to the start node.
     */
    static std::vector<glm::vec3> reconstructPath(const Node* goalNode) {
        std::vector<glm::vec3> path;
        const Node* current = goalNode;
        while (current != nullptr) {
            path.push_back(current->position);
            current = current->parent;
        }
        std::reverse(path.begin(), path.end()); 
        return path;
    }


    // --- AStarStrategy Class Member Functions ---
    
    void AStarStrategy::initialize() {
        std::cout << "A* Strategy: Initialized" << std::endl;
    }
    
    void AStarStrategy::shutdown() {
        std::cout << "A* Strategy: Shutdown" << std::endl;
    }
    
    void AStarStrategy::configure(const Config& config) {
        // Using gridResolution as per your astar_strategy.hpp
        gridResolution = config.stepSize > 0.01f ? config.stepSize : 0.5f; 
        maxIterations = config.maxIterations > 0 ? config.maxIterations : 10000;
        
        std::cout << "A* Strategy: Configured" << std::endl;
        std::cout << "  - Grid Resolution: " << gridResolution << std::endl;
        std::cout << "  - Max Iterations (Safety): " << maxIterations << std::endl;
    }
    
    std::vector<glm::vec3> AStarStrategy::planPath(const Environment& environment) {
        std::cout << "A*: Planning path..." << std::endl;
        
        const float goalTolerance = gridResolution * 2.0f; // Goal check tolerance
        
        // Helper to snap positions to the grid for consistent hashing
        auto roundToGrid = [&](const glm::vec3& p) -> glm::vec3 {
            return glm::round(p / gridResolution) * gridResolution;
        };
        
        glm::vec3 startPos = roundToGrid(environment.agentStart);
        glm::vec3 goalPos = environment.goalPosition; 

        // 1. Initialization
        Node* startNode = new Node(startPos);
        startNode->h = calculateHValue(startNode->position, goalPos);
        startNode->f = startNode->g + startNode->h;

        std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
        openList.push(startNode);

        std::unordered_map<glm::vec3, Node*, Vec3Hash> closedList; 
        closedList[startNode->position] = startNode;

        int iterations = 0;
        
        // 2. Main Loop
        while (!openList.empty() && iterations < maxIterations) {
            Node* current = openList.top();
            openList.pop();
            iterations++;

            // 3. Goal Check
            if (glm::distance(current->position, goalPos) <= goalTolerance) {
                // Final segment check: If this segment is clear, we accept the path
                if (isPathSegmentCollisionFree(current->position, goalPos, environment)) {
                    std::cout << "A*: Path found after " << iterations << " iterations." << std::endl;
                    return reconstructPath(current);
                }
            }

            // 4. Neighbor Expansion 
            for (const auto& rawNeighborPos : generateNeighbors(current->position, gridResolution)) {
                
                // CRITICAL FIX: Check if the segment from the parent to the neighbor is collision-free
                // This prevents clipping corners or passing through thin obstacles.
                if (!isPathSegmentCollisionFree(current->position, rawNeighborPos, environment)) {
                    continue; 
                }
                
                // Calculate costs for the neighbor
                float gNew = current->g + glm::distance(current->position, rawNeighborPos);
                float hNew = calculateHValue(rawNeighborPos, goalPos);
                float fNew = gNew + hNew;

                auto it = closedList.find(rawNeighborPos);
                
                if (it == closedList.end()) {
                    // Case A: New node. 
                    Node* neighbor = new Node(rawNeighborPos, gNew, hNew, current);
                    closedList[rawNeighborPos] = neighbor;
                    openList.push(neighbor);
                } else if (gNew < it->second->g) {
                    // Case B: Better path found. Update and re-insert.
                    Node* existingNeighbor = it->second;
                    existingNeighbor->g = gNew;
                    existingNeighbor->f = fNew;
                    existingNeighbor->parent = current;
                    
                    openList.push(existingNeighbor);
                }
            }
        }
        
        // 5. Failure
        std::cout << "A*: Failed to find a path after " << iterations << " iterations." << std::endl;
        return {}; 
    }

} // namespace PathPlanning