#pragma once

#include "../path_planning_strategy.hpp"
#include <string> // Added this include

namespace PathPlanning {
    
    /**
     * A* (A-Star) Strategy
     * An informed search algorithm that uses heuristics to find the optimal path
     * Balances between Dijkstra's guaranteed optimality and greedy best-first speed
     */
    class AStarStrategy : public PathPlanningStrategy {
    private:
        // A* specific parameters
        float gridResolution = 0.5f;  // Grid discretization step for A* pathfinding
        
        // This was not necessary for A* but kept it for consistency with Config handling
        int maxIterations = 10000; 
        
    public:
        std::vector<glm::vec3> planPath(const Environment& environment) override;
        void configure(const Config& config) override;
        std::string getAlgorithmName() const override { return "A*"; }
        void initialize() override;
        void shutdown() override;
    };
}