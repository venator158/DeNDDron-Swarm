#pragma once

#include "../path_planning_strategy.hpp"

namespace PathPlanning {
    
    /**
     * RRT (Rapidly-exploring Random Tree) Strategy
     * A probabilistic path planning algorithm that explores the space
     * by growing a tree of random samples towards the goal
     */
    class RRTStrategy : public PathPlanningStrategy {
    private:
        // RRT-specific parameters
        int maxIterations = 1000;
        float stepSize = 1.0f;
        float goalTolerance = 0.5f;
        
    public:
        std::vector<glm::vec3> planPath(const Environment& environment) override;
        void configure(const Config& config) override;
        std::string getAlgorithmName() const override { return "RRT"; }
        void initialize() override;
        void shutdown() override;
    };
}
