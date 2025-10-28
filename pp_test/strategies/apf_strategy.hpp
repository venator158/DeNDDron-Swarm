#pragma once

#include "../path_planning_strategy.hpp"

namespace PathPlanning {
    
    /**
     * APF (Artificial Potential Fields) Strategy
     * Uses virtual forces to guide the agent: attraction to goal, repulsion from obstacles
     * Simple but effective for real-time applications, though can get stuck in local minima
     */
    class APFStrategy : public PathPlanningStrategy {
    private:
        // APF-specific parameters
        float attractiveForceGain = 1.0f;
        float repulsiveForceGain = 10.0f;
        float influenceRadius = 5.0f;
        float stepSize = 0.1f;
        int maxSteps = 1000;
        float goalTolerance = 0.5f;
        
    public:
        std::vector<glm::vec3> planPath(const Environment& environment) override;
        void configure(const Config& config) override;
        std::string getAlgorithmName() const override { return "APF (Artificial Potential Fields)"; }
        void initialize() override;
        void shutdown() override;
        
        // APF can support dynamic replanning very efficiently
        std::vector<glm::vec3> replanPath(const Environment& environment,
                                          const std::vector<glm::vec3>& currentPath,
                                          float deltaTime) override;
        bool supportsDynamicReplanning() const override { return true; }
    };
}
