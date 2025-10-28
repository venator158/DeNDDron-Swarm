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
        // APF-specific parameters (configured from Config)
        float attractiveForceGain = 1.0f;      // k parameter  
        float repulsiveForceGain = 10.0f;      // General repulsive strength
        float influenceRadius = 2.0f;          // Obstacle influence radius
        float stepSize = 0.1f;                 // Movement step size
        int maxSteps = 1000;                   // Maximum planning iterations
        float goalTolerance = 0.5f;            // Goal reach tolerance
        
        // APF advanced parameters
        float apfExponentialDecay = 0.8f;      // 'a' parameter for e^(-a*d)
        float apfInverseSquareScale = 1.0f;    // 'b' parameter for 1/(b*d²)
        float apfStuckGrowthRate = 0.1f;       // 'alpha' parameter for k growth
        
        // Helper function for obstacle distance calculation
        glm::vec3 getClosestPointOnBoundingBox(const glm::vec3& point, const BoundingBox& box) const;
        
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
