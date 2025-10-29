#pragma once

#include "../path_planning_strategy.hpp"

namespace PathPlanning {
    
    /**
     * APF MAPF (Artificial Potential Fields Multi-Agent Path Finding) Strategy
     * Uses virtual forces for multi-agent coordination: attraction to goal, repulsion from obstacles and other agents
     * Extends APF for simultaneous multi-agent planning with inter-agent collision avoidance
     */
    class APFMAPFStrategy : public PathPlanningStrategy {
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
        
        // MAPF-specific parameters
        float agentRepulsiveGain = 2.0f;       // Strength of agent-to-agent repulsion
        float agentInfluenceRadius = 3.0f;     // Distance at which agents affect each other
        
        // Helper functions for single agent planning
        glm::vec3 getClosestPointOnBoundingBox(const glm::vec3& point, const BoundingBox& box) const;
        
        // Helper functions for single agent planning
        glm::vec3 calculateObstacleRepulsiveForce(const glm::vec3& currentPos,
                                                 const std::vector<BoundingBox>& obstacles) const;
        
    public:
        std::vector<glm::vec3> planPath(const Environment& environment) override;
        void configure(const Config& config) override;
        std::string getAlgorithmName() const override { return "APF MAPF (Multi-Agent Potential Fields)"; }
        void initialize() override;
        void shutdown() override;
        
        // APF can support dynamic replanning very efficiently
        std::vector<glm::vec3> replanPath(const Environment& environment,
                                          const std::vector<glm::vec3>& currentPath,
                                          float deltaTime) override;
        bool supportsDynamicReplanning() const override { return true; }
        
        // MAPF (Multi-Agent Path Finding) support
        std::vector<std::vector<glm::vec3>> planMultipleAgentPaths(
            const std::vector<Environment>& environments) override;
        bool supportsMultiAgent() const override { return true; }
        
        // Public force calculation methods for real-time simulation
        glm::vec3 calculateTotalForceForAgent(int agentIndex,
                                             const std::vector<glm::vec3>& allAgentPositions,
                                             const std::vector<glm::vec3>& goalPositions,
                                             const std::vector<BoundingBox>& obstacles,
                                             const std::vector<float>& agentRadii) const;
        
        glm::vec3 calculateAgentRepulsiveForce(int agentIndex,
                                              const std::vector<glm::vec3>& allAgentPositions,
                                              const std::vector<float>& agentRadii) const;
    };
}
