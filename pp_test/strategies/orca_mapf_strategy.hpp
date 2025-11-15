#pragma once
#include "../path_planning_strategy.hpp"
#include <vector>
#include <cmath>

namespace PathPlanning {
    
    /**
     * ORCA (Optimal Reciprocal Collision Avoidance) MAPF Strategy
     * 
     * Real-time velocity-based collision avoidance for multiple agents.
     * Each agent computes a velocity that:
     * 1. Moves toward its goal
     * 2. Avoids collisions with other agents (reciprocal responsibility)
     * 3. Avoids static obstacles
     * 
     * Very efficient: O(n) per agent, scales to 100+ agents
     * Industry standard for games, robotics, and crowd simulation
     */
    class ORCAMAPFStrategy : public PathPlanningStrategy {
    private:
        // Configuration
        float timeHorizon = 2.0f;          // Time to look ahead for agent collisions
        float timeHorizonObst = 1.0f;      // Time to look ahead for obstacle collisions
        float maxSpeed = 3.0f;             // Maximum agent speed
        float maxAccel = 5.0f;             // Maximum acceleration
        float neighborDist = 5.0f;         // Distance to consider other agents
        int maxNeighbors = 10;             // Max number of neighbors to consider
        float stepSize = 0.1f;             // Time step for simulation
        int maxSteps = 1000;               // Maximum simulation steps
        float goalTolerance = 0.5f;        // Goal reach threshold
        
        // ===== ORCA LINE REPRESENTATION =====
        // Each ORCA line represents a half-plane of forbidden velocities
        struct Line {
            glm::vec3 point;      // Point on the line
            glm::vec3 direction;  // Direction of the line (normalized)
        };
        
        // ===== AGENT STATE FOR SIMULATION =====
        struct AgentState {
            glm::vec3 position;
            glm::vec3 velocity;
            glm::vec3 prefVelocity;  // Preferred velocity (toward goal)
            float radius;
            int id;
        };
        
        // ===== HELPER FUNCTIONS =====
        
        /**
         * Compute preferred velocity for an agent (direction toward goal)
         */
        glm::vec3 computePreferredVelocity(const glm::vec3& position, 
                                           const glm::vec3& goal,
                                           float maxSpeed) const;
        
        /**
         * Compute ORCA lines for agent-agent collision avoidance
         */
        void computeAgentORCALines(const AgentState& agent,
                                   const std::vector<AgentState>& allAgents,
                                   std::vector<Line>& orcaLines) const;
        
        /**
         * Compute ORCA lines for agent-obstacle collision avoidance
         */
        void computeObstacleORCALines(const AgentState& agent,
                                      const std::vector<BoundingBox>& obstacles,
                                      std::vector<Line>& orcaLines) const;
        
        /**
         * Find the closest point on a line segment to a given point
         */
        glm::vec3 closestPointOnSegment(const glm::vec3& point,
                                        const glm::vec3& segStart,
                                        const glm::vec3& segEnd) const;
        
        /**
         * Linear programming to find optimal velocity satisfying ORCA constraints
         * Finds velocity closest to preferred velocity that satisfies all ORCA lines
         */
        glm::vec3 linearProgram(const std::vector<Line>& lines,
                               const glm::vec3& prefVelocity,
                               float maxSpeed) const;
        
        /**
         * Project velocity onto a line (helper for linear programming)
         */
        glm::vec3 projectOnLine(const glm::vec3& velocity,
                               const Line& line) const;
        
        /**
         * Check if velocity is on the correct side of the line
         */
        bool isValidVelocity(const glm::vec3& velocity,
                            const Line& line) const;
        
        /**
         * Find intersection of velocity with line
         */
        float intersectLineWithVelocity(const Line& line,
                                       const glm::vec3& velocity,
                                       const glm::vec3& prefVelocity) const;
        
        /**
         * 2D cross product (for plane calculations)
         */
        float cross2D(const glm::vec2& a, const glm::vec2& b) const {
            return a.x * b.y - a.y * b.x;
        }
        
        /**
         * Determinant helper (3D version handled as 2D in XZ plane)
         */
        float det(const glm::vec3& v1, const glm::vec3& v2) const;
        
    public:
        // ===== STRATEGY INTERFACE =====
        
        void initialize() override;
        void shutdown() override;
        void configure(const Config& config) override;
        std::string getAlgorithmName() const override { 
            return "ORCA (Optimal Reciprocal Collision Avoidance)"; 
        }
        
        // Single agent planning (fallback - uses simplified ORCA)
        std::vector<glm::vec3> planPath(const Environment& environment) override;
        
        // Multi-agent planning (main ORCA functionality)
        std::vector<std::vector<glm::vec3>> planMultipleAgentPaths(
            const std::vector<Environment>& environments) override;
        
        bool supportsMultiAgent() const override { return true; }
        bool supportsDynamicReplanning() const override { return true; }
        
        // Dynamic replanning for ORCA
        std::vector<glm::vec3> replanPath(const Environment& environment,
                                          const std::vector<glm::vec3>& currentPath,
                                          float deltaTime) override;
        
        // ===== REAL-TIME SIMULATION SUPPORT =====
        // These functions allow the simulation to call ORCA in real-time
        
        /**
         * Compute optimal velocity for a single agent given current state
         * This is the core ORCA function for real-time use
         */
        glm::vec3 computeNewVelocity(int agentIndex,
                                     const std::vector<glm::vec3>& positions,
                                     const std::vector<glm::vec3>& velocities,
                                     const std::vector<glm::vec3>& goals,
                                     const std::vector<float>& radii,
                                     const std::vector<BoundingBox>& obstacles) const;
    };
}