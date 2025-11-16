#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <chrono>

namespace MAPFMetrics {
    
    /**
     * Per-Agent Metrics
     * Tracks individual agent performance
     */
    struct AgentMetrics {
        int agentId;
        
        // Distance Metrics
        float pathLength;           // Total distance traveled along path
        float directDistance;       // Straight-line distance (start to goal)
        float efficiency;           // directDistance / pathLength * 100 (%)
        
        // Path Quality
        int waypoints;              // Number of waypoints in path
        float pathSmoothness;       // Average turn angle (lower = smoother)
        int directionChanges;       // Number of significant direction changes
        
        // Time Metrics
        float completionTime;       // Time/steps to reach goal (makespan contributor)
        bool reachedGoal;           // Whether agent reached its goal
        
        // Collision Metrics
        int agentCollisions;        // Collisions with other agents
        int obstacleCollisions;     // Collisions with obstacles
        int nearMisses;             // Close calls (within safety margin)
        
        // Constructor
        AgentMetrics(int id = 0) 
            : agentId(id), pathLength(0.0f), directDistance(0.0f), efficiency(0.0f),
              waypoints(0), pathSmoothness(0.0f), directionChanges(0),
              completionTime(0.0f), reachedGoal(false),
              agentCollisions(0), obstacleCollisions(0), nearMisses(0) {}
    };
    
    /**
     * System-Wide MAPF Metrics
     * Aggregates metrics across all agents
     */
    struct SystemMetrics {
        // Algorithm Information
        std::string algorithmName;
        int numAgents;
        int numObstacles;
        
        // Essential MAPF Metrics
        float makespan;             // Max completion time across all agents
        float sumOfCosts;           // Sum of all agent path lengths
        float avgPathLength;        // Average path length per agent
        
        // Success Metrics
        int agentsReachedGoal;      // Number of agents that reached goal
        float successRate;          // Percentage of agents that succeeded
        
        // Efficiency Metrics
        float avgEfficiency;        // Average path efficiency
        float minEfficiency;        // Worst efficiency
        float maxEfficiency;        // Best efficiency
        
        // Planning Metrics
        float planningTime;         // Time to compute paths (seconds)
        
        // Collision Metrics
        int totalAgentCollisions;   // Total agent-agent collisions
        int totalObstacleCollisions;// Total agent-obstacle collisions
        int totalNearMisses;        // Total near-miss events
        bool collisionFree;         // True if no collisions occurred
        
        // Path Quality
        float avgPathSmoothness;    // Average smoothness across agents
        int totalDirectionChanges;  // Total direction changes (all agents)
        
        // Per-Agent Details
        std::vector<AgentMetrics> agentMetrics;
        
        // Constructor
        SystemMetrics() 
            : algorithmName("Unknown"), numAgents(0), numObstacles(0),
              makespan(0.0f), sumOfCosts(0.0f), avgPathLength(0.0f),
              agentsReachedGoal(0), successRate(0.0f),
              avgEfficiency(0.0f), minEfficiency(100.0f), maxEfficiency(0.0f),
              planningTime(0.0f),
              totalAgentCollisions(0), totalObstacleCollisions(0), totalNearMisses(0),
              collisionFree(true),
              avgPathSmoothness(0.0f), totalDirectionChanges(0) {}
        
        /**
         * Calculate aggregate metrics from individual agent metrics
         */
        void calculateFromAgents();
        
        /**
         * Print metrics to console in a formatted way
         */
        void print() const;
        
        /**
         * Print comparison table header
         */
        static void printComparisonHeader();
        
        /**
         * Print single row for comparison table
         */
        void printComparisonRow() const;
        
        /**
         * Save metrics to CSV file
         */
        void saveToCSV(const std::string& filename, bool append = true) const;
    };
    
    /**
     * Metrics Calculator
     * Computes metrics from paths and simulation data
     */
    class MetricsCalculator {
    public:
        /**
         * Calculate path length from waypoints
         */
        static float calculatePathLength(const std::vector<glm::vec3>& path);
        
        /**
         * Calculate path smoothness (average turn angle in degrees)
         * Lower values = smoother path
         */
        static float calculatePathSmoothness(const std::vector<glm::vec3>& path);
        
        /**
         * Count significant direction changes (turns > threshold degrees)
         */
        static int countDirectionChanges(const std::vector<glm::vec3>& path, 
                                         float thresholdDegrees = 30.0f);
        
        /**
         * Calculate metrics for a single agent from its path
         */
        static AgentMetrics calculateAgentMetrics(
            int agentId,
            const glm::vec3& start,
            const glm::vec3& goal,
            const std::vector<glm::vec3>& path,
            float completionTime,
            bool reachedGoal = true
        );
        
        /**
         * Detect collisions between agents at a given time/step
         * Returns number of collisions detected
         */
        static int detectAgentCollisions(
            const std::vector<glm::vec3>& agentPositions,
            const std::vector<float>& agentRadii,
            float safetyMargin = 1.2f,  // Multiplier for collision radius
            std::vector<std::pair<int, int>>* collisionPairs = nullptr
        );
        
        /**
         * Check if path collides with obstacles
         */
        static bool checkObstacleCollision(
            const glm::vec3& position,
            float agentRadius,
            const std::vector<glm::vec3>& obstacles,
            const std::vector<glm::vec3>& obstacleSizes
        );
    };
    
    /**
     * Metrics Tracker
     * Tracks metrics during simulation execution
     */
    class MetricsTracker {
    private:
        SystemMetrics currentMetrics;
        std::chrono::high_resolution_clock::time_point planningStartTime;
        bool planningTimerActive;
        
    public:
        MetricsTracker();
        
        /**
         * Start planning time measurement
         */
        void startPlanningTimer();
        
        /**
         * Stop planning time measurement and record
         */
        void stopPlanningTimer();
        
        /**
         * Initialize metrics for a new run
         */
        void initialize(const std::string& algorithmName, int numAgents, int numObstacles);
        
        /**
         * Set agent metrics
         */
        void setAgentMetrics(const std::vector<AgentMetrics>& metrics);
        
        /**
         * Update collision count during simulation
         */
        void recordCollision(int agentId, bool isAgentCollision);
        
        /**
         * Record near-miss event
         */
        void recordNearMiss(int agentId);
        
        /**
         * Finalize and return metrics
         */
        SystemMetrics finalize();
        
        /**
         * Get current metrics (without finalizing)
         */
        const SystemMetrics& getCurrentMetrics() const { return currentMetrics; }
    };
}
