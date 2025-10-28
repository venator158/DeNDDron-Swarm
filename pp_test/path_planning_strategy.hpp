#pragma once

#include "path_planning_interface.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <string>

namespace PathPlanning {
    
    /**
     * Base strategy interface for path planning algorithms
     * Each algorithm implements this interface to provide a unified API
     */
    class PathPlanningStrategy {
    public:
        virtual ~PathPlanningStrategy() = default;
        
        /**
         * Core planning method - each algorithm implements this
         * @param environment Complete environment data
         * @return Vector of waypoints representing the path
         */
        virtual std::vector<glm::vec3> planPath(const Environment& environment) = 0;
        
        /**
         * Multi-agent planning support (optional override)
         * Default implementation plans each agent independently
         */
        virtual std::vector<std::vector<glm::vec3>> planMultipleAgentPaths(
            const std::vector<Environment>& environments) {
            std::vector<std::vector<glm::vec3>> paths;
            paths.reserve(environments.size());
            for (const auto& env : environments) {
                paths.push_back(planPath(env));
            }
            return paths;
        }
        
        /**
         * Dynamic replanning support (optional override)
         * Default implementation does a full replan
         */
        virtual std::vector<glm::vec3> replanPath(const Environment& environment,
                                                  const std::vector<glm::vec3>& currentPath,
                                                  float deltaTime) {
            return planPath(environment);
        }
        
        /**
         * Algorithm-specific configuration
         */
        virtual void configure(const Config& config) {}
        
        /**
         * Algorithm name for debugging and identification
         */
        virtual std::string getAlgorithmName() const = 0;
        
        /**
         * Algorithm-specific initialization
         */
        virtual void initialize() {}
        
        /**
         * Algorithm-specific cleanup
         */
        virtual void shutdown() {}
        
        /**
         * Algorithm capabilities
         */
        virtual bool supportsMultiAgent() const { return false; }
        virtual bool supportsDynamicReplanning() const { return true; }
        virtual bool supportsAnytime() const { return false; }
    };
}
