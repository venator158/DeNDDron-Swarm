#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <zenoh.h>

#include "handlers/JobSelectionHandler.hpp"
#include "handlers/ConsensusHandler.hpp"
#include "handlers/ObstaclePerceptionHandler.hpp"
#include "handlers/PathPlanningHandler.hpp"

namespace denddron {
    struct AgentConfig {
        std::string agent_id;
        std::string zenoh_router_ip;
        float loop_rate_hz = 50.0f;
    };
    class Agent {
        public:
            Agent(const AgentConfig& config);
            ~Agent();

            Agent(const Agent&) = delete;
            Agent& operator=(const Agent&) = delete;

            void run();

            void stop();

        private:
            void setup_comms();
            z_session_t* _session;
            z_publisher_t* _pub_cnmd_vel;
            z_publisher_t* _pub_gossip;
            z_subscriber_t* _sub_sensors;
            z_subscriber_t* _sub_gossip;
            z_subscriber_t* _sub_threats;

            std::unique_ptr<JobSelectionHandler> _enlister;
            std::unique_ptr<ConsensusHandler> _talker;
            std::unique_ptr<ObstaclePerceptionHandler> _eyes;
            std::unique_ptr<PathPlanningHandler> _reflexes;

            AgentConfig _config;
            std::atomic<bool> _running;

            void on_sensor_data(const z_sample_t* sample);
            void on_gossip_message(const z_sample_t* sample);
            void on_threat_update(const z_sample_t* sample);

            void tick_reflexes();
            void tick_brain();
    };
}