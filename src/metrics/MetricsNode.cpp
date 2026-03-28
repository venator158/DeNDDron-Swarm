#include <iostream>
#include <string>
#include <zenoh.hxx>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

using json = nlohmann::json;

class MetricsNode {
public:
    MetricsNode() {
        zenoh::Config config;
        std::cout << "[MetricsNode] Connecting to Zenoh..." << std::endl;
        _session = zenoh::Session::open(std::move(config));
        
        auto sub_opt = zenoh::Session::SubscriberOptions::create_default();
        _sub_collisions = _session.declare_subscriber(
            zenoh::KeyExpr("swarm/metrics/collisions"),
            [this](const zenoh::Sample& sample) {
                this->on_collision_data(sample);
            },
            [](){},
            std::move(sub_opt)
        );
        std::cout << "[MetricsNode] Active." << std::endl;
    }

    void on_collision_data(const zenoh::Sample& sample) {
        std::string payload(reinterpret_cast<const char*>(sample.get_payload().buffers[0].buffer), 
                            sample.get_payload().buffers[0].len);
        try {
            json data = json::parse(payload);
            std::cout << "[Metrics] Collision: " << data.dump() << std::endl;
        } catch (...) {
            std::cout << "[Metrics] Parse error." << std::endl;
        }
    }

private:
    zenoh::Session _session;
    zenoh::Subscriber<void> _sub_collisions;
};

int main() {
    MetricsNode node;
    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
