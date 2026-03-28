#pragma once

#include <string>
#include <mutex>
#include <map>
#include <nlohmann/json.hpp>
#include <zenoh.hxx>

class GazeboSimulator {
public:
    GazeboSimulator();
    ~GazeboSimulator();

    void init();
    void step();
    void publish_state();

private:
    void on_motor_cmd(const zenoh::Sample& sample);

    zenoh::Session _session;
    zenoh::Publisher _pub_state;
    zenoh::Subscriber<void> _sub_motors;

    std::map<std::string, std::pair<double, double>> _drone_states; // drone_id -> (x, y)
    std::mutex _state_mtx;
};
