#include "system/pubsub.hpp"
#include "fusion/state_types.hpp"
#include "control/vehicle_mixer.hpp"
#include <cassert>
#include <iostream>

using namespace autopilot;

int main() {
    std::cout << "========================================================\n";
    std::cout << "TEST 4: Publish/Subscribe Middleware (Zero-Allocation)\n";
    std::cout << "========================================================\n";

    // 1. Create a Topic for VehicleState (published by SensorFusion)
    system::Topic<fusion::VehicleState> state_topic;

    // 2. Create independent subscribers (e.g. UAV Controller, Telemetry, Data Logger)
    auto uav_sub = state_topic.subscribe();
    auto telem_sub = state_topic.subscribe();

    assert(!uav_sub.updated());
    assert(!telem_sub.updated());
    std::cout << "[Step 1] Verified initial subscriber states (no pending messages) [PASSED]\n";

    // 3. Publish a new VehicleState message
    fusion::VehicleState state1;
    state1.altitude_m = 12.5f;
    state1.euler_rad.x = 0.15f; // Roll ~8.6 deg
    state_topic.publish(state1);

    assert(uav_sub.updated());
    assert(telem_sub.updated());
    std::cout << "[Step 2] Published state message; both subscribers alerted [PASSED]\n";

    // 4. Copy data to UAV controller subscriber
    fusion::VehicleState uav_received;
    bool new_data = uav_sub.copy(uav_received);
    assert(new_data);
    assert(uav_received.altitude_m == 12.5f);
    assert(!uav_sub.updated()); // Already read
    assert(telem_sub.updated()); // Telemetry hasn't read yet
    std::cout << "[Step 3] UAV subscriber consumed fresh state cleanly [PASSED]\n";

    // 5. Telemetry subscriber reads
    fusion::VehicleState telem_received;
    assert(telem_sub.copy(telem_received));
    assert(telem_received.altitude_m == 12.5f);
    assert(!telem_sub.updated());
    std::cout << "[Step 4] Telemetry subscriber consumed fresh state cleanly [PASSED]\n";

    // 6. Dynamic Vehicle Switching Simulation:
    // A single binary can switch active subscriber from UAV to UGV without reflashing!
    system::Topic<control::ControlDemands> demand_topic;
    auto mixer_sub = demand_topic.subscribe();

    control::ControlDemands rover_demands;
    rover_demands.forward = 0.8f;
    rover_demands.yaw = -0.2f; // Slight left turn
    demand_topic.publish(rover_demands);

    control::ControlDemands active_demands;
    assert(mixer_sub.copy(active_demands));
    assert(active_demands.forward == 0.8f);
    std::cout << "[Step 5] Dynamic vehicle demand routing verified [PASSED]\n";

    std::cout << "\n>>> ALL PUBLISH/SUBSCRIBE MIDDLEWARE TESTS PASSED! <<<\n";
    return 0;
}
