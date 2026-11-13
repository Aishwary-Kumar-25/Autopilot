#pragma once
/**
 * Simulated flight battery for SITL (test only): a 4S 5000 mAh pack whose
 * open-circuit voltage falls with the charge used and sags with current.
 */
#include <cmath>

namespace autopilot {
namespace sim {

struct SimBattery {
    double used_mah = 0.0;
    double capacity_mah = 5000.0;
    double current_a = 0.0, voltage = 16.8;
    void step(double load, double dt) {   // load: fraction of full power, 0..1
        current_a = 0.4 + 40.0 * std::pow(std::fmax(0.0, load), 1.5);
        used_mah += current_a * dt * (1000.0 / 3600.0);
        const double soc = std::fmax(0.0, 1.0 - used_mah / capacity_mah);
        voltage = 13.6 + 3.2 * soc - 0.012 * current_a;
    }
    int remaining_pct() const {
        return static_cast<int>(std::lround(100.0 * std::fmax(0.0, 1.0 - used_mah / capacity_mah)));
    }
};

} // namespace sim
} // namespace autopilot
