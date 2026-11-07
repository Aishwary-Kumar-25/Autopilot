#pragma once

#include "control/actuator_matrix_mixer.hpp"

namespace autopilot {
namespace vehicles {

/**
 * @brief Dynamic Underwater Submarine Thruster Matrix Mixer.
 * Implements generalized control allocation supporting 2 to 16 thrusters:
 * - 3 Thrusters: Simple budget ROV (2 Differential Horizontal + 1 Vertical Heave)
 * - 4 Thrusters: Basic ROV (2 Differential Horizontal + 2 Vertical Roll/Depth)
 * - 6 Thrusters: Standard Vectored BlueROV2 6-DOF
 * - 8 Thrusters: Heavy AUV Decoupled 6-DOF (4 Horizontal + 4 Vertical independent pitch/roll)
 * - Up to 16 Thrusters: Custom / Heavy-workclass ROV configurations
 */
class ROVThrusterMixer : public control::ActuatorMatrixMixer {
public:
    explicit ROVThrusterMixer(uint8_t num_thrusters = 6) {
        configure_rov(num_thrusters);
    }
};

} // namespace vehicles
} // namespace autopilot
