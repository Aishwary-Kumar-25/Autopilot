#pragma once

#include "control/actuator_matrix_mixer.hpp"

namespace autopilot {
namespace vehicles {

/**
 * @brief Dynamic Multirotor Motor Mixer.
 * Supports configurable rotors from 3 up to 16 rotors:
 * - Tricopter (3 rotors)
 * - Quad-X (4 rotors)
 * - Hexa-X (6 rotors)
 * - Octa-X (8 rotors)
 * - Dodecacopter (12 rotors)
 * - Hexadecacopter (16 rotors)
 */
class UAVQuadMixer : public control::ActuatorMatrixMixer {
public:
    explicit UAVQuadMixer(uint8_t num_rotors = 4) {
        configure_uav(num_rotors);
    }
};

} // namespace vehicles
} // namespace autopilot
