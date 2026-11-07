#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>

namespace autopilot {
namespace drivers {
namespace dronecan {

// Standard DroneCAN message data type IDs and signatures.
// IDs come from the file names in https://github.com/DroneCAN/DSDL; signatures were
// computed with tools/dsdl_signature.py and cross-checked against ArduPilot's
// dsdlc-generated headers. A wrong signature makes every multi-frame transfer
// fail its CRC, so do not edit these by hand.
static constexpr uint16_t MSG_ID_NODE_STATUS         = 341;  // uavcan.protocol.NodeStatus
static constexpr uint16_t MSG_ID_RAW_IMU             = 1003; // uavcan.equipment.ahrs.RawIMU  (1000 is ahrs.Solution)
static constexpr uint16_t MSG_ID_MAGNETIC_FIELD      = 1001; // uavcan.equipment.ahrs.MagneticFieldStrength
static constexpr uint16_t MSG_ID_MAGNETIC_FIELD2     = 1002; // uavcan.equipment.ahrs.MagneticFieldStrength2
static constexpr uint16_t MSG_ID_STATIC_PRESSURE     = 1028; // uavcan.equipment.air_data.StaticPressure (1020 is TrueAirspeed)
static constexpr uint16_t MSG_ID_STATIC_TEMPERATURE  = 1029; // uavcan.equipment.air_data.StaticTemperature
static constexpr uint16_t MSG_ID_ESC_RAW_COMMAND     = 1030; // uavcan.equipment.esc.RawCommand
static constexpr uint16_t MSG_ID_GNSS_FIX2           = 1063; // uavcan.equipment.gnss.Fix2 (Fix/1060 is deprecated)

static constexpr uint64_t SIG_NODE_STATUS         = 0x0F0868D0C1A7C6F1ULL;
static constexpr uint64_t SIG_RAW_IMU             = 0x8280632C40E574B5ULL;
static constexpr uint64_t SIG_MAGNETIC_FIELD      = 0xE2A7D4A9460BC2F2ULL;
static constexpr uint64_t SIG_MAGNETIC_FIELD2     = 0xB6AC0C442430297EULL;
static constexpr uint64_t SIG_STATIC_PRESSURE     = 0xCDC7C43412BDC89AULL;
static constexpr uint64_t SIG_STATIC_TEMPERATURE  = 0x49272A6477D96271ULL;
static constexpr uint64_t SIG_ESC_RAW_COMMAND     = 0x217F5C87D7EC951DULL;
static constexpr uint64_t SIG_GNSS_FIX2           = 0xCA41E7000F37435FULL;

/** Returns the data type signature for a message we know how to handle, or 0. */
inline uint64_t signature_for(uint16_t data_type_id) {
    switch (data_type_id) {
        case MSG_ID_NODE_STATUS:        return SIG_NODE_STATUS;
        case MSG_ID_RAW_IMU:            return SIG_RAW_IMU;
        case MSG_ID_MAGNETIC_FIELD:     return SIG_MAGNETIC_FIELD;
        case MSG_ID_MAGNETIC_FIELD2:    return SIG_MAGNETIC_FIELD2;
        case MSG_ID_STATIC_PRESSURE:    return SIG_STATIC_PRESSURE;
        case MSG_ID_STATIC_TEMPERATURE: return SIG_STATIC_TEMPERATURE;
        case MSG_ID_ESC_RAW_COMMAND:    return SIG_ESC_RAW_COMMAND;
        case MSG_ID_GNSS_FIX2:          return SIG_GNSS_FIX2;
        default:                        return 0;
    }
}

// Node Health Codes (uavcan.protocol.NodeStatus.health)
enum class NodeHealth : uint8_t {
    OK = 0,
    WARNING = 1,
    ERROR = 2,
    CRITICAL = 3
};

// Node Operating Modes
enum class NodeMode : uint8_t {
    OPERATIONAL = 0,
    INITIALIZATION = 1,
    MAINTENANCE = 2,
    SOFTWARE_UPDATE = 3,
    OFFLINE = 7
};

/**
 * @brief Unpacks a 29-bit CAN ID into DroneCAN broadcast message fields.
 * Broadcast CAN ID Format:
 * [28:24] Priority (5 bits)
 * [23:8]  Message Data Type ID (16 bits)
 * [7]     Service not message flag (1 bit, 0 for message broadcast)
 * [6:0]   Source Node ID (7 bits, 1 - 127)
 */
struct CANIDFields {
    uint8_t priority{0};
    uint16_t data_type_id{0};
    bool is_service{false};
    uint8_t source_node_id{0};

    static CANIDFields parse(uint32_t can_id) {
        CANIDFields f;
        f.priority = static_cast<uint8_t>((can_id >> 24) & 0x1F);
        f.data_type_id = static_cast<uint16_t>((can_id >> 8) & 0xFFFF);
        f.is_service = ((can_id >> 7) & 0x01) != 0;
        f.source_node_id = static_cast<uint8_t>(can_id & 0x7F);
        return f;
    }

    static uint32_t construct_message_id(uint8_t priority, uint16_t data_type_id, uint8_t source_node_id) {
        return ((static_cast<uint32_t>(priority & 0x1F) << 24) |
                (static_cast<uint32_t>(data_type_id & 0xFFFF) << 8) |
                (static_cast<uint32_t>(source_node_id & 0x7F)));
    }
};

/**
 * @brief Tail byte descriptor (last byte of CAN 2.0B 8-byte frame).
 * Bit 7: Start of Transfer (SOT)
 * Bit 6: End of Transfer (EOT)
 * Bit 5: Toggle bit
 * Bit 4..0: Transfer ID (0 - 31)
 */
struct TailByte {
    bool sot{false};
    bool eot{false};
    bool toggle{false};
    uint8_t transfer_id{0};

    static TailByte parse(uint8_t byte) {
        TailByte tb;
        tb.sot = (byte & 0x80) != 0;
        tb.eot = (byte & 0x40) != 0;
        tb.toggle = (byte & 0x20) != 0;
        tb.transfer_id = byte & 0x1F;
        return tb;
    }

    static uint8_t construct(bool sot, bool eot, bool toggle, uint8_t transfer_id) {
        return (sot ? 0x80 : 0x00) |
               (eot ? 0x40 : 0x00) |
               (toggle ? 0x20 : 0x00) |
               (transfer_id & 0x1F);
    }
};

/**
 * @brief Convert IEEE 754 half-precision (float16) to single-precision float32.
 * Heavily used in DroneCAN messages for bandwidth conservation.
 */
inline float float16_to_float32(uint16_t h) {
    uint32_t sign = (h & 0x8000) << 16;
    uint32_t exp  = (h & 0x7C00) >> 10;
    uint32_t mant = (h & 0x03FF);

    if (exp == 0) {
        if (mant == 0) {
            // Zero
            uint32_t res = sign;
            float f;
            std::memcpy(&f, &res, sizeof(float));
            return f;
        } else {
            // Subnormal
            while (!(mant & 0x0400)) {
                mant <<= 1;
                exp--;
            }
            exp++;
            mant &= ~0x0400;
        }
    } else if (exp == 31) {
        // Inf or NaN
        uint32_t res = sign | 0x7F800000 | (mant << 13);
        float f;
        std::memcpy(&f, &res, sizeof(float));
        return f;
    }

    exp = exp + (127 - 15);
    mant = mant << 13;
    uint32_t res = sign | (exp << 23) | mant;

    float f;
    std::memcpy(&f, &res, sizeof(float));
    return f;
}

/**
 * @brief Convert single-precision float32 to IEEE 754 half-precision (float16).
 */
inline uint16_t float32_to_float16(float f) {
    uint32_t x;
    std::memcpy(&x, &f, sizeof(float));

    uint16_t sign = (x >> 16) & 0x8000;
    int32_t exp = ((x >> 23) & 0xFF) - 127 + 15;
    uint32_t mant = x & 0x007FFFFF;

    if (exp <= 0) {
        return sign;
    } else if (exp >= 31) {
        return sign | 0x7C00; // Inf
    }

    return sign | (static_cast<uint16_t>(exp) << 10) | static_cast<uint16_t>(mant >> 13);
}

} // namespace dronecan
} // namespace drivers
} // namespace autopilot

