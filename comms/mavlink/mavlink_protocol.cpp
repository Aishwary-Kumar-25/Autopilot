#include "comms/mavlink/mavlink_protocol.hpp"
#include "comms/mavlink/mavlink_messages.hpp"

namespace autopilot {
namespace comms {
namespace mavlink {

bool crc_extra_for(uint32_t msgid, uint8_t& crc_extra) {
    // Values from the MAVLink common.xml definitions (checked against pymavlink in
    // tests/test_mavlink.cpp golden frames).
    switch (msgid) {
        case Heartbeat::ID:          crc_extra = Heartbeat::CRC_EXTRA; return true;
        case SysStatus::ID:          crc_extra = SysStatus::CRC_EXTRA; return true;
        case Ping::ID:               crc_extra = Ping::CRC_EXTRA; return true;
        case SetMode::ID:            crc_extra = SetMode::CRC_EXTRA; return true;
        case ParamRequestRead::ID:   crc_extra = ParamRequestRead::CRC_EXTRA; return true;
        case ParamRequestList::ID:   crc_extra = ParamRequestList::CRC_EXTRA; return true;
        case ParamValue::ID:         crc_extra = ParamValue::CRC_EXTRA; return true;
        case ParamSet::ID:           crc_extra = ParamSet::CRC_EXTRA; return true;
        case GpsRawInt::ID:          crc_extra = GpsRawInt::CRC_EXTRA; return true;
        case Attitude::ID:           crc_extra = Attitude::CRC_EXTRA; return true;
        case GlobalPositionInt::ID:  crc_extra = GlobalPositionInt::CRC_EXTRA; return true;
        case ServoOutputRaw::ID:     crc_extra = ServoOutputRaw::CRC_EXTRA; return true;
        case MissionRequestList::ID: crc_extra = MissionRequestList::CRC_EXTRA; return true;
        case MissionCount::ID:       crc_extra = MissionCount::CRC_EXTRA; return true;
        case MissionAck::ID:         crc_extra = MissionAck::CRC_EXTRA; return true;
        case ManualControl::ID:      crc_extra = ManualControl::CRC_EXTRA; return true;
        case VfrHud::ID:             crc_extra = VfrHud::CRC_EXTRA; return true;
        case CommandInt::ID:         crc_extra = CommandInt::CRC_EXTRA; return true;
        case CommandLong::ID:        crc_extra = CommandLong::CRC_EXTRA; return true;
        case CommandAck::ID:         crc_extra = CommandAck::CRC_EXTRA; return true;
        case Timesync::ID:           crc_extra = Timesync::CRC_EXTRA; return true;
        case AutopilotVersion::ID:   crc_extra = AutopilotVersion::CRC_EXTRA; return true;
        case StatusText::ID:         crc_extra = StatusText::CRC_EXTRA; return true;
        case SetupSigning::ID:       crc_extra = SetupSigning::CRC_EXTRA; return true;
        default: return false;
    }
}

size_t encode_v2(uint8_t* out, uint8_t seq, uint8_t sysid, uint8_t compid, uint32_t msgid,
                 const uint8_t* payload, size_t payload_len, uint8_t crc_extra, uint8_t incompat_flags) {
    if (payload_len > MAX_PAYLOAD) payload_len = MAX_PAYLOAD;
    while (payload_len > 1 && payload[payload_len - 1] == 0) --payload_len; // v2 truncation
    out[0] = STX_V2;
    out[1] = static_cast<uint8_t>(payload_len);
    out[2] = incompat_flags;
    out[3] = 0; // compat flags
    out[4] = seq;
    out[5] = sysid;
    out[6] = compid;
    out[7] = static_cast<uint8_t>(msgid & 0xFF);
    out[8] = static_cast<uint8_t>((msgid >> 8) & 0xFF);
    out[9] = static_cast<uint8_t>((msgid >> 16) & 0xFF);
    std::memcpy(out + 10, payload, payload_len);
    uint16_t crc = crc_calculate(out + 1, 9 + payload_len);
    crc = crc_accumulate(crc_extra, crc);
    out[10 + payload_len] = static_cast<uint8_t>(crc & 0xFF);
    out[11 + payload_len] = static_cast<uint8_t>(crc >> 8);
    return 12 + payload_len;
}

bool Parser::parse(uint8_t byte) {
    switch (_state) {
        case State::IDLE:
            if (byte == STX_V2 || byte == STX_V1) {
                _buf[0] = byte;
                _pos = 1;
                _header_len = (byte == STX_V2) ? 10 : 6;
                _state = State::HEADER;
            }
            return false;

        case State::HEADER:
            _buf[_pos++] = byte;
            if (_pos == _header_len) {
                const size_t len = _buf[1];
                if (_buf[0] == STX_V2 && (_buf[2] & ~INCOMPAT_FLAG_SIGNED) != 0) {
                    // Unknown incompatibility flag: we must not try to parse it.
                    _state = State::IDLE;
                    return false;
                }
                _need = _header_len + len + 2;
                _state = (len == 0) ? State::CRC : State::PAYLOAD;
            }
            return false;

        case State::PAYLOAD:
            _buf[_pos++] = byte;
            if (_pos == _need - 2) _state = State::CRC;
            return false;

        case State::CRC:
            _buf[_pos++] = byte;
            if (_pos == _need) {
                if (_buf[0] == STX_V2 && (_buf[2] & INCOMPAT_FLAG_SIGNED)) {
                    _need += SIGNATURE_LEN;
                    _state = State::SIGNATURE;
                    return false;
                }
                _state = State::IDLE;
                return finish();
            }
            return false;

        case State::SIGNATURE:
            _buf[_pos++] = byte;
            if (_pos == _need) {
                _state = State::IDLE;
                return finish();
            }
            return false;
    }
    return false;
}

bool Parser::finish() {
    const bool v2 = _buf[0] == STX_V2;
    const uint8_t len = _buf[1];
    uint32_t msgid;
    uint8_t seq, sysid, compid;
    if (v2) {
        seq = _buf[4]; sysid = _buf[5]; compid = _buf[6];
        msgid = _buf[7] | (static_cast<uint32_t>(_buf[8]) << 8) | (static_cast<uint32_t>(_buf[9]) << 16);
    } else {
        seq = _buf[2]; sysid = _buf[3]; compid = _buf[4]; msgid = _buf[5];
    }
    uint8_t extra;
    if (!crc_extra_for(msgid, extra)) {
        ++_unknown_ids;
        return false;
    }
    const size_t crc_pos = _header_len + len;
    uint16_t crc = crc_calculate(_buf + 1, crc_pos - 1);
    crc = crc_accumulate(extra, crc);
    const uint16_t rx_crc = static_cast<uint16_t>(_buf[crc_pos] | (_buf[crc_pos + 1] << 8));
    if (crc != rx_crc) {
        ++_crc_errors;
        return false;
    }
    _msg.msgid = msgid;
    _msg.sysid = sysid;
    _msg.compid = compid;
    _msg.seq = seq;
    _msg.len = len;
    _msg.v2 = v2;
    _msg.signed_frame = v2 && (_buf[2] & INCOMPAT_FLAG_SIGNED);
    std::memset(_msg.payload, 0, sizeof(_msg.payload));
    std::memcpy(_msg.payload, _buf + _header_len, len);
    _frame_len = _need;

    if (_have_seq) _seq_lost += static_cast<uint8_t>(seq - _last_seq - 1);
    _have_seq = true;
    _last_seq = seq;
    ++_frames_ok;
    return true;
}

} // namespace mavlink
} // namespace comms
} // namespace autopilot
