#!/usr/bin/env python3
"""
Regenerate tests/mavlink_golden.hpp: reference MAVLink v2 frames built by
pymavlink (the reference implementation), used by tests/test_mavlink.cpp to
check our hand-written encoders and decoder byte for byte.

    pip install pymavlink
    python3 tools/gen_mavlink_golden.py > tests/mavlink_golden.hpp

The field values here must match the ones in tests/test_mavlink.cpp.
"""
import os
import sys

os.environ["MAVLINK20"] = "1"
from pymavlink.dialects.v20 import common as mav  # noqa: E402


def frame(msg, seq, sysid=1, compid=1):
    m = mav.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    m.seq = seq
    return bytes(msg.pack(m))


# --- encoders under test (vehicle -> GCS), sysid 1 compid 1 ---
# Keyword arguments: pymavlink's positional order is the XML order, not wire order.
M = mav
TX = [
    ("HEARTBEAT", 0, M.MAVLink_heartbeat_message(
        type=M.MAV_TYPE_QUADROTOR, autopilot=M.MAV_AUTOPILOT_GENERIC, base_mode=1 | 64 | 16 | 128,
        custom_mode=2, system_status=M.MAV_STATE_ACTIVE, mavlink_version=3)),
    ("SYS_STATUS", 1, M.MAVLink_sys_status_message(
        onboard_control_sensors_present=0x1020002B, onboard_control_sensors_enabled=0x1020002B,
        onboard_control_sensors_health=0x10200023, load=125, voltage_battery=15800, current_battery=1234,
        battery_remaining=87, drop_rate_comm=50, errors_comm=7, errors_count1=0, errors_count2=0,
        errors_count3=0, errors_count4=0)),
    ("ATTITUDE", 2, M.MAVLink_attitude_message(
        time_boot_ms=123456, roll=0.1, pitch=-0.2, yaw=3.0, rollspeed=0.01, pitchspeed=-0.02, yawspeed=0.5)),
    ("GLOBAL_POSITION_INT", 3, M.MAVLink_global_position_int_message(
        time_boot_ms=5000, lat=265123000, lon=802329000, alt=128500, relative_alt=2500, vx=120, vy=-35, vz=-10,
        hdg=9000)),
    ("GPS_RAW_INT", 4, M.MAVLink_gps_raw_int_message(
        time_usec=7000000, fix_type=3, lat=265123000, lon=802329000, alt=128500, eph=65535, epv=65535, vel=126,
        cog=34000, satellites_visible=12)),
    ("VFR_HUD", 5, M.MAVLink_vfr_hud_message(
        airspeed=1.25, groundspeed=1.25, heading=271, throttle=48, alt=128.5, climb=-0.4)),
    ("SERVO_OUTPUT_RAW", 6, M.MAVLink_servo_output_raw_message(
        time_usec=4000, port=0, servo1_raw=1500, servo2_raw=1510, servo3_raw=1490, servo4_raw=1505,
        servo5_raw=0, servo6_raw=0, servo7_raw=0, servo8_raw=0)),
    ("PARAM_VALUE_F", 7, M.MAVLink_param_value_message(
        param_id=b"MC_ROLLRATE_P", param_value=0.15, param_type=M.MAV_PARAM_TYPE_REAL32, param_count=22,
        param_index=5)),
    ("PARAM_VALUE_16", 8, M.MAVLink_param_value_message(
        param_id=b"ABCDEFGHIJKLMNOP", param_value=2.5, param_type=M.MAV_PARAM_TYPE_REAL32, param_count=22,
        param_index=21)),
    ("COMMAND_ACK", 9, M.MAVLink_command_ack_message(
        command=400, result=M.MAV_RESULT_FAILED, progress=0, result_param2=0, target_system=255,
        target_component=190)),
    ("STATUSTEXT", 10, M.MAVLink_statustext_message(
        severity=M.MAV_SEVERITY_CRITICAL, text=b"PreArm: no RC input", id=0, chunk_seq=0)),
    ("AUTOPILOT_VERSION", 11, M.MAVLink_autopilot_version_message(
        capabilities=8192 | 2 | 16 | 8 | 4, flight_sw_version=0x01000000, middleware_sw_version=0,
        os_sw_version=0, board_version=0, flight_custom_version=[0] * 8, middleware_custom_version=[0] * 8,
        os_custom_version=[0] * 8, vendor_id=0, product_id=0, uid=0x4543_4C55_4241_5001, uid2=[0] * 18)),
    ("MISSION_COUNT", 12, M.MAVLink_mission_count_message(
        target_system=255, target_component=190, count=0, mission_type=0)),
]

# --- frames our decoder must accept (GCS -> vehicle), sysid 255 compid 190 ---
RX = [
    ("RX_HEARTBEAT_GCS", 40, M.MAVLink_heartbeat_message(
        type=M.MAV_TYPE_GCS, autopilot=M.MAV_AUTOPILOT_INVALID, base_mode=0, custom_mode=0, system_status=0,
        mavlink_version=3)),
    ("RX_ARM", 41, M.MAVLink_command_long_message(
        target_system=1, target_component=1, command=400, confirmation=0, param1=1, param2=0, param3=0,
        param4=0, param5=0, param6=0, param7=0)),
    ("RX_PARAM_SET", 42, M.MAVLink_param_set_message(
        target_system=1, target_component=1, param_id=b"MC_ROLLRATE_P", param_value=0.2,
        param_type=M.MAV_PARAM_TYPE_REAL32)),
    ("RX_MANUAL_CONTROL", 43, M.MAVLink_manual_control_message(
        target=1, x=250, y=-500, z=700, r=100, buttons=0)),
    ("RX_PARAM_REQUEST_READ", 44, M.MAVLink_param_request_read_message(
        target_system=1, target_component=1, param_id=b"FS_RC_TIMEOUT", param_index=-1)),
]


# --- MAVLink 2 signing (tests/test_signing.cpp) ---
SIGN_KEY = bytes(range(32))           # test key 00 01 02 .. 1F
SIGN_TS_TX = 1000000                  # vehicle clock (10 us units since 2015)
SIGN_TS_RX = 500000000                # GCS clock


def signed_frame(msg, seq, sysid, compid, link_id, ts):
    m = mav.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    m.seq = seq
    m.signing.secret_key = SIGN_KEY
    m.signing.sign_outgoing = True
    m.signing.link_id = link_id
    m.signing.timestamp = ts
    return bytes(msg.pack(m))


SIGNED = [
    # vehicle -> GCS: our signer must produce exactly these bytes
    ("SIGNED_TX_HEARTBEAT", signed_frame(M.MAVLink_heartbeat_message(
        type=M.MAV_TYPE_QUADROTOR, autopilot=M.MAV_AUTOPILOT_GENERIC, base_mode=1 | 64 | 16 | 128,
        custom_mode=2, system_status=M.MAV_STATE_ACTIVE, mavlink_version=3), 0, 1, 1, 0, SIGN_TS_TX)),
    # GCS -> vehicle: our verifier must accept these
    ("SIGNED_RX_ARM", signed_frame(M.MAVLink_command_long_message(
        target_system=1, target_component=1, command=400, confirmation=0, param1=1, param2=0, param3=0,
        param4=0, param5=0, param6=0, param7=0), 41, 255, 190, 1, SIGN_TS_RX)),
    ("SIGNED_RX_HEARTBEAT", signed_frame(M.MAVLink_heartbeat_message(
        type=M.MAV_TYPE_GCS, autopilot=M.MAV_AUTOPILOT_INVALID, base_mode=0, custom_mode=0, system_status=0,
        mavlink_version=3), 40, 255, 190, 1, SIGN_TS_RX - 10)),
    # SETUP_SIGNING codec (unsigned, as sent over USB)
    ("SETUP_SIGNING", frame(M.MAVLink_setup_signing_message(
        target_system=1, target_component=1, secret_key=list(SIGN_KEY), initial_timestamp=SIGN_TS_RX), 46,
        255, 190)),
]


def cbytes(b):
    return ", ".join("0x%02X" % x for x in b)


def main():
    out = sys.stdout
    out.write("#pragma once\n// GENERATED by tools/gen_mavlink_golden.py from pymavlink. Do not edit.\n")
    out.write("#include <cstddef>\n#include <cstdint>\n\nnamespace golden {\n")
    out.write("struct Frame { const char* name; const uint8_t* data; size_t len; };\n\n")
    for name, seq, msg in TX:
        b = frame(msg, seq)
        out.write("static const uint8_t %s[] = {%s};\n" % (name, cbytes(b)))
    for name, seq, msg in RX:
        b = frame(msg, seq, 255, 190)
        out.write("static const uint8_t %s[] = {%s};\n" % (name, cbytes(b)))
    # MAVLink 1 framing must be accepted too (older GCS / radios)
    m1 = mav.MAVLink(None, srcSystem=255, srcComponent=190)
    m1.seq = 45
    hb = mav.MAVLink_heartbeat_message(type=M.MAV_TYPE_GCS, autopilot=M.MAV_AUTOPILOT_INVALID, base_mode=0,
                                       custom_mode=0, system_status=0, mavlink_version=3)
    out.write("static const uint8_t RX_V1_HEARTBEAT[] = {%s};\n" % cbytes(bytes(hb.pack(m1, force_mavlink1=True))))
    for name, b in SIGNED:
        out.write("static const uint8_t %s[] = {%s};\n" % (name, cbytes(b)))
    out.write("} // namespace golden\n")


if __name__ == "__main__":
    main()
