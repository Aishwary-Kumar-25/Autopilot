"""A tiny external physics simulator speaking the ArduPilot JSON SITL protocol (plain Python).

It is the smallest possible stand-in for Gazebo: autopilot_sitl --physics json
sends it motor PWM, it integrates a quadcopter for one step and answers with
the state, in lock-step. Use it to check the external-physics path without
installing Gazebo, or as a template for your own physics (a different airframe,
a payload, a tether, ...).

    python3 -m autopilot_sim.json_physics_quad           # listens on UDP 9002
    build/autopilot_sitl --physics json --rc-neutral       # in another terminal

Motor order and spin as ArduPilot / our Quad-X mixer: 1 front-right CCW,
2 rear-left CCW, 3 front-left CW, 4 rear-right CW. ESC range 1100..1900 us.
"""
import argparse
import json
import math
import socket
import struct
import sys

SERVO_FMT = "<HHI16H"
SERVO_LEN = struct.calcsize(SERVO_FMT)   # 40 bytes
MAGIC_16 = 18458


def qmul(a, b):
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def rotate(q, v):
    """Body -> world for a body->world quaternion."""
    p = qmul(qmul(q, (0.0, v[0], v[1], v[2])), (q[0], -q[1], -q[2], -q[3]))
    return p[1:]


def rotate_inv(q, v):
    return rotate((q[0], -q[1], -q[2], -q[3]), v)


class Quad:
    mass = 1.5
    inertia = (0.029, 0.029, 0.055)
    arm = 0.18
    max_thrust = 8.0          # N per motor -> hover ~46 %
    kq = 0.016
    tau = 0.04
    drag = 0.3
    g = 9.80665

    def __init__(self, pwm_min=1100, pwm_max=1900):
        self.pwm_min, self.pwm_max = pwm_min, pwm_max
        a = self.arm * math.sqrt(0.5)
        # (x forward, y right) per motor, yaw reaction sign (+1 = CCW prop)
        self.motors = [((+a, +a), +1), ((-a, -a), +1), ((+a, -a), -1), ((-a, +a), -1)]
        self.thrust = [0.0] * 4
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.w = [0.0, 0.0, 0.0]
        self.accel_ned = [0.0, 0.0, 0.0]
        self.t = 0.0

    def step(self, pwm, dt):
        tz = 0.0
        torque = [0.0, 0.0, 0.0]
        for i, ((x, y), spin) in enumerate(self.motors):
            us = pwm[i]
            cmd = 0.0 if us == 0 else min(1.0, max(0.0, (us - self.pwm_min) / float(self.pwm_max - self.pwm_min)))
            self.thrust[i] += (cmd * self.max_thrust - self.thrust[i]) * dt / (self.tau + dt)
            f = self.thrust[i]
            tz += f
            # r x F with F = (0, 0, -f) in FRD
            torque[0] += -y * f
            torque[1] += x * f
            torque[2] += spin * self.kq * f
        torque = [torque[k] - 0.003 * self.w[k] for k in range(3)]
        thrust_ned = rotate(self.q, (0.0, 0.0, -tz))
        acc = [thrust_ned[k] / self.mass - self.drag * self.vel[k] / self.mass for k in range(3)]
        acc[2] += self.g
        on_ground = self.pos[2] >= 0.0 and acc[2] >= 0.0
        if on_ground:
            acc = [0.0, 0.0, 0.0]
            self.vel = [0.0, 0.0, 0.0]
            self.w = [0.0, 0.0, 0.0]
            self.pos[2] = 0.0
            yaw = math.atan2(2 * (self.q[0] * self.q[3] + self.q[1] * self.q[2]),
                             1 - 2 * (self.q[2] ** 2 + self.q[3] ** 2))
            self.q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        else:
            for k in range(3):
                self.vel[k] += acc[k] * dt
                self.pos[k] += self.vel[k] * dt
            if self.pos[2] > 0.0:
                self.pos[2] = 0.0
                self.vel = [0.0, 0.0, 0.0]
            I = self.inertia
            w = self.w
            Iw = (I[0] * w[0], I[1] * w[1], I[2] * w[2])
            gyro = (w[1] * Iw[2] - w[2] * Iw[1], w[2] * Iw[0] - w[0] * Iw[2], w[0] * Iw[1] - w[1] * Iw[0])
            self.w = [w[k] + (torque[k] - gyro[k]) / I[k] * dt for k in range(3)]
            dq = qmul(self.q, (0.0, self.w[0], self.w[1], self.w[2]))
            q = [self.q[k] + 0.5 * dt * dq[k] for k in range(4)]
            n = math.sqrt(sum(c * c for c in q))
            self.q = tuple(c / n for c in q)
        self.accel_ned = acc
        self.t += dt

    def state_json(self):
        f = rotate_inv(self.q, (self.accel_ned[0], self.accel_ned[1], self.accel_ned[2] - self.g))
        return json.dumps({
            "timestamp": self.t,
            "imu": {"gyro": list(self.w), "accel_body": list(f)},
            "position": list(self.pos),
            "quaternion": list(self.q),
            "velocity": list(self.vel),
        })


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=9002)
    ap.add_argument("--rate", type=float, default=1000.0, help="physics steps per second (default 1000)")
    ap.add_argument("--pwm", default="1100:1900", help="ESC range MIN:MAX")
    args = ap.parse_args(argv)
    lo, hi = (int(x) for x in args.pwm.split(":"))
    quad = Quad(lo, hi)
    dt = 1.0 / args.rate
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    print("json_physics_quad: waiting for servo packets on UDP %d (%.0f Hz physics)" % (args.port, args.rate))
    sys.stdout.flush()
    last_frame = None
    while True:
        data, addr = sock.recvfrom(1024)
        if len(data) < SERVO_LEN:
            continue
        magic, _rate, frame, *pwm = struct.unpack(SERVO_FMT, data[:SERVO_LEN])
        if magic != MAGIC_16:
            continue
        if frame != last_frame:          # a repeated frame = the controller is re-asking: don't step twice
            if last_frame is not None and frame < last_frame:
                quad = Quad(lo, hi)      # controller restarted
            quad.step(pwm, dt)
            last_frame = frame
        sock.sendto(("\n" + quad.state_json() + "\n").encode(), addr)


if __name__ == "__main__":
    main()
