#!/usr/bin/env python3
"""
mavlink_fake_sender.py - pretend to be the STM32: sends the same MAVLink messages
(HEARTBEAT, ATTITUDE, DEBUG_VECT "COMP", SCALED_IMU) from synthetic data, so you can
test the receiver without hardware.

Terminal 1:  python mavlink_fake_sender.py
Terminal 2:  python animate_filters.py --mavlink udpin:0.0.0.0:14550

pip install pymavlink
"""
import math
import sys
import time

from pymavlink import mavutil
from plot_filters import make_demo

target = sys.argv[1] if len(sys.argv) > 1 else "udpout:127.0.0.1:14550"
m = mavutil.mavlink_connection(target, source_system=1, source_component=200)
data, _ = make_demo(seconds=30.0)
d2r = math.pi / 180.0
print("Sending to", target, "(Ctrl+C to stop)")

t0 = time.time()
last_hb = 0.0
i = 0
while True:
    now = time.time() - t0
    if now - last_hb >= 1.0:
        last_hb = now
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                             mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0,
                             mavutil.mavlink.MAV_STATE_ACTIVE)
    r = data[i % len(data)]
    t_ms = int(now * 1000)
    m.mav.attitude_send(t_ms, r[7] * d2r, r[8] * d2r, 0.0, r[4] * d2r, r[5] * d2r, 0.0)
    m.mav.debug_vect_send(b"COMP", t_ms * 1000, r[9], r[10], 0.0)
    m.mav.scaled_imu_send(t_ms, int(r[1] * 1000), int(r[2] * 1000), int(r[3] * 1000),
                          int(r[4] * d2r * 1000), int(r[5] * d2r * 1000), int(r[6] * d2r * 1000),
                          0, 0, 0)
    i += 4              # data is 200 Hz, we send 50 Hz
    time.sleep(0.02)