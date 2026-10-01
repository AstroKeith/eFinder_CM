#!/usr/bin/python3

# Program to implement read altitude from a BNO08x IMU
# Copyright (C) 2026 Keith Venables.

import time
import math
import quaternion
import numpy as np

from adafruit_bno08x import (BNO_REPORT_ROTATION_VECTOR)
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_extended_bus import ExtendedI2C as I2C

class IMU:
    """The IMU utility class"""
    def __init__(self):
        """Opens channel to the BNO08x IMU """
        try:
            self.bno = BNO08X_I2C(I2C(3),address=0x4A)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            print ("Connected to BNO08x IMU via I2C")
            time.sleep(1)
        except:
            print('failed to open i2c to BNO08x IMU')

    def getAlt(self):
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        q_m = np.normalized(np.quaternion(quat_real, quat_i, quat_j, quat_k))   
        altaz = quaternion.as_euler_angles(q_m)
        alt = altaz[1]*180/math.pi - 90
        alt = max(min(90,alt),0)
        return alt

    