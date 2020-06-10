#!/usr/bin/env python
import time
import busio
import board
import adafruit_shtc3
from adafruit_debug_i2c import DebugI2C
i2c = busio.I2C(board.SCL, board.SDA)
i2c = DebugI2C(i2c)
sht = adafruit_shtc3.SHTC3(i2c)
print("got out")
print("hum:", sht.humidity)