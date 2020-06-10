# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_shtc3`
================================================================================

A helper library for using the Senserion SHTC3 Humidity and Temperature Sensor


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* Adafruit's ICM20649 Breakout: https://www.adafruit.com/product/4636

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads



# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_SHTC3.git"

# Common imports remove if unused or pylint will complain
from time import sleep as delay_seconds
from time import monotonic
import adafruit_bus_device.i2c_device as i2c_device

from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct, Struct
from adafruit_register.i2c_bit import RWBit, ROBit
from adafruit_register.i2c_bits import RWBits
from struct import unpack_from
#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>

_SHTC3_DEFAULT_ADDR = 0x70 # SHTC3 I2C Address
_SHTC3_NORMAL_MEAS_TFIRST_STRETCH = 0x7CA2 # Normal measurement, temp first with Clock Stretch Enabled
_SHTC3_LOWPOW_MEAS_TFIRST_STRETCH = 0x6458 # Low power measurement, temp first with Clock Stretch Enabled
_SHTC3_NORMAL_MEAS_HFIRST_STRETCH = 0x5C24 # Normal measurement, hum first with Clock Stretch Enabled
_SHTC3_LOWPOW_MEAS_HFIRST_STRETCH = 0x44DE # Low power measurement, hum first with Clock Stretch Enabled

_SHTC3_NORMAL_MEAS_TFIRST = 0x7866 # Normal measurement, temp first with Clock Stretch disabled
_SHTC3_LOWPOW_MEAS_TFIRST = 0x609C # Low power measurement, temp first with Clock Stretch disabled
_SHTC3_NORMAL_MEAS_HFIRST = 0x58E0 # Normal measurement, hum first with Clock Stretch disabled
_SHTC3_LOWPOW_MEAS_HFIRST = 0x401A # Low power measurement, hum first with Clock Stretch disabled

_SHTC3_READID = 0xEFC8    # Read Out of ID Register
_SHTC3_SOFTRESET = 0x805D # Soft Reset
_SHTC3_SLEEP = 0xB098     # Enter sleep mode
_SHTC3_WAKEUP = 0x3517    # Wakeup mode
_SHTC3_CHIP_ID = 0x807

class SHTC3:
  def __init__(self, i2c_bus, address = _SHTC3_DEFAULT_ADDR):
    self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

    self._buffer = bytearray(6)
    self.reset()
    self.sleep = False
    if (self._chip_id & 0x083F != _SHTC3_CHIP_ID):
      raise RuntimeError("Failed to find an ICM20X sensor - check your wiring!")


  def _write_command(self, command):
    self._buffer[0] = command >> 8
    self._buffer[1] = command & 0xFF

    with self.i2c_device as i2c:
      i2c.write(self._buffer, start=0, end=0)

  @property
  def _chip_id(self): #   readCommand(SHTC3_READID, data, 3);
    self._buffer[0] = _SHTC3_READID >> 8
    self._buffer[1] = _SHTC3_READID & 0xFF

    with self.i2c_device as i2c:
        i2c.write_then_readinto(self._buffer, self._buffer, out_start=0, out_end=2, in_start=0)

    return unpack_from(">H", self._buffer)[0]

  def reset(self):
    self._write_command(_SHTC3_SOFTRESET)
    delay_seconds(0.001)

  @property
  def sleep(self):
    """Determines the sleep state of the sensor"""
    return self._cached_sleep

  @sleep.setter
  def sleep(self, sleep_enabled):
      if sleep_enabled:
        self._write_command(_SHTC3_SLEEP)
      else:
        self._write_command(_SHTC3_WAKEUP)
      delay_seconds(0.001)

# lowPowerMode(bool readmode) { _lpMode = readmode

  @property
  def humidity(self):
    """Current relative humidity in % rH"""
  # sensors_event_t *humidity,
  #                               sensors_event_t *temp) {
    t = monotonic()
    print("reading hum/temp")
    readbuffer = bytearray(6)

    self.sleep = False
    read_bytes = []

    # self._write_command(_SHTC3_LOWPOW_MEAS_TFIRST)
    if False: #lowPower
      self._write_command(_SHTC3_LOWPOW_MEAS_TFIRST)
      delay_seconds(0.001)
    else:
      self._write_command(_SHTC3_NORMAL_MEAS_TFIRST)
      delay_seconds(0.013)

  #   while (!i2c_dev->read(readbuffer, sizeof(readbuffer))) {
  #     delay(1)
    while True:
      with self.i2c_device as i2c:
          print("reading")
          i2c.readinto(self._buffer)
      print("buf:", [hex(i) for i in self._buffer])
      read_bytes = unpack_from(">hbh", self._buffer)
      print("unpacked:", read_bytes)
      if read_bytes[0] != 0 and read_bytes[1] != 0 and read_bytes[2] != 0:
        print ("not all zeros, breaking")
        break
      print ("all zeros, continuing")

    # if (self._buffer[2] != crc8(self._buffer, 2) or
    #     self._buffer[5] != crc8(self._buffer + 3, 2)):
    #   print("NOT CHECKING")

    stemp = read_bytes[0]
    stemp = ((4375 * stemp) >> 14) - 4500
    temperature = stemp / 100.0
    print("temp:", temperature, "stemp:", stemp)

    shum = read_bytes[2]
    shum = (625 * shum) >> 12
    humidity = shum / 100.0

    self.sleep = True

    print("shum:", shum, "humidity:", humidity)
    return (temperature, humidity)
  #   delay_seconds(true)

  #   // use helpers to fill in the events
  #   if (temp)
  #     fillTempEvent(temp, t)
  #   if (humidity)
  #     fillHumidityEvent(humidity, t)
  #   return true
#




# /**
#  * Performs a CRC8 calculation on the supplied values.
#  *
#  * @param data  Pointer to the data to use when calculating the CRC8.
#  * @param len   The number of bytes in 'data'.
#  *
#  * @return The computed CRC8 value.
#  */
# static uint8_t crc8(const uint8_t *data, int len) {
#   /*
#    *
#    * CRC-8 formula from page 14 of SHT spec pdf
#    *
#    * Test data 0xBE, 0xEF should yield 0x92
#    *
#    * Initialization data 0xFF
#    * Polynomial 0x31 (x8 + x5 +x4 +1)
#    * Final XOR 0x00
#    */

#   const uint8_t POLYNOMIAL(0x31)
#   uint8_t crc(0xFF)

#   for (int j = len j --j) {
#     crc ^= *data++

#     for (int i = 8 i --i) {
#       crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1)
#    
#  
#   return crc
#
