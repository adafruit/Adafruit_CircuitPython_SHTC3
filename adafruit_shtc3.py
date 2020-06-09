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

# Common imports; remove if unused or pylint will complain
from time import sleep
import adafruit_bus_device.i2c_device as i2c_device

from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct, Struct
from adafruit_register.i2c_bit import RWBit, ROBit
from adafruit_register.i2c_bits import RWBits

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

class SHTC3:
  def __init__(self, i2c_bus, address = _SHTC3_DEFAULT_ADDR):
    self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

    self.reset()
    #   reset();
    self._sleep = False

    if ((readID() & 0x083F) != 0x807) {
      return false;
    }

#  * Internal function to perform and I2C write.
#  *
#  * @param cmd   The 16-bit command ID to send.
#  */
  def _write_command(self, command):
    self._buffer.clear() #??
    self._buffer = bytearray(command)
    # MSBYTE first
      self._buffer[0] = command >> 8
      self._buffer[1] = command & 0xFF

    with obj.i2c_device as i2c:
      i2c.write(self._buffer)


# /**
#  * Internal function to perform an I2C read.
#  *
#  * @param cmd   The 16-bit command ID to send.
#  */
# bool Adafruit_SHTC3::readCommand(uint16_t command, uint8_t *buffer,
#                                  uint8_t num_bytes) {

  def _write_command(self, command):
    self._buffer.clear() #??
    self._buffer = bytearray(command)
    # MSBYTE first
    self._buffer[0] = command >> 8
    self._buffer[1] = command & 0xFF

    with obj.i2c_device as i2c:
        i2c.write_then_readinto(self.buffer, self.buffer, out_end=1, in_start=1)


#   cmd[0] = command >> 8;
#   cmd[1] = command & 0xFF;

#   return i2c_dev->write_then_read(cmd, 2, buffer, num_bytes);
# }


# /**
#  * @brief Brings the SHTC3 in or out of sleep mode
#  *
#  * @param sleepmode If true, go into sleep mode. Else, wakeup
#  */
# void Adafruit_SHTC3::sleep(bool sleepmode) {
#   if (sleepmode) {
#     writeCommand(SHTC3_SLEEP);
#   } else {
#     writeCommand(SHTC3_WAKEUP);
#     delayMicroseconds(250);
#   }
# }

# /**
#  * @brief Tells the SHTC3 to read future data in low power (fast) or normal
#  * (precise)
#  *
#  * @param readmode If true, use low power mode for reads
#  */
# void Adafruit_SHTC3::lowPowerMode(bool readmode) { _lpMode = readmode; }

# /**
#  * Gets the ID register contents.
#  *
#  * @return The 16-bit ID register.
#  */
# uint16_t Adafruit_SHTC3::readID(void) {
#   uint8_t data[3];

#   readCommand(SHTC3_READID, data, 3);

#   uint16_t id = data[0];
#   id <<= 8;
#   id |= data[1];

#   return id;
# }

# /**
#  * Performs a reset of the sensor to put it into a known state.
#  */
# void Adafruit_SHTC3::reset(void) {
#   writeCommand(SHTC3_SOFTRESET);
#   delay(1);
# }

# /**************************************************************************/
# /*!
#     @brief  Gets the humidity sensor and temperature values as sensor events
#     @param  humidity Sensor event object that will be populated with humidity
#    data
#     @param  temp Sensor event object that will be populated with temp data
#     @returns true if the event data was read successfully
# */
# /**************************************************************************/
# bool Adafruit_SHTC3::(sensors_event_t *humidity,
#                               sensors_event_t *temp) {
#   uint32_t t = millis();

#   uint8_t readbuffer[6];

#   sleep(false);
#   if (_lpMode) {
#     // low power
#     writeCommand(SHTC3_LOWPOW_MEAS_TFIRST);
#     delay(1);
#   } else {
#     writeCommand(SHTC3_NORMAL_MEAS_TFIRST);
#     delay(13);
#   }

#   while (!i2c_dev->read(readbuffer, sizeof(readbuffer))) {
#     delay(1);
#   }

#   if (readbuffer[2] != crc8(readbuffer, 2) ||
#       readbuffer[5] != crc8(readbuffer + 3, 2))
#     return false;

#   int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
#   // simplified (65536 instead of 65535) integer version of:
#   // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
#   stemp = ((4375 * stemp) >> 14) - 4500;
#   _temperature = (float)stemp / 100.0f;

#   uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
#   // simplified (65536 instead of 65535) integer version of:
#   // humidity = (shum * 100.0f) / 65535.0f;
#   shum = (625 * shum) >> 12;
#   _humidity = (float)shum / 100.0f;

#   sleep(true);

#   // use helpers to fill in the events
#   if (temp)
#     fillTempEvent(temp, t);
#   if (humidity)
#     fillHumidityEvent(humidity, t);
#   return true;
# }




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

#   const uint8_t POLYNOMIAL(0x31);
#   uint8_t crc(0xFF);

#   for (int j = len; j; --j) {
#     crc ^= *data++;

#     for (int i = 8; i; --i) {
#       crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
#     }
#   }
#   return crc;
# }