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

from struct import unpack_from
import time
import adafruit_bus_device.i2c_device as i2c_device


# include "Arduino.h"
# include <Adafruit_I2CDevice.h>
# include <Adafruit_Sensor.h>

_SHTC3_DEFAULT_ADDR = 0x70  # SHTC3 I2C Address
_SHTC3_NORMAL_MEAS_TFIRST_STRETCH = (
    0x7CA2  # Normal measurement, temp first with Clock Stretch Enabled
)
_SHTC3_LOWPOW_MEAS_TFIRST_STRETCH = (
    0x6458  # Low power measurement, temp first with Clock Stretch Enabled
)
_SHTC3_NORMAL_MEAS_HFIRST_STRETCH = (
    0x5C24  # Normal measurement, hum first with Clock Stretch Enabled
)
_SHTC3_LOWPOW_MEAS_HFIRST_STRETCH = (
    0x44DE  # Low power measurement, hum first with Clock Stretch Enabled
)

_SHTC3_NORMAL_MEAS_TFIRST = (
    0x7866  # Normal measurement, temp first with Clock Stretch disabled
)
_SHTC3_LOWPOW_MEAS_TFIRST = (
    0x609C  # Low power measurement, temp first with Clock Stretch disabled
)
_SHTC3_NORMAL_MEAS_HFIRST = (
    0x58E0  # Normal measurement, hum first with Clock Stretch disabled
)
_SHTC3_LOWPOW_MEAS_HFIRST = (
    0x401A  # Low power measurement, hum first with Clock Stretch disabled
)

_SHTC3_READID = 0xEFC8  # Read Out of ID Register
_SHTC3_SOFTRESET = 0x805D  # Soft Reset
_SHTC3_SLEEP = 0xB098  # Enter sleep mode
_SHTC3_WAKEUP = 0x3517  # Wakeup mode
_SHTC3_CHIP_ID = 0x807
#   const uint8_t POLYNOMIAL(0x31)
_POLYNOMIAL = 0x31


class SHTC3:
    """
    A driver for the SHTC3 temperature and humidity sensor.
    :param i2c_bus: The `busio.I2C` object to use. This is the only required parameter.
    :param int address: (optional) The I2C address of the device.
    """

    def __init__(self, i2c_bus):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, _SHTC3_DEFAULT_ADDR)

        self._buffer = bytearray(6)
        self.low_power = False
        self.reset()
        self.sleep = False
        if self._chip_id & 0x083F != _SHTC3_CHIP_ID:
            raise RuntimeError("Failed to find an ICM20X sensor - check your wiring!")

    def _write_command(self, command):
        self._buffer[0] = command >> 8
        self._buffer[1] = command & 0xFF

        with self.i2c_device as i2c:
            i2c.write(self._buffer, start=0, end=2)

    @property
    def _chip_id(self):  #   readCommand(SHTC3_READID, data, 3);
        self._buffer[0] = _SHTC3_READID >> 8
        self._buffer[1] = _SHTC3_READID & 0xFF

        with self.i2c_device as i2c:
            i2c.write_then_readinto(
                self._buffer, self._buffer, out_start=0, out_end=2, in_start=0, in_end=2
            )

        return unpack_from(">H", self._buffer)[0]

    def reset(self):
        """Perform a soft reset of the sensor, resetting all settings to their power-on defaults"""
        try:
            self._write_command(_SHTC3_SOFTRESET)

        except RuntimeError as run_err:
            if run_err.args and run_err.args[0] != "I2C slave address was NACK'd":
                raise run_err
        time.sleep(0.001)

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
        time.sleep(0.001)
        self._cached_sleep = sleep_enabled

    # lowPowerMode(bool readmode) { _lpMode = readmode

    @property
    def low_power(self):
        """Enables the less accurate low power mode, trading accuracy for power consumption"""
        return self._low_power

    @low_power.setter
    def low_power(self, low_power_enabled):
        self._low_power = low_power_enabled

    @property
    def relative_humidity(self):
        """Current relative humidity in % rH"""
        return self.measurements[1]

    @property
    def temperature(self):
        """Current temperature in degrees celcius"""
        return self.measurements[0]

    @property
    def measurements(self):
        """both `temperature` and `relative_humidity`, read simultaneously"""

        self.sleep = False
        raw_readings = []

        if self.low_power:
            self._write_command(_SHTC3_LOWPOW_MEAS_TFIRST)
            time.sleep(0.001)
        else:
            self._write_command(_SHTC3_NORMAL_MEAS_TFIRST)
            time.sleep(0.013)

        self._buffer = bytearray(6)
        with self.i2c_device as i2c:
            i2c.readinto(self._buffer)
        raw_readings = unpack_from(">hbh", self._buffer)

        # print("CRC8-ing some beef:", hex(self._crc8(bytearray([0xBE, 0xEF]), 2)))
        # check CRC of bytes
        # if (self._buffer[2] != crc8(self._buffer, 2) or
        #     self._buffer[5] != crc8(self._buffer + 3, 2)):
        #   print("NOT CHECKING")

        raw_temp = raw_readings[0]
        raw_temp = ((4375 * raw_temp) >> 14) - 4500
        temperature = raw_temp / 100.0

        raw_humidity = raw_readings[2]
        raw_humidity = (625 * raw_humidity) >> 12
        humidity = raw_humidity / 100.0

        self.sleep = True
        return (temperature, humidity)

    ## CRC-8 formula from page 14 of SHT spec pdf
    #
    # Test data 0xBE, 0xEF should yield 0x92
    #
    # Initialization data 0xFF
    # Polynomial 0x31 (x8 + x5 +x4 +1)
    # Final XOR 0x00

    # @staticmethod
    # def _crc8(buffer, length):
    #     print("buffer:", buffer)
    #     crc = 0xFF
    #     print("0: crc = ", format(crc, "#010b"))

    #     buff_index = 0
    #     for i in range(length):
    #         curr_buff = buffer[i]
    #         print("buffer[%d]" % i, hex(curr_buff), format(curr_buff, "#010b"))
    #         crc ^= curr_buff
    #         print("1 crc = ", format(crc, "#010b"))

    #         for i in range(8):
    #             print("crc & 0x80: ", format(crc & 0x80, "#010b"))
    #             crc_shift_one = (crc << 1)
    #             if crc & 0x80: # if crc top bit is set
    #                 print("yes")
    #                 crc = crc_shift_one ^ _POLYNOMIAL
    #             else:
    #                 print("no")
    #                 crc = crc_shift_one
    #                 print("3 crc = crc << 1", format(crc, "#010b"))
    #             print("\t*** NEXT BIT **")
    #         print("*** NEXT BYTE **")
    #     return crc
