# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Carter Nelson for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_vl53l1x`
================================================================================

CircuitPython module for interacting with the VL53L1X distance sensor.


* Author(s): Carter Nelson

Implementation Notes
--------------------

**Hardware:**

* Adafruit `VL53L1X Time of Flight Distance Sensor - ~30 to 4000mm
  <https://www.adafruit.com/product/3967>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import time
import struct
from adafruit_bus_device import i2c_device
from micropython import const

__version__ = "1.2.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_VL53L1X.git"

_VL53L1X_I2C_SLAVE_DEVICE_ADDRESS = const(0x0001)
_VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = const(0x0008)
_GPIO_HV_MUX__CTRL = const(0x0030)
_GPIO__TIO_HV_STATUS = const(0x0031)
_PHASECAL_CONFIG__TIMEOUT_MACROP = const(0x004B)
_RANGE_CONFIG__TIMEOUT_MACROP_A_HI = const(0x005E)
_RANGE_CONFIG__VCSEL_PERIOD_A = const(0x0060)
_RANGE_CONFIG__TIMEOUT_MACROP_B_HI = const(0x0061)
_RANGE_CONFIG__VCSEL_PERIOD_B = const(0x0063)
_RANGE_CONFIG__VALID_PHASE_HIGH = const(0x0069)
_SD_CONFIG__WOI_SD0 = const(0x0078)
_SD_CONFIG__INITIAL_PHASE_SD0 = const(0x007A)
_ROI_CONFIG__USER_ROI_CENTRE_SPAD = const(0x007F)
_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE = const(0x0080)
_SYSTEM__INTERRUPT_CLEAR = const(0x0086)
_SYSTEM__MODE_START = const(0x0087)
_VL53L1X_RESULT__RANGE_STATUS = const(0x0089)
_VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = const(0x0096)
_VL53L1X_IDENTIFICATION__MODEL_ID = const(0x010F)

TB_SHORT_DIST = {
    # ms: (MACROP_A_HI, MACROP_B_HI)
    15: (b"\x00\x1D", b"\x00\x27"),
    20: (b"\x00\x51", b"\x00\x6E"),
    33: (b"\x00\xD6", b"\x00\x6E"),
    50: (b"\x01\xAE", b"\x01\xE8"),
    100: (b"\x02\xE1", b"\x03\x88"),
    200: (b"\x03\xE1", b"\x04\x96"),
    500: (b"\x05\x91", b"\x05\xC1"),
}

TB_LONG_DIST = {
    # ms: (MACROP_A_HI, MACROP_B_HI)
    20: (b"\x00\x1E", b"\x00\x22"),
    33: (b"\x00\x60", b"\x00\x6E"),
    50: (b"\x00\xAD", b"\x00\xC6"),
    100: (b"\x01\xCC", b"\x01\xEA"),
    200: (b"\x02\xD9", b"\x02\xF8"),
    500: (b"\x04\x8F", b"\x04\xA4"),
}


class VL53L1X:
    """Driver for the VL53L1X distance sensor."""

    def __init__(self, i2c, address=41):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        self._i2c = i2c
        model_id, module_type, mask_rev = self.model_info
        if model_id != 0xEA or module_type != 0xCC or mask_rev != 0x10:
            raise RuntimeError("Wrong sensor ID or type!")
        self._sensor_init()
        self._timing_budget = None
        self.timing_budget = 50

    def _sensor_init(self):
        # pylint: disable=line-too-long
        init_seq = bytes(
            [  # value    addr : description
                0x00,  # 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
                0x00,  # 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
                0x00,  # 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
                0x01,  # 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity()
                0x02,  # 0x31 : bit 1 = interrupt depending on the polarity
                0x00,  # 0x32 : not user-modifiable
                0x02,  # 0x33 : not user-modifiable
                0x08,  # 0x34 : not user-modifiable
                0x00,  # 0x35 : not user-modifiable
                0x08,  # 0x36 : not user-modifiable
                0x10,  # 0x37 : not user-modifiable
                0x01,  # 0x38 : not user-modifiable
                0x01,  # 0x39 : not user-modifiable
                0x00,  # 0x3a : not user-modifiable
                0x00,  # 0x3b : not user-modifiable
                0x00,  # 0x3c : not user-modifiable
                0x00,  # 0x3d : not user-modifiable
                0xFF,  # 0x3e : not user-modifiable
                0x00,  # 0x3f : not user-modifiable
                0x0F,  # 0x40 : not user-modifiable
                0x00,  # 0x41 : not user-modifiable
                0x00,  # 0x42 : not user-modifiable
                0x00,  # 0x43 : not user-modifiable
                0x00,  # 0x44 : not user-modifiable
                0x00,  # 0x45 : not user-modifiable
                0x20,  # 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
                0x0B,  # 0x47 : not user-modifiable
                0x00,  # 0x48 : not user-modifiable
                0x00,  # 0x49 : not user-modifiable
                0x02,  # 0x4a : not user-modifiable
                0x0A,  # 0x4b : not user-modifiable
                0x21,  # 0x4c : not user-modifiable
                0x00,  # 0x4d : not user-modifiable
                0x00,  # 0x4e : not user-modifiable
                0x05,  # 0x4f : not user-modifiable
                0x00,  # 0x50 : not user-modifiable
                0x00,  # 0x51 : not user-modifiable
                0x00,  # 0x52 : not user-modifiable
                0x00,  # 0x53 : not user-modifiable
                0xC8,  # 0x54 : not user-modifiable
                0x00,  # 0x55 : not user-modifiable
                0x00,  # 0x56 : not user-modifiable
                0x38,  # 0x57 : not user-modifiable
                0xFF,  # 0x58 : not user-modifiable
                0x01,  # 0x59 : not user-modifiable
                0x00,  # 0x5a : not user-modifiable
                0x08,  # 0x5b : not user-modifiable
                0x00,  # 0x5c : not user-modifiable
                0x00,  # 0x5d : not user-modifiable
                0x01,  # 0x5e : not user-modifiable
                0xCC,  # 0x5f : not user-modifiable
                0x0F,  # 0x60 : not user-modifiable
                0x01,  # 0x61 : not user-modifiable
                0xF1,  # 0x62 : not user-modifiable
                0x0D,  # 0x63 : not user-modifiable
                0x01,  # 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm
                0x68,  # 0x65 : Sigma threshold LSB
                0x00,  # 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
                0x80,  # 0x67 : Min count Rate LSB
                0x08,  # 0x68 : not user-modifiable
                0xB8,  # 0x69 : not user-modifiable
                0x00,  # 0x6a : not user-modifiable
                0x00,  # 0x6b : not user-modifiable
                0x00,  # 0x6c : Intermeasurement period MSB, 32 bits register
                0x00,  # 0x6d : Intermeasurement period
                0x0F,  # 0x6e : Intermeasurement period
                0x89,  # 0x6f : Intermeasurement period LSB
                0x00,  # 0x70 : not user-modifiable
                0x00,  # 0x71 : not user-modifiable
                0x00,  # 0x72 : distance threshold high MSB (in mm, MSB+LSB)
                0x00,  # 0x73 : distance threshold high LSB
                0x00,  # 0x74 : distance threshold low MSB ( in mm, MSB+LSB)
                0x00,  # 0x75 : distance threshold low LSB
                0x00,  # 0x76 : not user-modifiable
                0x01,  # 0x77 : not user-modifiable
                0x0F,  # 0x78 : not user-modifiable
                0x0D,  # 0x79 : not user-modifiable
                0x0E,  # 0x7a : not user-modifiable
                0x0E,  # 0x7b : not user-modifiable
                0x00,  # 0x7c : not user-modifiable
                0x00,  # 0x7d : not user-modifiable
                0x02,  # 0x7e : not user-modifiable
                0xC7,  # 0x7f : ROI center
                0xFF,  # 0x80 : XY ROI (X=Width, Y=Height)
                0x9B,  # 0x81 : not user-modifiable
                0x00,  # 0x82 : not user-modifiable
                0x00,  # 0x83 : not user-modifiable
                0x00,  # 0x84 : not user-modifiable
                0x01,  # 0x85 : not user-modifiable
                0x00,  # 0x86 : clear interrupt, 0x01=clear
                0x00,  # 0x87 : ranging, 0x00=stop, 0x40=start
            ]
        )
        self._write_register(0x002D, init_seq)
        self.start_ranging()
        while not self.data_ready:
            time.sleep(0.01)
        self.clear_interrupt()
        self.stop_ranging()
        self._write_register(_VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, b"\x09")
        self._write_register(0x0B, b"\x00")

    @property
    def model_info(self):
        """A 3 tuple of Model ID, Module Type, and Mask Revision."""
        info = self._read_register(_VL53L1X_IDENTIFICATION__MODEL_ID, 3)
        return (info[0], info[1], info[2])  # Model ID, Module Type, Mask Rev

    @property
    def distance(self):
        """The distance in units of centimeters."""
        if self._read_register(_VL53L1X_RESULT__RANGE_STATUS)[0] != 0x09:
            return None
        dist = self._read_register(
            _VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 2
        )
        dist = struct.unpack(">H", dist)[0]
        return dist / 10

    def start_ranging(self):
        """Starts ranging operation."""
        self._write_register(_SYSTEM__MODE_START, b"\x40")

    def stop_ranging(self):
        """Stops ranging operation."""
        self._write_register(_SYSTEM__MODE_START, b"\x00")

    def clear_interrupt(self):
        """Clears new data interrupt."""
        self._write_register(_SYSTEM__INTERRUPT_CLEAR, b"\x01")

    @property
    def data_ready(self):
        """Returns true if new data is ready, otherwise false."""
        if (
            self._read_register(_GPIO__TIO_HV_STATUS)[0] & 0x01
            == self._interrupt_polarity
        ):
            return True
        return False

    @property
    def timing_budget(self):
        """Ranging duration in milliseconds. Increasing the timing budget
        increases the maximum distance the device can range and improves
        the repeatability error. However, average power consumption augments
        accordingly. ms = 15 (short mode only), 20, 33, 50, 100, 200, 500.
        Defaults to 50."""
        return self._timing_budget

    @timing_budget.setter
    def timing_budget(self, val):
        reg_vals = None
        mode = self.distance_mode
        if mode == 1:
            reg_vals = TB_SHORT_DIST
        if mode == 2:
            reg_vals = TB_LONG_DIST
        if reg_vals is None:
            raise RuntimeError("Unknown distance mode.")
        if val not in reg_vals:
            raise ValueError("Invalid timing budget.")
        self._write_register(_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, reg_vals[val][0])
        self._write_register(_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, reg_vals[val][1])
        self._timing_budget = val

    @property
    def _interrupt_polarity(self):
        int_pol = self._read_register(_GPIO_HV_MUX__CTRL)[0] & 0x10
        int_pol = (int_pol >> 4) & 0x01
        return 0 if int_pol else 1

    @property
    def distance_mode(self):
        """The distance mode. 1=short (up to 136cm) , 2=long (up to 360cm)."""
        mode = self._read_register(_PHASECAL_CONFIG__TIMEOUT_MACROP)[0]
        if mode == 0x14:
            return 1  # short distance
        if mode == 0x0A:
            return 2  # long distance
        return None  # unknown

    @distance_mode.setter
    def distance_mode(self, mode):
        if mode == 1:
            # short distance
            self._write_register(_PHASECAL_CONFIG__TIMEOUT_MACROP, b"\x14")
            self._write_register(_RANGE_CONFIG__VCSEL_PERIOD_A, b"\x07")
            self._write_register(_RANGE_CONFIG__VCSEL_PERIOD_B, b"\x05")
            self._write_register(_RANGE_CONFIG__VALID_PHASE_HIGH, b"\x38")
            self._write_register(_SD_CONFIG__WOI_SD0, b"\x07\x05")
            self._write_register(_SD_CONFIG__INITIAL_PHASE_SD0, b"\x06\x06")
        elif mode == 2:
            # long distance
            self._write_register(_PHASECAL_CONFIG__TIMEOUT_MACROP, b"\x0A")
            self._write_register(_RANGE_CONFIG__VCSEL_PERIOD_A, b"\x0F")
            self._write_register(_RANGE_CONFIG__VCSEL_PERIOD_B, b"\x0D")
            self._write_register(_RANGE_CONFIG__VALID_PHASE_HIGH, b"\xB8")
            self._write_register(_SD_CONFIG__WOI_SD0, b"\x0F\x0D")
            self._write_register(_SD_CONFIG__INITIAL_PHASE_SD0, b"\x0E\x0E")
        else:
            raise ValueError("Unsupported mode.")
        self.timing_budget = self._timing_budget

    @property
    def roi_xy(self):
        """Returns the x and y coordinates of the sensor's region of interest"""
        temp = self._read_register(_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE)

        x = (int.from_bytes(temp, 'big') & 0x0F) + 1
        y = ((int.from_bytes(temp, 'big') & 0xF0) >> 4) + 1

        return x, y

    @roi_xy.setter
    def roi_xy(self, data):
        x, y = data
        optical_center = 199

        x = min(x, 16)
        y = min(y, 16)

        if x > 10 or y > 10:
            optical_center = 199

        self._write_register(
            _ROI_CONFIG__USER_ROI_CENTRE_SPAD, optical_center.to_bytes(1, 'big')
        )
        self._write_register(
            _ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
            ((y - 1) << 4 | (x - 1)).to_bytes(1, 'big'),
        )

    @property
    def roi_center(self):
        """Returns the center of the sensor's region of interest"""
        temp = self._read_register(_ROI_CONFIG__USER_ROI_CENTRE_SPAD)
        return int.from_bytes(temp,'big')

    @roi_center.setter
    def roi_center(self, center):
        self._write_register(_ROI_CONFIG__USER_ROI_CENTRE_SPAD, center.to_bytes(1, 'big'))

    def _write_register(self, address, data, length=None):
        if length is None:
            length = len(data)
        with self.i2c_device as i2c:
            i2c.write(struct.pack(">H", address) + data[:length])

    def _read_register(self, address, length=1):
        data = bytearray(length)
        with self.i2c_device as i2c:
            i2c.write(struct.pack(">H", address))
            i2c.readinto(data)
        return data

    def set_address(self, new_address):
        """
        Set a new I2C address to the instantaited object. This is only called when using
        multiple VL53L0X sensors on the same I2C bus (SDA & SCL pins). See also the
        `example <examples.html#multiple-vl53l1x-on-same-i2c-bus>`_ for proper usage.
        """
        self._write_register(
            _VL53L1X_I2C_SLAVE_DEVICE_ADDRESS, struct.pack(">B", new_address)
        )
        self.i2c_device = i2c_device.I2CDevice(self._i2c, new_address)
