# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Carter Nelson for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

# Simple demo of the VL53L1X distance sensor.
# Will print the sensed range/distance every second.

import time
import board
import adafruit_vl53l1x
import busio
import digitalio

i2c = busio.I2C(board.GP3, board.GP2)  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

xshut = [
    digitalio.DigitalInOut(board.GP15),
    digitalio.DigitalInOut(board.GP16)
    ]


for shutdown_pin in xshut:
    shutdown_pin.switch_to_output(value=False)

sensors = []

for pin_number, shutdown_pin in enumerate(xshut):
    shutdown_pin.value = True
    sensor_i2c = adafruit_vl53l1x.VL53L1X(i2c)
    sensor_i2c.distance_mode = 2
    sensor_i2c.timing_budget = 200
    roi = 10, 10
    sensor_i2c.roi_xy = roi
    sensor_i2c.roi_center = 199
    sensors.append(sensor_i2c)
    
    if pin_number < len (xshut) - 1:
        sensor_i2c.set_address(pin_number + 0x30)

if i2c.try_lock():
    print("Sensor I2C addresses:", [hex(x) for x in i2c.scan()])
    i2c.unlock()


for number, sensor in enumerate(sensors):
    print(f'Sensor: {number + 1},Distance Mode: {sensor.distance_mode}, ROI: {sensor.roi_xy}, ROI Center: {sensor.roi_center}, Timing Budget:{sensor.timing_budget}') 


# Start ranging for sensor data collection.
for sensor in sensors:
    sensor.start_ranging()

LR_dict = {'L-Dist': 0, 'R-Dist': 0}

while True:
    # Extract the appropriate data from the current list, and print
    # the sensor distance readings for all available sensors.
    for sensor_number, sensor in enumerate(sensors):
        if sensor.data_ready:
            distance = sensor.distance
#             print("Sensor {}: {}".format(sensor_number + 1, distance))
            sensor.clear_interrupt()
            if sensor_number + 1 == 1:
                LR_dict['R-Dist'] = sensor.distance
            if sensor_number + 1 == 2:
                LR_dict['L-Dist'] = sensor.distance
        print(LR_dict)
        time.sleep(0.1)

