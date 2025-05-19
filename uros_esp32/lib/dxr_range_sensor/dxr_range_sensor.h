/**
 * @file dxr_range_sensor.h
 * @author Ethan Widger
 * @brief Header file for the range sensor reading from the VL53L1X range
 * sensor
 * @version 0.1
 * @date 2024-03-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DXR_RANGE_SENSOR_
#define _DXR_RANGE_SENSOR_
#include <Adafruit_VL53L1X.h>
#include <Arduino.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/range.h>
#include "dxr_oled_screen.h"

#define RANGE_RATE 20

void range_setup(sensor_msgs__msg__Range* msg);
void range_message_create(sensor_msgs__msg__Range* msg);

#endif