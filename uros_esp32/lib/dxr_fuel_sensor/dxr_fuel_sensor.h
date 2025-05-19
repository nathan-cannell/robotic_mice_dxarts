/**
 * @file dxr_fuel_sensor.h
 * @author Ethan Widger
 * @brief
 * @version 0.1
 * @date 2024-04-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DXR_FUEL_SENSOR_
#define _DXR_FUEL_SENSOR_
#include <Adafruit_MAX1704X.h>
#include <Arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include "dxr_oled_screen.h"

#define FUEL_RATE 10000

void fuel_setup();
void fuel_message_create(std_msgs__msg__Float32* msg);

#endif