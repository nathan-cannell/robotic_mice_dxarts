/**
 * @file dxr_dark_sensor.h
 * @author Ethan Widger
 * @brief Header file for the darkness sensor reading analog values from a
 * photoresistor voltage divider.
 * @version 0.1
 * @date 2024-03-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DXR_DARK_SENSOR_
#define _DXR_DARK_SENSOR_
#include <Arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int16.h>
#include "dxr_oled_screen.h"

#define DARK_PIN 36
#define DARK_RATE 500

void dark_setup();
void dark_message_create(std_msgs__msg__Int16* msg);

#endif