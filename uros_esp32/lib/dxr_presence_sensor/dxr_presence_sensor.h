/**
 * @file dxr_presence_sensor.h
 * @author Ethan Widger
 * @brief Header file for the human detection sensor STHS34PF80 to identify
 * movement and human presence
 * @version 0.1
 * @date 2024-03-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DXR_PRESENCE_SENSOR_
#define _DXR_PRESENCE_SENSOR_
#include <Arduino.h>
#include <SparkFun_STHS34PF80_Arduino_Library.h>
#include <custom_msgs/msg/presence.h>
#include <rclc/rclc.h>
#include "dxr_oled_screen.h"

#define PRESENCE_RATE 1000

void presence_setup(custom_msgs__msg__Presence* msg);
void presence_message_create(custom_msgs__msg__Presence* msg);

#endif
