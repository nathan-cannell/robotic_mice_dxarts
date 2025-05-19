/**
 * @file dxr_dark_sensor.cpp
 * @author Ethan Widger
 * @brief Darkness sensor reading analog values from a photoresistor voltage
 * divider.
 * @version 0.1
 * @date 2024-03-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "dxr_dark_sensor.h"

void dark_setup() {
  pinMode(DARK_PIN, INPUT);
}

void dark_message_create(std_msgs__msg__Int16* msg) {
  msg->data = analogRead(DARK_PIN);
}