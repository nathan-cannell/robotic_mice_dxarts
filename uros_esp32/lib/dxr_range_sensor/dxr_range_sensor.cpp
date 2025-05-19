/**
 * @file dxr_range_sensor.cpp
 * @author Ethan Widger
 * @brief The settings for the VL53L1X range sensor set up and publish message
 * creation
 * @version 0.1
 * @date 2024-03-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "dxr_range_sensor.h"

Adafruit_VL53L1X range_sensor;

void range_setup(sensor_msgs__msg__Range* msg) {
  // Set up VL53L1X for range sensing
  if (!range_sensor.begin(0x29, &Wire)) {
    print_error(__FILENAME__, __LINE__, 29);
    while (1) {
      // Using WiFi transport, show errors with on board LED
      digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
      delay(1000);
    }
  }

  // Allocate memory for range message
  msg->header.frame_id.capacity = 8;
  msg->header.frame_id.data =
      (char*)malloc(msg->header.frame_id.capacity * sizeof(char));
  sprintf(msg->header.frame_id.data, "VL53L1X");
  msg->header.frame_id.size = strlen(msg->header.frame_id.data);
  msg->radiation_type = sensor_msgs__msg__Range__INFRARED;
  msg->field_of_view = 0.47;  // 27 degrees or 0.471239 rads
  msg->max_range = 4.0;       // 4 m
  msg->min_range = 0.04;      // 4 cm

  // Set up ranging
  range_sensor.startRanging();
  range_sensor.setTimingBudget(RANGE_RATE);
}

void range_message_create(sensor_msgs__msg__Range* msg) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  msg->header.stamp.sec = ts.tv_sec;
  msg->header.stamp.nanosec = ts.tv_nsec;

  int16_t distance = range_sensor.distance();
  range_sensor.clearInterrupt();

  // Set up ranging range
  msg->range = (float)distance / 1000;  // Convert mm -> m
}