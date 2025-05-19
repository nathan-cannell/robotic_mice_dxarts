/**
 * @file dxr_presence_sensor.cpp
 * @author Ethan Widger
 * @brief The human detection sensor using a STHS34PF80 to identify both
 * movement and a human presence
 * @version 0.1
 * @date 2024-03-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "dxr_presence_sensor.h"

STHS34PF80_I2C presence_sensor;

void presence_setup(custom_msgs__msg__Presence* msg) {
  if (presence_sensor.begin() == false) {
    print_error(__FILENAME__, __LINE__, 42);
    while (1) {
      // Using WiFi transport, show errors with on board LED
      digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
      delay(200);
      digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
      delay(1000);
    }
  }

  // Allocate memory for messages
  msg->header.frame_id.capacity = 12;
  msg->header.frame_id.data =
      (char*)malloc(msg->header.frame_id.capacity * sizeof(char));
  sprintf(msg->header.frame_id.data, "STHS34PF80");
  msg->header.frame_id.size = strlen(msg->header.frame_id.data);

  delay(1000);
}

void presence_message_create(custom_msgs__msg__Presence* msg) {
  int16_t presenceVal = 0;
  int16_t motionVal = 0;
  float temperatureVal = 0;

  sths34pf80_tmos_drdy_status_t dataReady;
  presence_sensor.getDataReady(&dataReady);

  // Check whether sensor has new data - run through loop if data is ready
  if (dataReady.drdy == 1) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    msg->header.stamp.sec = ts.tv_sec;
    msg->header.stamp.nanosec = ts.tv_nsec;

    sths34pf80_tmos_func_status_t status;
    presence_sensor.getStatus(&status);

    if (status.pres_flag == 1) {
      presence_sensor.getPresenceValue(&presenceVal);
      msg->presence = presenceVal;  // Presence Units: cm^-1
    } else {
      msg->presence = -1;
    }

    if (status.mot_flag == 1) {
      msg->motion = true;
    } else {
      msg->motion = false;
    }

    if (status.tamb_shock_flag == 1) {
      presence_sensor.getTemperatureData(&temperatureVal);
      msg->temperature = temperatureVal;
    }
  }
}