/**
 * @file dxr_imu.h
 * @author Ethan Widger
 * @brief Header file for the inertial measurement unit MPU6050 sensor interface
 * for uROS.
 * @version 0.1
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _DXR_IMU_
#define _DXR_IMU_
#include <MPU6050_6Axis_MotionApps20.h>
#include <sensor_msgs/msg/imu.h>
#include "dxr_oled_screen.h"

#define EARTH_GRAVITY 9.80665  // m/s2
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define IMU_RATE 50

// Reset to specific IMU calibrations
#define ACC_XOFFSET -6978
#define ACC_YOFFSET -128
#define ACC_ZOFFSET 1036
#define GYRO_XOFFSET 29
#define GYRO_YOFFSET 10
#define GYRO_ZOFFSET -17

void imu_setup(sensor_msgs__msg__Imu* msg);
void imu_message_create(sensor_msgs__msg__Imu* msg);

#endif