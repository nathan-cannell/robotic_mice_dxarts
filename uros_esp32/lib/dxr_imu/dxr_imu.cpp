/**
 * @file dxr_imu.cpp
 * @author Ethan Widger
 * @brief Inertial measurement unit set up and message constuction for uROS
 * using the MPU6050 sensor.
 * @version 0.1
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "dxr_imu.h"

MPU6050 mpu;
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z] quaternion container
VectorInt16 a;       // [x, y, z] accel sensor measurements
VectorInt16 g;       // [x, y, z] gyro sensor measurements
VectorInt16 aWorld;  // [x, y, z] world-frame accel sensor measurements
VectorInt16 gWorld;  // [x, y, z] world-frame accel sensor measurements

void imu_setup(sensor_msgs__msg__Imu* msg) {
  // Setup IMU and start digital motion processor
  mpu.initialize();
  if (!mpu.testConnection() || mpu.dmpInitialize()) {
    print_error(__FILENAME__, __LINE__, 29);
    while (1) {
      // Using WiFi transport, show errors with on board LED
      digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
      delay(1500);
    }
  }

  // Set up offsets
  mpu.setXAccelOffset(ACC_XOFFSET);
  mpu.setYAccelOffset(ACC_YOFFSET);
  mpu.setZAccelOffset(ACC_ZOFFSET);
  mpu.setXGyroOffset(GYRO_XOFFSET);
  mpu.setYGyroOffset(GYRO_YOFFSET);
  mpu.setZGyroOffset(GYRO_ZOFFSET);

  // Calibrate and enable dmp
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  mpu.setDMPEnabled(true);

  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();

  // Allocate memory for IMU message
  msg->header.frame_id.capacity = 8;
  msg->header.frame_id.data =
      (char*)malloc(msg->header.frame_id.capacity * sizeof(char));
  sprintf(msg->header.frame_id.data, "MPU6050");
  msg->header.frame_id.size = strlen(msg->header.frame_id.data);
}

void imu_message_create(sensor_msgs__msg__Imu* msg) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Set up timestamp
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    msg->header.stamp.sec = ts.tv_sec;
    msg->header.stamp.nanosec = ts.tv_nsec;

    // Get Quaternion from DMP
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    msg->orientation.w = q.w;
    msg->orientation.x = q.x;
    msg->orientation.y = q.y;
    msg->orientation.z = q.z;

    // Calculate worldframe acceleration
    mpu.dmpGetAccel(&a, fifoBuffer);
    mpu.dmpConvertToWorldFrame(&aWorld, &a, &q);
    msg->linear_acceleration.x =
        aWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY;
    msg->linear_acceleration.y =
        aWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY;
    msg->linear_acceleration.z =
        aWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY;

    // Calculate worldframe gyroscope
    mpu.dmpGetGyro(&g, fifoBuffer);
    mpu.dmpConvertToWorldFrame(&gWorld, &g, &q);
    msg->angular_velocity.x = gWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
    msg->angular_velocity.y = gWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
    msg->angular_velocity.z = gWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD;
  }
}