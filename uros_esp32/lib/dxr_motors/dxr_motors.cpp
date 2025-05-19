/**
 * @file dxr_motors.cpp
 * @author Ethan Widger
 * @brief
 * @version 0.1
 * @date 2024-03-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "dxr_motors.h"

// Motors
Motor right_motor{AIN1_PIN,  AIN2_PIN, PWMA_PIN,   STBY_PIN,   AENC1_PIN,
                  AENC2_PIN, 1,        GEAR_RATIO, ENC_PER_REV};
Motor left_motor{BIN1_PIN,  BIN2_PIN, PWMB_PIN,   STBY_PIN,   BENC1_PIN,
                 BENC2_PIN, 2,        GEAR_RATIO, ENC_PER_REV};

void motor_setup(custom_msgs__msg__Encoders* msg) {
  // Set up motors
  right_motor.init(kP, kI, kD);
  left_motor.init(kP, kI, kD);

  // Set up encoder msg
  msg->header.frame_id.capacity = 14;
  msg->header.frame_id.data =
      (char*)malloc(msg->header.frame_id.capacity * sizeof(char));
  sprintf(msg->header.frame_id.data, "Motor Encoders");
  msg->header.frame_id.size = strlen(msg->header.frame_id.data);
}

void encoder_message_create(custom_msgs__msg__Encoders* msg) {
  // Set up timestamp
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  msg->header.stamp.sec = ts.tv_sec;
  msg->header.stamp.nanosec = ts.tv_nsec;

  msg->right_position = right_motor.encoder_pos;
  msg->left_position = left_motor.encoder_pos;
  // Linear Velocities of each wheel
  msg->right_vel = right_motor.get_speed() * 2 * PI * wheel_radius;
  msg->left_vel = left_motor.get_speed() * 2 * PI * wheel_radius;
}

void cmd_vel_sub_callback(const void* msg_in) {
  const geometry_msgs__msg__Twist* twist_msg =
      (const geometry_msgs__msg__Twist*)msg_in;

  float v = twist_msg->linear.x;
  float w = twist_msg->angular.z;

  // Compute wheels velocities in rev/s:
  float right_speed;
  float left_speed;
  right_speed = (v - w * wheel_separation / 2.0) / (2 * PI * wheel_radius);
  left_speed = (v + w * wheel_separation / 2.0) / (2 * PI * wheel_radius);

  // Scale to max speed
  if (right_speed == left_speed &&
      (right_speed + left_speed > max_angular_speed * 2)) {
    right_speed = max_angular_speed;
    left_speed = max_angular_speed;
  } else if (right_speed > max_angular_speed) {
    left_speed = (left_speed / right_speed) * max_angular_speed;
    right_speed = max_angular_speed;
  } else if (left_speed > max_angular_speed) {
    right_speed = (right_speed / left_speed) * max_angular_speed;
    left_speed = max_angular_speed;
  }

  // Send to motors
  right_motor.set_speed(right_speed);
  left_motor.set_speed(left_speed);
}

void motor_loop() {
  right_motor.loop();
  left_motor.loop();
}

void motor_task(void* pvParameters) {
  for (;;) {
    right_motor.loop();
    left_motor.loop();
    vTaskDelay(MOTOR_RATE / portTICK_PERIOD_MS);
  }
}