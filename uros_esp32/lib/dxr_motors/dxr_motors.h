/**
 * @file dxr_motors.h
 * @author Ethan Widger
 * @brief Header for the mouse motors.
 * @version 0.1
 * @date 2024-03-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DXR_MOTORS_
#define _DXR_MOTORS_
#include <Arduino.h>
#include <custom_msgs/msg/encoders.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include "Motor_Controller.h"
#include "dxr_oled_screen.h"

// TB6612FNG Motor Controller Pins
#define AIN1_PIN 27
#define AIN2_PIN 14
#define PWMA_PIN 12

#define BIN1_PIN 26
#define BIN2_PIN 25
#define PWMB_PIN 33

#define STBY_PIN 13

// Motor Encoder Pins
#define AENC1_PIN 35
#define AENC2_PIN 32

#define BENC1_PIN 34
#define BENC2_PIN 39

#define ENCODER_RATE 20
#define MOTOR_RATE 10

#define GEAR_RATIO 51.45  // : 1
#define ENC_PER_REV 28    // rev

const double max_angular_speed = 3.0;   // rev / s
const double wheel_separation = 0.074;  // m
const double wheel_radius = 0.0167;     // m

const double kP = 500;
const double kI = 20000;
const double kD = 1000;

void motor_setup(custom_msgs__msg__Encoders* msg);
void encoder_message_create(custom_msgs__msg__Encoders* msg);
void cmd_vel_sub_callback(const void* msg_in);
void motor_loop();
void motor_task(void* pvParameters);

#endif