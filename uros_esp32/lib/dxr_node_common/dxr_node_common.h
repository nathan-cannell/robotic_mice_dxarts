/**
 * @file dxr_node_common.h
 * @author Ethan Widger
 * @brief Header that is common to all nodes, with node debug information.
 * @version 0.1
 * @date 2024-02-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DXR_NODE_COMMON_
#define _DXR_NODE_COMMON_
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "dxr_oled_screen.h"

#include "dxr_dark_sensor.h"
#include "dxr_fuel_sensor.h"
#include "dxr_imu.h"
#include "dxr_motors.h"
#include "dxr_presence_sensor.h"
#include "dxr_range_sensor.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#define OBLED_PIN 2
#define NAME_UL 10000
#define SPIN_RATE 20

// microROS function checks preface rclc functions with these checks for error
// protection.
#define RCCHECK(fn)                                      \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK)) {                       \
      print_error(__FILENAME__, __LINE__, (int)temp_rc); \
      error_loop();                                      \
    }                                                    \
  }
#define RCSOFTCHECK(fn)                                  \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK)) {                       \
      print_error(__FILENAME__, __LINE__, (int)temp_rc); \
    }                                                    \
  }

void error_loop();
void dark_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void fuel_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void encoder_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void presence_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void range_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void node_setup();
void node_spin_task(void* pvParameters);

#endif