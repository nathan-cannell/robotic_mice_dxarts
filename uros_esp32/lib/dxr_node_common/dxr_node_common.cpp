/**
 * @file dxr_node_common.cpp
 * @author Ethan Widger
 * @brief Common node functions for all microROS nodes.
 * @version 0.1
 * @date 2024-02-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <dxr_node_common.h>

rcl_allocator_t allocator;
rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;

rcl_publisher_t dark_publisher;
rcl_timer_t dark_timer;
std_msgs__msg__Int16 dark_msg;

rcl_publisher_t enc_publisher;
rcl_timer_t enc_timer;
custom_msgs__msg__Encoders encoders_msg;

rcl_publisher_t fuel_publisher;
rcl_timer_t fuel_timer;
std_msgs__msg__Float32 fuel_msg;

rcl_publisher_t imu_publisher;
rcl_timer_t imu_timer;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t presence_publisher;
rcl_timer_t presence_timer;
custom_msgs__msg__Presence presence_msg;

rcl_publisher_t range_publisher;
rcl_timer_t range_timer;
sensor_msgs__msg__Range range_msg;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

/// @brief Common error loop, flashes on board LED if there is an error.
void error_loop() {
  while (1) {
    // Using WiFi transport, show errors with on board LED
    digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
    delay(200);
  }
}

void dark_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    dark_message_create(&dark_msg);
    RCSOFTCHECK(rcl_publish(&dark_publisher, &dark_msg, NULL));
  }
}

void encoder_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    encoder_message_create(&encoders_msg);
    RCSOFTCHECK(rcl_publish(&enc_publisher, &encoders_msg, NULL));
  }
}

void fuel_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    fuel_message_create(&fuel_msg);
    RCSOFTCHECK(rcl_publish(&fuel_publisher, &fuel_msg, NULL));
  }
}

void imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    imu_message_create(&imu_msg);
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void presence_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    presence_message_create(&presence_msg);
    RCSOFTCHECK(rcl_publish(&presence_publisher, &presence_msg, NULL));
  }
}

void range_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    range_message_create(&range_msg);

    if (range_msg.range >= range_msg.min_range &&
        range_msg.range <= range_msg.max_range) {
      RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL));
    }
  }
}

void node_setup() {
  // Create a node identifier for this robot
  char node_name[50] = {0};
  int robotid = rand() % (NAME_UL + 1);
  sprintf(node_name, "mouse_%d", robotid);
  print("Mouse", node_name);

  // Set up the different sensors and topics
  dark_setup();
  fuel_setup();
  motor_setup(&encoders_msg);
  imu_setup(&imu_msg);
  presence_setup(&presence_msg);
  range_setup(&range_msg);

  allocator = rcl_get_default_allocator();

  // Create init options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, node_name, node_name, &support));

  // ---------------------------- CREATE PUBLISHERS
  RCCHECK(rclc_publisher_init_default(
      &dark_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "dark"));
  RCCHECK(rclc_timer_init_default(
      &dark_timer, &support, RCL_MS_TO_NS(DARK_RATE), dark_timer_callback));

  RCCHECK(rclc_publisher_init_default(
      &fuel_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "fuel"));
  RCCHECK(rclc_timer_init_default(
      &fuel_timer, &support, RCL_MS_TO_NS(FUEL_RATE), fuel_timer_callback));

  RCCHECK(rclc_publisher_init_default(
      &enc_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Encoders), "encoders"));
  RCCHECK(rclc_timer_init_default(&enc_timer, &support,
                                  RCL_MS_TO_NS(ENCODER_RATE),
                                  encoder_timer_callback));

  RCCHECK(rclc_publisher_init_default(
      &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu"));
  RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(IMU_RATE),
                                  imu_timer_callback));

  RCCHECK(rclc_publisher_init_default(
      &presence_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Presence), "presence"));
  RCCHECK(rclc_timer_init_default(&presence_timer, &support,
                                  RCL_MS_TO_NS(PRESENCE_RATE),
                                  presence_timer_callback));

  RCCHECK(rclc_publisher_init_default(
      &range_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range"));
  RCCHECK(rclc_timer_init_default(
      &range_timer, &support, RCL_MS_TO_NS(RANGE_RATE), range_timer_callback));

  // ---------------------------- CREATE SUBSCRIBERS
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // ---------------------------- CREATE EXECUTORS
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &dark_timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &fuel_timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &enc_timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &imu_timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &presence_timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &range_timer));

  executor_sub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &cmd_vel_subscriber,
                                         &cmd_vel_msg, &cmd_vel_sub_callback,
                                         ON_NEW_DATA));

  delay(1000);
}

void node_spin_task(void* pvParameters) {
  for (;;) {
    RCSOFTCHECK(
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(SPIN_RATE)));
    RCSOFTCHECK(
        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(SPIN_RATE)));
    vTaskDelay(SPIN_RATE / portTICK_PERIOD_MS);
  }
}