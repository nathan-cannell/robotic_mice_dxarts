/**
 * @file Motor_Controller.h
 * @author Ethan Widger
 * @brief Header file for the class to control and manage the speed of N20
 * Motors using TB6612FNG H Bridge Driver
 * @version 0.1
 * @date 2024-03-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <Arduino.h>
#include <AutoPID.h>
#include <ESP32Encoder.h>
#include <driver/ledc.h>

#define FREQ 5000
#define RES 12
#define PWM_MOTOR_MIN 1100
#define PWM_MOTOR_MAX pow(2, RES) - 1

class Motor {
 public:
  Motor(int in1pin,
        int in2pin,
        int pwmpin,
        int stbypin,
        int encapin,
        int encbpin,
        int pwm_channel,
        double gear_ratio,
        double enc_per_rev);

  double kp = 1.0f, kd = 0.0f, ki = 0.0f;
  bool motor_state{false};
  int64_t encoder_pos{0};

  void init(double kp, double ki, double kd);
  void set_speed(double speed);
  double get_speed();
  void standby();
  void turn_on();
  void turn_off();
  void loop();

 private:
  double gear_ratio{1}, enc_per_rev{1};
  long prev_us;
  double target_vel{0}, curr_vel{0}, curr_vel_raw{0};
  double pwm_val{0};
  uint8_t encAp, encBp, in1p, in2p, pwmp, offsetp, standbyp, pwmc;

  ESP32Encoder enc;
  AutoPID speed_pid{&curr_vel,     &target_vel, &pwm_val, PWM_MOTOR_MIN,
                    PWM_MOTOR_MAX, kp,          ki,       kd};

  void set_motor(int dir, int pwmval);
  void stop();
  void coast();
  void read_encoder();
};