/**
 * @file Motor_Controller.cpp
 * @author Ethan Widger
 * @brief A class to control and manage the speed of N20 Motors using TB6612FNG
 * H Bridge Driver
 * @version 0.1
 * @date 2024-03-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "Motor_Controller.h"

Motor::Motor(int in1pin,
             int in2pin,
             int pwmpin,
             int stbypin,
             int encapin,
             int encbpin,
             int pwm_channel,
             double gear_ratio,
             double enc_per_rev) {
  this->in1p = in1pin;
  this->in2p = in2pin;
  this->pwmp = pwmpin;
  this->standbyp = stbypin;
  this->encAp = encapin;
  this->encBp = encbpin;
  this->pwmc = pwm_channel;
  this->gear_ratio = gear_ratio;
  this->enc_per_rev = enc_per_rev;
}

void Motor::init(double kp, double ki, double kd) {
  // Set up PID values
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

  // Set up motor pins
  pinMode(in1p, OUTPUT);
  pinMode(in2p, OUTPUT);
  pinMode(pwmp, OUTPUT);
  pinMode(standbyp, OUTPUT);

  // Create motor pwm generator
  ledcSetup(pwmc, FREQ, RES);
  ledcAttachPin(pwmp, pwmc);

  // Activate motor
  motor_state = true;

  // Set up encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc.attachFullQuad(encAp, encBp);

  // Set up PID
  speed_pid.setGains(kp, ki, kd);
  speed_pid.setTimeStep(20);
}

void Motor::set_motor(int dir, int set_pwm) {
  // Constrain PWM within operating abilities
  if (set_pwm != 0)
    constrain(set_pwm, PWM_MOTOR_MIN, PWM_MOTOR_MAX);

  digitalWrite(standbyp, HIGH);
  if (dir > 0 && motor_state == true) {
    // Move Forward
    digitalWrite(in1p, HIGH);
    digitalWrite(in2p, LOW);
    ledcWrite(pwmc, set_pwm);
  } else if (dir < 0 && motor_state == true) {
    // Move Reverse
    digitalWrite(in1p, LOW);
    digitalWrite(in2p, HIGH);
    ledcWrite(pwmc, set_pwm);
  } else {
    // Brake
    coast();
  }
}

void Motor::stop() {
  digitalWrite(in1p, LOW);
  digitalWrite(in2p, LOW);
  ledcWrite(pwmc, PWM_MOTOR_MAX);
}

void Motor::coast() {
  digitalWrite(in1p, HIGH);
  digitalWrite(in2p, HIGH);
  ledcWrite(pwmc, 0);
}

void Motor::read_encoder() {
  long curr_us = micros();

  double delta_s = ((double)(curr_us - prev_us)) / 1.0e6;
  int32_t m1c = (int32_t)enc.getCount();
  enc.clearCount();

  encoder_pos += m1c;
  curr_vel_raw = m1c / (delta_s * gear_ratio * enc_per_rev);
  curr_vel = curr_vel_raw;

  prev_us = curr_us;
}

void Motor::set_speed(double speed) {
  this->target_vel = speed;
}

double Motor::get_speed() {
  return this->curr_vel;
}

void Motor::standby() {
  digitalWrite(standbyp, LOW);
}

void Motor::turn_on() {
  this->motor_state = true;
}

void Motor::turn_off() {
  this->motor_state = false;
  stop();
}

void Motor::loop() {
  this->read_encoder();

  speed_pid.run();

  if (target_vel == 0) {
    speed_pid.reset();
    pwm_val = 0;
  }

  int dir = 1;
  if (target_vel < 0)
    dir = -1;

  this->set_motor(dir, this->pwm_val);
}
