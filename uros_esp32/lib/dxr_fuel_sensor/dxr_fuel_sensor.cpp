#include "dxr_fuel_sensor.h"

Adafruit_MAX17048 fuel_sensor;

void fuel_setup() {
  if (!fuel_sensor.begin()) {
    // Error no battery
    print_error(__FILENAME__, __LINE__, 44);
    while (1) {
      // Using WiFi transport, show errors with on board LED
      digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
      delay(400);
      digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
      delay(2000);
    }
  }
}

void fuel_message_create(std_msgs__msg__Float32* msg) {
  float voltage = fuel_sensor.cellVoltage();
  while (isnan(voltage)) {
    print_error(__FILENAME__, __LINE__, 88);
  }
  msg->data = fuel_sensor.cellPercent();
}