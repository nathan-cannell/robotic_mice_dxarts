/**
 * @file main.cpp
 * @author Ethan Widger
 * @brief The main launch file for the dxArts Robotic Mouse Project.
 * @version 0.1
 * @date 2024-02-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <main.h>

/**
 * @brief Main initialization function.
 *
 */
void setup() {
  /// @brief Connection information for WiFi
  IPAddress agent_ip(192, 168, 4, 1);
  size_t agent_port = 8888;
  char ssid[] = "dxRobots";
  char psk[] = "letMe$500in";

  // Random Number
  srand(time(NULL));

  // Set up I2C
  Wire.begin();

  // Set up feedback for LED
  pinMode(OBLED_PIN, OUTPUT);
  digitalWrite(OBLED_PIN, HIGH);
  oled_setup();
  print("WiFi", "Connecting...");

// Connect to Serial Transport
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
// Connect to WiFi Transport
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  digitalWrite(OBLED_PIN, LOW);  // Connected to WiFi turns off LED.

  char buffer[127];
  sprintf(buffer, "Connected as\n%s", WiFi.localIP().toString());
  print("WiFi", buffer);
#else
#error "No transport defined. Check platformio.ini file."
#endif

  delay(2000);
  clear();
  delay(1000);

  node_setup();

  xTaskCreate(node_spin_task, "Node Spin", 5000, NULL, 2, NULL);
  xTaskCreate(motor_task, "Motors", 1000, NULL, 1, NULL);
}

/**
 * @brief Main loop function.
 *
 */
void loop() {}