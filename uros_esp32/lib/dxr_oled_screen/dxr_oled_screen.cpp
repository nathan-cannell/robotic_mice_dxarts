#include "dxr_oled_screen.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void oled_setup() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Error loop
    while (1) {
      digitalWrite(OBLED_PIN, !digitalRead(OBLED_PIN));
      delay(250);
    }
  }
  delay(1000);
  clear();
}

void clear() {
  display.clearDisplay();
  display.display();
}

void print(const char* title, const char* status) {
  char buffer1[11];
  strncpy(buffer1, title, 10);

  char buffer2[127];
  strncpy(buffer2, status, 126);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 2);

  // Display static text
  display.println(buffer1);

  // Print the status
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println(status);
  display.display();
}

void print_error(const char* file, const int line, const int error_num) {
  char buffer1[11];
  char buffer2[127];
  int size;
  size = sprintf(buffer1, "ERROR:%d", error_num);
  // Print the file name
  size = sprintf(buffer2, "File: %s\nLine: %d", file, line);
  print(buffer1, buffer2);
}
