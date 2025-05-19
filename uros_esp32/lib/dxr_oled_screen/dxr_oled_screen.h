#ifndef _OLED_SCREEN_H_
#define _OLED_SCREEN_H_
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define OBLED_PIN 2

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

void oled_setup();
void clear();
void print(const char* title, const char* status);
void print_error(const char* file, const int line, const int error_num);

#endif