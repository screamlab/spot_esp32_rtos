#pragma once
#include <stdint.h>

/*
 * https://circuits4you.com
 * ESP32 LED Blink Example
 * Board ESP23 DEVKIT V1
 *
 * ON Board LED GPIO 2
 * Ref: https://circuits4you.com/2018/02/02/esp32-led-blink-example/
 */
#define ESP32_LED 2
#define UPDATE_ARM_DELAY 0.01
const float ARM_MOVEMENT_STEP = 1.0;
const size_t NUM_OF_SERVOS = 12;
const uint8_t servoMinAngles[] = {80, 0, 30, 80, 30, 0, 80, 30, 0, 80, 0, 30};
const uint8_t servoMaxAngles[] = {100, 150, 180, 100, 180, 150, 100, 180, 150, 100, 150, 180};
const uint8_t servoInitAngles[] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};