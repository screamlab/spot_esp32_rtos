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
const uint8_t servoMinAngles[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 180, 180, 120, 180, 90, 180, 180, 180, 180, 180, 180};
const uint8_t servoInitAngles[] = {10, 170, 80, 10, 80, 10, 0, 0, 0, 0, 0, 0};

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};