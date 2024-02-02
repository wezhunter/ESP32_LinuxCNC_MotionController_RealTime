#pragma once
#ifdef ARDUINO_ESP32_MKS_DLC32

#include <Arduino.h>



extern "C" int  __digitalRead(uint8_t pin);
extern "C" void __pinMode(uint8_t pin, uint8_t mode);
extern "C" void __digitalWrite(uint8_t pin, uint8_t val);

String pinName(uint8_t pin);
#endif