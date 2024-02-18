#pragma once

#include <Arduino.h>
#include "I2SOut.h"
#include "Config.h"
#include "Types.hpp"

extern "C" int  __digitalRead(uint8_t pin);
extern "C" void __pinMode(uint8_t pin, uint8_t mode);
extern "C" void __digitalWrite(uint8_t pin, uint8_t val);

String pinName(uint8_t pin);
