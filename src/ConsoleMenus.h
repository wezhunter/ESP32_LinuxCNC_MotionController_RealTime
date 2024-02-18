#pragma once

#ifndef CONSOLEMENUS_H
#define CONSOLEMENUS_H

#include <Arduino.h>
//#include <ESP32Console.h>
//#include "ESP32Console/Helpers/PWDHelpers.h"
//#include "FastAccelStepper.h"
#include "Types.hpp"
//#include "ConfigManager.h"
//#include "ControllerMode.h"
//#include "ClientMode.h"

inline static char prompt_text[60] = "\e[1;32mESP32LCNC_Unknown >\e[0m ";
inline static const char controller_mode_name[22] = "ESP32LCNC_Controller";
inline static const char client_mode_name[18] = "ESP32LCNC_Client";
inline static const char unknown_mode_name[22] = "ESP32LCNC_UnknownMode";

typedef enum { UNDEF = 0, RED, ORANGE, YELLOW, GREEN,  BLUE, PURPLE, WHITE } prompt_colour;


void extraSafeModeConsolePrompt();
void safeModeConsolePrompt();
void defaultConsolePrompt();
void setConsolePrompt(prompt_colour colour, String new_status);
void registerDebuggingCmds();
void registerSafeModeConsoleCmds();
void registerConsoleCmds();


#endif