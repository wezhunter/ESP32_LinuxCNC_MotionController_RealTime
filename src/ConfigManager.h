#pragma once

#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <Arduino.h>
#include <ESP32Console.h>
#include "ESP32Console/Helpers/PWDHelpers.h"
#include "Types.hpp"
#include "Config.h"
#include <WiFi.h>


extern char **environ; // !! Important for console environment vars inherited from esp_console
inline uint8_t configNumSteppers = 0;
inline config_mode_t configMode = MODE_CONTROLLER; 
inline wifi_mode_t configWifiMode = WIFI_OFF;
inline String configWifiSSID = "";
inline String configWifiPwd = "";
inline uint8_t configWifiHide = 0;

inline uint8_t configSpiMosiPin = 23;
inline uint8_t configSpiMisoPin = 19;
inline uint8_t configSpiSckPin = 18;
inline uint8_t configSpiCsPin = 0;
inline uint8_t configSpiIntPin = 4;

inline board_type_t configBoardType = BOARD_TYPE_NONE;
inline String configBoardName = "";
inline uint16_t configVersion = 0;
inline bool doMotorConfig = false;

inline bool configEspNowEnabled = false;
inline String configRemotePeerAddress = "";

inline String getWifiConfigMode()
{
    switch(configWifiMode) {
        case WIFI_OFF:
            return "OFF";
        case WIFI_AP:
            return "AP";
        case WIFI_AP_STA:
            return "AP+STA";
        case WIFI_STA:
            return "STA";
        default:
            return "Unknown";
    }
}

inline String getBoardName(uint8_t board_type) {
     switch(board_type) {
        case BOARD_TYPE_NONE:
            return "None";
        case BOARD_TYPE_ESP32_WOKWI_SIMUL:
            return "Wowki Simulator ESP32";
        case BOARD_TYPE_ESP32_EVB:
            return "Olimex ESP32-EVB";
        case BOARD_TYPE_ESP32_POE:
            return "Olimex ESP32-POE";
        case BOARD_TYPE_ESP32_GATEWAY:
            return "Olimex ESP32-GATEWAY";
        case BOARD_TYPE_ESP32_WT32_ETH01:
            return "SeedStudio WT32-ETH01";
        case BOARD_TYPE_ESP32_MKSDLC32:
            return "MKS-DLC32";
        default:
            return "Default";
    }
}

inline uint8_t getIONumber(uint8_t io_num)
{
  switch(io_num) {
    case 0:
      return IO_00;
    case 1:
      return IO_01;
    case 2:
      return IO_02;
    case 3:
      return IO_03;
    case 4:
      return IO_04;
    case 5:
      return IO_05;
    case 6:
      return IO_06;
    case 7:
      return IO_07;
    default:
      return IO_00;
  }
}


void readNvsConfig(bool silent=true);
bool resetNvsConfig();
bool saveNvsConfig();
void setConfigEnvVars(bool silent=true);


#endif