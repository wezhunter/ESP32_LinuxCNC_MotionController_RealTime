/****************************************************************************************************************************
  WebServer_ESP32_ENC.h
  
  For Ethernet shields using ESP32_ENC (ESP32 + ENC28J60)

  WebServer_ESP32_ENC is a library for the ESP32 with Ethernet ENC28J60 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_ENC
  Licensed under GPLv3 license
  
  Version: 1.5.3

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.5.1   K Hoang      28/11/2022 Initial coding for ESP32_ENC (ESP32 + ENC28J60). Sync with WebServer_WT32_ETH01 v1.5.1
  1.5.3   K Hoang      11/01/2023 Using built-in ESP32 MAC. Increase default SPI clock to 20MHz from 8MHz
 *****************************************************************************************************************************/

#pragma once

#ifndef WEBSERVER_ESP32_ENC_HPP
#define WEBSERVER_ESP32_ENC_HPP

//////////////////////////////////////////////////////////////

#include <WiFi.h>
#include <WebServer.h> // Introduce corresponding libraries

#if !defined(SPI_HOST)
  #define SPI_HOST            SPI3_HOST
#endif

#if !defined(ETH_SPI_HOST)
  #define ETH_SPI_HOST        SPI3_HOST
#endif

#if !defined(SPI_CLOCK_MHZ)
  #define SPI_CLOCK_MHZ       20			//8
#endif

#if !defined(INT_GPIO)
  #define INT_GPIO            4
#endif

#if !defined(MISO_GPIO)
  #define MISO_GPIO           19
#endif

#if !defined(MOSI_GPIO)
  #define MOSI_GPIO           23
#endif

#if !defined(SCK_GPIO)
  #define SCK_GPIO            18
#endif

#if !defined(CS_GPIO)
  #define CS_GPIO             5
#endif

//////////////////////////////////////////////////////////////

#ifndef SHIELD_TYPE
  #define SHIELD_TYPE         "ESP32_ENC28J60"
#endif

//////////////////////////////////////////////////////////////

extern bool ESP32_ENC_eth_connected;

extern void ESP32_ENC_onEvent();

extern void ESP32_ENC_waitForConnect();

extern bool ESP32_ENC_isConnected();

extern void ESP32_ENC_event(WiFiEvent_t event);

//////////////////////////////////////////////////////////////

#endif    // WEBSERVER_ESP32_ENC_HPP
