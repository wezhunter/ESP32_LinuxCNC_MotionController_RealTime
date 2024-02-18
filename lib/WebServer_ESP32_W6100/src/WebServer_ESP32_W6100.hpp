/****************************************************************************************************************************
  WebServer_ESP32_W6100.h
  
  For Ethernet shields using ESP32_W6100 (ESP32 + W6100)

  WebServer_ESP32_W6100 is a library for the ESP32 with Ethernet W6100 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_W6100
  Licensed under GPLv3 license

  Version: 1.5.3

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.5.2   K Hoang      06/01/2022 Initial coding for ESP32_W6100 (ESP32 + W6100). Sync with WebServer_ESP32_W5500 v1.5.2
  1.5.3   K Hoang      11/01/2023 Using `SPI_DMA_CH_AUTO`
 *****************************************************************************************************************************/

#pragma once

#ifndef WEBSERVER_ESP32_W6100_HPP
#define WEBSERVER_ESP32_W6100_HPP

//////////////////////////////////////////////////////////////

//#define CONFIG_ETH_SPI_ETHERNET_W6100				true

//////////////////////////////////////////////////////////////

#include <WiFi.h>
#include <WebServer.h> // Introduce corresponding libraries

#include <hal/spi_types.h>

//////////////////////////////////////////////////////////////

#if !defined(ETH_SPI_HOST)
  #define ETH_SPI_HOST            SPI3_HOST
#endif

#if !defined(SPI_CLOCK_MHZ)
	// Using 25MHz for W6100, 14MHz for W5100
  #define SPI_CLOCK_MHZ       25
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
  #define SHIELD_TYPE         "ESP32_W6100"
#endif

//////////////////////////////////////////////////////////////

extern bool ESP32_W6100_eth_connected;

extern void ESP32_W6100_onEvent();

extern void ESP32_W6100_waitForConnect();

extern bool ESP32_W6100_isConnected();

extern void ESP32_W6100_event(WiFiEvent_t event);

//////////////////////////////////////////////////////////////

#endif    // WEBSERVER_ESP32_W6100_HPP
