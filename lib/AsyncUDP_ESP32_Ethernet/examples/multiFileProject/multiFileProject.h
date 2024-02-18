/****************************************************************************************************************************
  multiFileProject.h
  AsyncUDP_ESP32_Ethernet is a Async UDP library for the ESP32_Ethernet (ESP32S2/S3/C3 + LwIP W5500 / ENC28J60)

  Based on and modified from ESPAsyncUDP Library (https://github.com/me-no-dev/ESPAsyncUDP)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncUDP_ESP32_Ethernet
  Licensed under GPLv3 license
*****************************************************************************************************************************/

// To demo how to include files in multi-file Projects

#pragma once

#if !( defined(ESP32) )
  #error This code is designed for (ESP32 + LwIP W5500, W6100 or ENC28J60) to run on ESP32 platform! Please check your Tools->Board setting.
#endif

#include <Arduino.h>

#define USING_W5500           false
#define USING_W6100           true
#define USING_ENC28J60        false

#if !USING_W5500 && !USING_W6100 && !USING_ENC28J60
  #undef USING_W5500
  #define USING_W5500           true
#endif

#define ASYNC_UDP_ESP32_ETHERNET_DEBUG_PORT      Serial

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_      2

//////////////////////////////////////////////////////////

#if USING_W5500

  #define ESP32_Ethernet_onEvent            ESP32_W5500_onEvent
  #define ESP32_Ethernet_waitForConnect     ESP32_W5500_waitForConnect

  // For ESP32_S3
  // Optional values to override default settings
  // Don't change unless you know what you're doing
  //#define ETH_SPI_HOST        SPI3_HOST
  //#define SPI_CLOCK_MHZ       25
  
  // Must connect INT to GPIOxx or not working
  //#define INT_GPIO            4
  
  //#define MISO_GPIO           13
  //#define MOSI_GPIO           11
  //#define SCK_GPIO            12
  //#define CS_GPIO             10

  // For ESP32_C3
  // Optional values to override default settings
  // Don't change unless you know what you're doing
  //#define ETH_SPI_HOST        SPI2_HOST
  //#define SPI_CLOCK_MHZ       25
  
  // Must connect INT to GPIOxx or not working
  //#define INT_GPIO            10
  
  //#define MISO_GPIO           5
  //#define MOSI_GPIO           6
  //#define SCK_GPIO            4
  //#define CS_GPIO             7

  //////////////////////////////////////////////////////////

#elif USING_W6100

  #define ESP32_Ethernet_onEvent            ESP32_W6100_onEvent
  #define ESP32_Ethernet_waitForConnect     ESP32_W6100_waitForConnect

  // For ESP32_S3
  // Optional values to override default settings
  // Don't change unless you know what you're doing
  //#define ETH_SPI_HOST        SPI3_HOST
  //#define SPI_CLOCK_MHZ       25
  
  // Must connect INT to GPIOxx or not working
  //#define INT_GPIO            4
  
  //#define MISO_GPIO           13
  //#define MOSI_GPIO           11
  //#define SCK_GPIO            12
  //#define CS_GPIO             10

  // For ESP32_C3
  // Optional values to override default settings
  // Don't change unless you know what you're doing
  //#define ETH_SPI_HOST        SPI2_HOST
  //#define SPI_CLOCK_MHZ       25
  
  // Must connect INT to GPIOxx or not working
  //#define INT_GPIO            10
  
  //#define MISO_GPIO           5
  //#define MOSI_GPIO           6
  //#define SCK_GPIO            4
  //#define CS_GPIO             7

  //////////////////////////////////////////////////////////
  
#else   // #if USING_W5500

  //////////////////////////////////////////////////////////

  // For ENC28J60

  #define ESP32_Ethernet_onEvent            ESP32_ENC_onEvent
  #define ESP32_Ethernet_waitForConnect     ESP32_ENC_waitForConnect
  #define ETH_SPI_HOST                      SPI_HOST

  // Optional values to override default settings
  // Don't change unless you know what you're doing
  //#define SPI_HOST            SPI2_HOST
  //#define SPI_CLOCK_MHZ       8
  
  // Must connect INT to GPIOxx or not working
  //#define INT_GPIO            4
  
  //#define MISO_GPIO           13
  //#define MOSI_GPIO           11
  //#define SCK_GPIO            12
  //#define CS_GPIO             10

  //////////////////////////////////////////////////////////

#endif    // #if USING_W5500

//////////////////////////////////////////////////////////

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "AsyncUDP_ESP32_Ethernet.hpp"
