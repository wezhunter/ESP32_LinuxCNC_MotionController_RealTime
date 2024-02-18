/****************************************************************************************************************************
  AsyncUDP_ESP32_Ethernet.h

  AsyncUDP_ESP32_Ethernet is a Async UDP library for the ESP32_Ethernet (ESP32S2/S3/C3 + LwIP W5500 / ENC28J60)

  Based on and modified from ESPAsyncUDP Library (https://github.com/me-no-dev/ESPAsyncUDP)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncUDP_ESP32_Ethernet
  Licensed under GPLv3 license

  Version: 2.1.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  2.1.0   K Hoang      10/01/2023 Initial coding for ESP32 using LwIP W5500 / W6100 / ENC28J60
 *****************************************************************************************************************************/

#pragma once


#ifndef ASYNC_UDP_ESP32_ETHERNET_H
#define ASYNC_UDP_ESP32_ETHERNET_H

////////////////////////////////////////////////////

#if USING_W5500

  #include <WebServer_ESP32_W5500.h>     // https://github.com/khoih-prog/WebServer_ESP32_W5500
  
#elif USING_W6100

  #include <WebServer_ESP32_W6100.h>     // https://github.com/khoih-prog/WebServer_ESP32_W6100
    
#elif USING_ENC28J60

  #include <WebServer_ESP32_ENC.h>       // https://github.com/khoih-prog/WebServer_ESP32_ENC
  
#else

  #include <WebServer_ESP32_W5500.h>     // https://github.com/khoih-prog/WebServer_ESP32_W5500
  
#endif

////////////////////////////////////////////////

#include "AsyncUDP_ESP32_Ethernet.hpp"
#include "AsyncUDP_ESP32_Ethernet_Impl.h"

////////////////////////////////////////////////

#endif    //ASYNC_UDP_ESP32_ETHERNET_H
