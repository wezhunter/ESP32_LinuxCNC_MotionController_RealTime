/****************************************************************************************************************************
  AsyncUDP_ESP32_Ethernet_Debug.h

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

#ifndef ASYNC_UDP_ESP32_ETHERNET_DEBUG_H
#define ASYNC_UDP_ESP32_ETHERNET_DEBUG_H

////////////////////////////////////////////////

#ifdef ASYNC_UDP_ESP32_ETHERNET_DEBUG_PORT
  #define ASYNC_UDP_DBG_PORT      ASYNC_UDP_ESP32_ETHERNET_DEBUG_PORT
#else
  #define ASYNC_UDP_DBG_PORT      Serial
#endif

////////////////////////////////////////////////

// Change _ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#ifndef _ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_
  #define _ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_       0
#endif

///////////////////////////////////////

const char ASYNC_UDP_MARK[]  = "[UDP] ";
const char ASYNC_UDP_SP[]    = " ";

#define ASYNC_UDP_PRINT          ASYNC_UDP_DBG_PORT.print
#define ASYNC_UDP_PRINTLN        ASYNC_UDP_DBG_PORT.println
#define ASYNC_UDP_FLUSH          ASYNC_UDP_DBG_PORT.flush

#define ASYNC_UDP_PRINT_MARK     ASYNC_UDP_PRINT(ASYNC_UDP_MARK)
#define ASYNC_UDP_PRINT_SP       ASYNC_UDP_PRINT(ASYNC_UDP_SP)

///////////////////////////////////////

#define ASYNC_UDP_LOG(x)         { ASYNC_UDP_PRINTLN(x); }
#define ASYNC_UDP_LOG0(x)        { ASYNC_UDP_PRINT(x); }
#define ASYNC_UDP_LOG1(x,y)      { ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINTLN(y); }
#define ASYNC_UDP_LOG2(x,y,z)    { ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINTLN(z); }
#define ASYNC_UDP_LOG3(x,y,z,w)  { ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT(z); ASYNC_UDP_PRINTLN(w); }

///////////////////////////////////////

#define UDP_LOGERROR(x)         if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>0) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINTLN(x); }
#define UDP_LOGERROR0(x)        if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>0) { ASYNC_UDP_PRINT(x); }
#define UDP_LOGERROR1(x,y)      if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>0) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(y); }
#define UDP_LOGERROR2(x,y,z)    if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>0) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(z); }
#define UDP_LOGERROR3(x,y,z,w)  if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>0) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(z); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(w); }

///////////////////////////////////////

#define UDP_LOGWARN(x)          if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>1) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINTLN(x); }
#define UDP_LOGWARN0(x)         if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>1) { ASYNC_UDP_PRINT(x); }
#define UDP_LOGWARN1(x,y)       if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>1) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(y); }
#define UDP_LOGWARN2(x,y,z)     if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>1) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(z); }
#define UDP_LOGWARN3(x,y,z,w)   if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>1) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(z); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(w); }

///////////////////////////////////////

#define UDP_LOGINFO(x)          if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>2) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINTLN(x); }
#define UDP_LOGINFO0(x)         if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>2) { ASYNC_UDP_PRINT(x); }
#define UDP_LOGINFO1(x,y)       if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>2) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(y); }
#define UDP_LOGINFO2(x,y,z)     if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>2) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(z); }
#define UDP_LOGINFO3(x,y,z,w)   if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>2) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(z); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(w); }

///////////////////////////////////////

#define UDP_LOGDEBUG(x)         if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>3) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINTLN(x); }
#define UDP_LOGDEBUG0(x)        if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>3) { ASYNC_UDP_PRINT(x); }
#define UDP_LOGDEBUG1(x,y)      if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>3) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(y); }
#define UDP_LOGDEBUG2(x,y,z)    if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>3) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(z); }
#define UDP_LOGDEBUG3(x,y,z,w)  if(_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_>3) { ASYNC_UDP_PRINT_MARK; ASYNC_UDP_PRINT(x); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(y); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINT(z); ASYNC_UDP_PRINT_SP; ASYNC_UDP_PRINTLN(w); }

///////////////////////////////////////

#endif    //ASYNC_UDP_ESP32_ETHERNET_DEBUG_H
