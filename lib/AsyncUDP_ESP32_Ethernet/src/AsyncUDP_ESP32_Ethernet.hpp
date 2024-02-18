/****************************************************************************************************************************
  AsyncUDP_ESP32_Ethernet.hpp
  
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

#ifndef ASYNC_UDP_ESP32_ETHERNET_HPP
#define ASYNC_UDP_ESP32_ETHERNET_HPP

////////////////////////////////////////////////////

#if ( ARDUINO_ESP32S2_DEV || ARDUINO_FEATHERS2 || ARDUINO_ESP32S2_THING_PLUS || ARDUINO_MICROS2 || \
        ARDUINO_METRO_ESP32S2 || ARDUINO_MAGTAG29_ESP32S2 || ARDUINO_FUNHOUSE_ESP32S2 || \
        ARDUINO_ADAFRUIT_FEATHER_ESP32S2_NOPSRAM )
  #error ESP32_S2 not supported. Use AsyncUDP_ESP32_SC_Ethernet library

////////////////////////////////////////
 
#elif ( ARDUINO_ESP32C3_DEV )
  #error ESP32_C3 not supported. Use AsyncUDP_ESP32_SC_Ethernet library

////////////////////////////////////////
    
#elif ( defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32_S3_BOX) || defined(ARDUINO_TINYS3) || \
        defined(ARDUINO_PROS3) || defined(ARDUINO_FEATHERS3) )
  #error ESP32_S3 not supported. Use AsyncUDP_ESP32_SC_Ethernet library

////////////////////////////////////////

#elif ESP32

  #if (_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ > 3)
    #warning Using ESP32 architecture for AsyncUDP_ESP32_Ethernet
  #endif

//////////////////////////////////////// 
  
#else
	#error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

/////////////////////////////////////////////////

#define BOARD_NAME      ARDUINO_BOARD

/////////////////////////////////////////////////

#if ( ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2) ) && ( ARDUINO_ESP32_GIT_VER != 0x46d5afb1 ) )

	#if (_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ > 3)
		#warning Using code for ESP32 core v2.0.0+ in AsyncUDP_ESP32_Ethernet.h
	#endif

	#define ASYNC_UDP_ESP32_ETHERNET_VERSION      "AsyncUDP_ESP32_Ethernet v2.1.0 for core v2.0.0+"

	extern "C"
	{
		#include "lwip/ip_addr.h"
		#include "freertos/queue.h"
		#include "freertos/semphr.h"
	}

#else

	#if (_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ > 2)
		#warning Using code for ESP32 core v1.0.6- in AsyncUDP_ESP32_Ethernet.h
	#endif

	#define ASYNC_UDP_ESP32_ETHERNET_VERSION      "AsyncUDP_ESP32_Ethernet v2.1.0 for core v1.0.6-"

	extern "C"
	{
		#include "lwip/ip_addr.h"
		#include <tcpip_adapter.h>
		#include "freertos/queue.h"
		#include "freertos/semphr.h"
	}
#endif

////////////////////////////////////////////////

#include "IPAddress.h"
#include "IPv6Address.h"
#include "Print.h"
#include <functional>

////////////////////////////////////////////////

#include "AsyncUDP_ESP32_Ethernet_Debug.h"

////////////////////////////////////////////////////

#if USING_W5500
	#if (_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ > 3)    
    #warning USING_W5500
  #endif
  
	#include <WebServer_ESP32_W5500.hpp>     // https://github.com/khoih-prog/WebServer_ESP32_W5500
	
#elif USING_W6100
	#if (_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ > 3)    
    #warning USING_W6100
  #endif
  
	#include <WebServer_ESP32_W6100.hpp>     // https://github.com/khoih-prog/WebServer_ESP32_W6100
		
#elif USING_ENC28J60
	#if (_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ > 3)    
    #warning USING_ENC28J60
  #endif
  
	#include <WebServer_ESP32_ENC.hpp>     	// https://github.com/khoih-prog/WebServer_ESP32_ENC
	
#else
	#if (_ASYNC_UDP_ESP32_ETHERNET_LOGLEVEL_ > 3)    
    #warning Default to USING_W5500
  #endif
  
	#include <WebServer_ESP32_W5500.hpp>     // https://github.com/khoih-prog/WebServer_ESP32_W5500
#endif	
	
////////////////////////////////////////////////

class AsyncUDP;
class AsyncUDPPacket;
class AsyncUDPMessage;
struct udp_pcb;
struct pbuf;
struct netif;

typedef std::function<void(AsyncUDPPacket& packet)> AuPacketHandlerFunction;
typedef std::function<void(void * arg, AsyncUDPPacket& packet)> AuPacketHandlerFunctionWithArg;

////////////////////////////////////////////////

class AsyncUDPMessage : public Print
{
  protected:
    uint8_t * _buffer;
    size_t    _index;
    size_t    _size;
    
  public:
    AsyncUDPMessage(size_t size = CONFIG_TCP_MSS);
    virtual ~AsyncUDPMessage();
    
    size_t    write(const uint8_t *data, size_t len);
    size_t    write(uint8_t data);
    size_t    space();
    uint8_t * data();
    size_t    length();
    void      flush();
    
    operator bool()
    {
      return _buffer != NULL;
    }
};

////////////////////////////////////////////////

class AsyncUDPPacket : public Stream
{
  protected:
  
    AsyncUDP *          _udp;
    pbuf *              _pb;
    tcpip_adapter_if_t  _if;
    ip_addr_t           _localIp;
    uint16_t            _localPort;
    ip_addr_t           _remoteIp;
    uint16_t            _remotePort;
    uint8_t             _remoteMac[6];
    uint8_t *           _data;
    size_t              _len;
    size_t              _index;
    
  public:
    AsyncUDPPacket(AsyncUDPPacket &packet);
    AsyncUDPPacket(AsyncUDP *udp, pbuf *pb, const ip_addr_t *addr, uint16_t port, struct netif * netif);
    virtual ~AsyncUDPPacket();

    uint8_t * data();
    size_t    length();
    bool      isBroadcast();
    bool      isMulticast();
    bool      isIPv6();

    tcpip_adapter_if_t interface();

    IPAddress   localIP();
    IPv6Address localIPv6();
    uint16_t    localPort();
    IPAddress   remoteIP();
    IPv6Address remoteIPv6();
    uint16_t    remotePort();
    void        remoteMac(uint8_t * mac);

    size_t send(AsyncUDPMessage &message);

    int     available();
    size_t  read(uint8_t *data, size_t len);
    int     read();
    int     peek();
    void    flush();

    size_t  write(const uint8_t *data, size_t len);
    size_t  write(uint8_t data);
};

////////////////////////////////////////////////

class AsyncUDP : public Print
{
  protected:
  
    udp_pcb *_pcb;
    //xSemaphoreHandle _lock;
    bool _connected;
    esp_err_t _lastErr;
    AuPacketHandlerFunction _handler;

    bool _init();
    void _recv(udp_pcb *upcb, pbuf *pb, const ip_addr_t *addr, uint16_t port, struct netif * netif);

  public:
  
    AsyncUDP();
    virtual ~AsyncUDP();

    void onPacket(AuPacketHandlerFunctionWithArg cb, void * arg = NULL);
    void onPacket(AuPacketHandlerFunction cb);

    bool listen(const ip_addr_t *addr, uint16_t port);
    bool listen(const IPAddress addr, uint16_t port);
    bool listen(const IPv6Address addr, uint16_t port);
    bool listen(uint16_t port);

    bool listenMulticast(const ip_addr_t *addr, uint16_t port, uint8_t ttl = 1, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    bool listenMulticast(const IPAddress addr, uint16_t port, uint8_t ttl = 1, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    bool listenMulticast(const IPv6Address addr, uint16_t port, uint8_t ttl = 1, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);

    bool connect(const ip_addr_t *addr, uint16_t port);
    bool connect(const IPAddress addr, uint16_t port);
    bool connect(const IPv6Address addr, uint16_t port);

    void close();

    size_t writeTo(const uint8_t *data, size_t len, const ip_addr_t *addr, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t writeTo(const uint8_t *data, size_t len, const IPAddress addr, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t writeTo(const uint8_t *data, size_t len, const IPv6Address addr, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t write(const uint8_t *data, size_t len);
    size_t write(uint8_t data);

    size_t broadcastTo(uint8_t *data, size_t len, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t broadcastTo(const char * data, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t broadcast(uint8_t *data, size_t len);
    size_t broadcast(const char * data);

    size_t sendTo(AsyncUDPMessage &message, const ip_addr_t *addr, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t sendTo(AsyncUDPMessage &message, const IPAddress addr, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t sendTo(AsyncUDPMessage &message, const IPv6Address addr, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t send(AsyncUDPMessage &message);

    size_t broadcastTo(AsyncUDPMessage &message, uint16_t port, tcpip_adapter_if_t tcpip_if = TCPIP_ADAPTER_IF_MAX);
    size_t broadcast(AsyncUDPMessage &message);

    IPAddress   listenIP();
    IPv6Address listenIPv6();
    
    bool      connected();
    esp_err_t lastErr();
    operator  bool();

    static void _s_recv(void *arg, udp_pcb *upcb, pbuf *p, const ip_addr_t *addr, uint16_t port, struct netif * netif);
};

////////////////////////////////////////////////

#endif    //ASYNC_UDP_ESP32_ETHERNET_HPP
