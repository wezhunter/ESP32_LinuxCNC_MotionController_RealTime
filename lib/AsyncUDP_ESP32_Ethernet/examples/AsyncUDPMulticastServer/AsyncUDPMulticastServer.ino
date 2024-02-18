/****************************************************************************************************************************
  AsyncUDPMulticastServer.ino

  AsyncUDP_ESP32_Ethernet is a Async UDP library for the ESP32_Ethernet (ESP32S2/S3/C3 + LwIP W5500 / ENC28J60)

  Based on and modified from ESPAsyncUDP Library (https://github.com/me-no-dev/ESPAsyncUDP)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncUDP_ESP32_Ethernet
  Licensed under GPLv3 license
 *****************************************************************************************************************************/

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

#include <AsyncUDP_ESP32_Ethernet.h>

/////////////////////////////////////////////

// Enter a MAC address and IP address for your controller below.
#define NUMBER_OF_MAC      20

byte mac[][NUMBER_OF_MAC] =
{
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x02 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x04 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x06 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x07 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x08 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x09 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0A },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0B },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0C },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0D },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0E },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0F },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x10 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x11 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x12 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x13 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x14 },
};

// Select the IP address according to your local network
IPAddress myIP(192, 168, 2, 232);
IPAddress myGW(192, 168, 2, 1);
IPAddress mySN(255, 255, 255, 0);

// Google DNS Server IP
IPAddress myDNS(8, 8, 8, 8);

/////////////////////////////////////////////

AsyncUDP udp;

void parsePacket(AsyncUDPPacket packet)
{
  Serial.print("UDP Packet Type: ");
  Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
  Serial.print(", From: ");
  Serial.print(packet.remoteIP());
  Serial.print(":");
  Serial.print(packet.remotePort());
  Serial.print(", To: ");
  Serial.print(packet.localIP());
  Serial.print(":");
  Serial.print(packet.localPort());
  Serial.print(", Length: ");
  Serial.print(packet.length());
  Serial.print(", Data: ");
  Serial.write(packet.data(), packet.length());
  Serial.println();
  //reply to the client
  packet.printf("Got %u bytes of data", packet.length());
}

void initEthernet()
{
  UDP_LOGWARN(F("Default SPI pinout:"));
  UDP_LOGWARN1(F("SPI_HOST:"), ETH_SPI_HOST);
  UDP_LOGWARN1(F("MOSI:"), MOSI_GPIO);
  UDP_LOGWARN1(F("MISO:"), MISO_GPIO);
  UDP_LOGWARN1(F("SCK:"),  SCK_GPIO);
  UDP_LOGWARN1(F("CS:"),   CS_GPIO);
  UDP_LOGWARN1(F("INT:"),  INT_GPIO);
  UDP_LOGWARN1(F("SPI Clock (MHz):"), SPI_CLOCK_MHZ);
  UDP_LOGWARN(F("========================="));

  ///////////////////////////////////

  // To be called before ETH.begin()
  ESP32_Ethernet_onEvent();

  // start the ethernet connection and the server:
  // Use DHCP dynamic IP and random mac
  //bool begin(int MISO_GPIO, int MOSI_GPIO, int SCLK_GPIO, int CS_GPIO, int INT_GPIO, int SPI_CLOCK_MHZ,
  //           int SPI_HOST, uint8_t *W6100_Mac = W6100_Default_Mac);
  ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, ETH_SPI_HOST );
  //ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, ETH_SPI_HOST, mac[millis() % NUMBER_OF_MAC] );

  // Static IP, leave without this line to get IP via DHCP
  //bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = 0, IPAddress dns2 = 0);
  //ETH.config(myIP, myGW, mySN, myDNS);

  ESP32_Ethernet_waitForConnect();

  ///////////////////////////////////
}

////////////////////////////////////

void setup()
{
  Serial.begin(115200);

  while (!Serial && (millis() < 5000));

  delay(500);

  Serial.print(F("\nStart AsyncUDPMulticastServer on "));
  Serial.print(ARDUINO_BOARD);
  Serial.print(F(" with "));
  Serial.println(SHIELD_TYPE);

#if USING_W5500
  Serial.println(WEBSERVER_ESP32_W5500_VERSION);
#elif USING_W6100
  Serial.println(WEBSERVER_ESP32_W6100_VERSION);  
#else
  Serial.println(WEBSERVER_ESP32_ENC_VERSION);
#endif
  
  Serial.println(ASYNC_UDP_ESP32_ETHERNET_VERSION);

  Serial.setDebugOutput(true);

  ///////////////////////////////////

  initEthernet();

  ///////////////////////////////////

  // Client address
  Serial.print("Async_UDPClient started @ IP address: ");
  Serial.println(ETH.localIP());


  if (udp.listenMulticast(IPAddress(239, 1, 2, 3), 1234))
  {
    Serial.print("UDP Listening on IP: ");
    Serial.println(ETH.localIP());

    udp.onPacket([](AsyncUDPPacket packet)
    {
      parsePacket(packet);
    });

    //Send multicast
    udp.print("Hello!");
  }
}

void loop()
{
  delay(1000);
  //Send multicast
  udp.print("Anyone here?");
}
