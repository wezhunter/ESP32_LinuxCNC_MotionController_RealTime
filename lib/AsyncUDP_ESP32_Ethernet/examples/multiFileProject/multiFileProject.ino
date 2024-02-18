/****************************************************************************************************************************
  multiFileProject.ino
  AsyncUDP_ESP32_Ethernet is a Async UDP library for the ESP32_Ethernet (ESP32S2/S3/C3 + LwIP W5500 / ENC28J60)

  Based on and modified from ESPAsyncUDP Library (https://github.com/me-no-dev/ESPAsyncUDP)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncUDP_ESP32_Ethernet
  Licensed under GPLv3 license
*****************************************************************************************************************************/

// To demo how to include files in multi-file Projects

#include "multiFileProject.h"

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "AsyncUDP_ESP32_Ethernet.h"

// Select the IP address according to your local network
IPAddress myIP(192, 168, 2, 232);
IPAddress myGW(192, 168, 2, 1);
IPAddress mySN(255, 255, 255, 0);

// Google DNS Server IP
IPAddress myDNS(8, 8, 8, 8);

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

  Serial.print(F("\nStart multiFileProject on "));
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

  Serial.print("You're OK now");
}

void loop()
{
  // put your main code here, to run repeatedly:
}
