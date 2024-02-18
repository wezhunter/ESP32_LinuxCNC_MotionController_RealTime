/****************************************************************************************************************************
  multiFileProject.ino
  For Ethernet shields using ESP32_W5500 (ESP32 + W5500)

  WebServer_ESP32_W5500 is a library for the ESP32 with Ethernet W5500 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_W5500
  Licensed under GPLv3 license
*****************************************************************************************************************************/

// To demo how to include files in multi-file Projects

#include "multiFileProject.h"

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "WebServer_ESP32_W5500.h"

void setup()
{
  Serial.begin(115200);

  while (!Serial);

  delay(500);

  Serial.println("\nStart multiFileProject");
  Serial.println(WEBSERVER_ESP32_W5500_VERSION);


  Serial.print("You're OK now");
}

void loop()
{
  // put your main code here, to run repeatedly:
}
