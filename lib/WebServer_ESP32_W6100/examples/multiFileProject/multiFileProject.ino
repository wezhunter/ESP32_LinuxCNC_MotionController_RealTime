/****************************************************************************************************************************
  multiFileProject.ino
  For Ethernet shields using ESP32_W6100 (ESP32 + W6100)

  WebServer_ESP32_W6100 is a library for the ESP32 with Ethernet W6100 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_W6100
  Licensed under GPLv3 license
*****************************************************************************************************************************/

// To demo how to include files in multi-file Projects

#include "multiFileProject.h"

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "WebServer_ESP32_W6100.h"

void setup()
{
  Serial.begin(115200);

  while (!Serial);

  delay(500);

  Serial.println("\nStart multiFileProject");
  Serial.println(WEBSERVER_ESP32_W6100_VERSION);


  Serial.print("You're OK now");
}

void loop()
{
  // put your main code here, to run repeatedly:
}
