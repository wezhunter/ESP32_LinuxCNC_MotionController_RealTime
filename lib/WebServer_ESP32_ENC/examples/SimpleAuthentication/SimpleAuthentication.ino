/****************************************************************************************************************************
  SimpleAuthentication.ino - Dead simple web-server for Ethernet shields

  For Ethernet shields using ESP32_ENC (ESP32 + ENC28J60)

  WebServer_ESP32_ENC is a library for the ESP32 with Ethernet ENC28J60 to run WebServer

  Based on and modified from ESP8266 https://github.com/esp8266/Arduino/releases
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_ENC
  Licensed under GPLv3 license
 *****************************************************************************************************************************/

#if !( defined(ESP32) )
  #error This code is designed for (ESP32 + ENC28J60) to run on ESP32 platform! Please check your Tools->Board setting.
#endif

#define DEBUG_ETHERNET_WEBSERVER_PORT       Serial

// Debug Level from 0 to 4
#define _ETHERNET_WEBSERVER_LOGLEVEL_       3

//////////////////////////////////////////////////////////

// Optional values to override default settings
//#define ETH_SPI_HOST        SPI3_HOST
//#define SPI_CLOCK_MHZ       20

// Must connect INT to GPIOxx or not working
//#define INT_GPIO            4

//#define MISO_GPIO           19
//#define MOSI_GPIO           23
//#define SCK_GPIO            18
//#define CS_GPIO             5

//////////////////////////////////////////////////////////

#include <WebServer_ESP32_ENC.h>

WebServer server(80);

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

//Check if header is present and correct
bool is_authenticated()
{
  Serial.println(F("Enter is_authenticated"));

  if (server.hasHeader(F("Cookie")))
  {
    Serial.print(F("Found cookie: "));
    String cookie = server.header(F("Cookie"));
    Serial.println(cookie);

    if (cookie.indexOf(F("ESPSESSIONID=1")) != -1)
    {
      Serial.println(F("Authentication Successful"));
      return true;
    }
  }

  Serial.println(F("Authentication Failed"));
  return false;
}

//login page, also called for disconnect
void handleLogin()
{
  String msg;

  if (server.hasHeader(F("Cookie")))
  {
    Serial.print(F("Found cookie: "));
    String cookie = server.header(F("Cookie"));
    Serial.println(cookie);
  }

  if (server.hasArg(F("DISCONNECT")))
  {
    Serial.println(F("Disconnection"));
    server.sendHeader(F("Location"), F("/login"));
    server.sendHeader(F("Cache-Control"), F("no-cache"));
    server.sendHeader(F("Set-Cookie"), F("ESPSESSIONID=0"));
    server.send(301);
    return;
  }

  if (server.hasArg(F("USERNAME")) && server.hasArg(F("PASSWORD")))
  {
    if (server.arg(F("USERNAME")) == F("admin") &&  server.arg(F("PASSWORD")) == F("password"))
    {
      server.sendHeader(F("Location"), F("/"));
      server.sendHeader(F("Cache-Control"), F("no-cache"));
      server.sendHeader(F("Set-Cookie"), F("ESPSESSIONID=1"));
      server.send(301);
      Serial.println(F("Log in Successful"));
      return;
    }

    msg = F("Wrong username/password! try again.");
    Serial.println(F("Log in Failed"));
  }

  String content = F("<html><body><form action='/login' method='POST'>To log in, please use : admin/password<br>");
  content += F("User:<input type='text' name='USERNAME' placeholder='user name'><br>");
  content += F("Password:<input type='password' name='PASSWORD' placeholder='password'><br>");
  content += F("<input type='submit' name='SUBMIT' value='Submit'></form>");
  content += msg;
  content += F("<br>");
  content += F("You also can go <a href='/inline'>here</a></body></html>");
  server.send(200, F("text/html"), content);
}

//root page can be accessed only if authentication is ok
void handleRoot()
{
  String header;

  Serial.println(F("Enter handleRoot"));

  if (!is_authenticated())
  {
    server.sendHeader(F("Location"), F("/login"));
    server.sendHeader(F("Cache-Control"), F("no-cache"));
    server.send(301);
    return;
  }

  String content = F("<html><body><H2>Hello, you're connected to WebServer_ESP32_ENC running on ");

  content += String(ARDUINO_BOARD);
  content += F("!</H2><br>");

  if (server.hasHeader(F("User-Agent")))
  {
    content += F("the user agent used is : ");
    content += server.header(F("User-Agent"));
    content += F("<br><br>");
  }

  content += F("You can access this page until you <a href=\"/login?DISCONNECT=YES\">disconnect</a></body></html>");
  server.send(200, F("text/html"), content);
}

//no need authentication
void handleNotFound()
{
  String message = F("File Not Found\n\n");

  message += F("URI: ");
  message += server.uri();
  message += F("\nMethod: ");
  message += (server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\nArguments: ");
  message += server.args();
  message += F("\n");

  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, F("text/plain"), message);
}

void setup()
{
  Serial.begin(115200);

  while (!Serial && (millis() < 5000));

  Serial.print(F("\nStart SimpleAuthentication on "));
  Serial.print(ARDUINO_BOARD);
  Serial.print(F(" with "));
  Serial.println(SHIELD_TYPE);
  Serial.println(WEBSERVER_ESP32_ENC_VERSION);

  ET_LOGWARN(F("Default SPI pinout:"));
  ET_LOGWARN1(F("MOSI:"), MOSI_GPIO);
  ET_LOGWARN1(F("MISO:"), MISO_GPIO);
  ET_LOGWARN1(F("SCK:"),  SCK_GPIO);
  ET_LOGWARN1(F("CS:"),   CS_GPIO);
  ET_LOGWARN1(F("INT:"),  INT_GPIO);
  ET_LOGWARN1(F("SPI Clock (MHz):"), SPI_CLOCK_MHZ);
  ET_LOGWARN(F("========================="));

  ///////////////////////////////////

  // To be called before ETH.begin()
  ESP32_ENC_onEvent();

  // start the ethernet connection and the server:
  // Use DHCP dynamic IP and random mac
  //bool begin(int MISO_GPIO, int MOSI_GPIO, int SCLK_GPIO, int CS_GPIO, int INT_GPIO, int SPI_CLOCK_MHZ,
  //           int SPI_HOST, uint8_t *W6100_Mac = W6100_Default_Mac);
  ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, ETH_SPI_HOST );
  //ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, ETH_SPI_HOST, mac[millis() % NUMBER_OF_MAC] );

  // Static IP, leave without this line to get IP via DHCP
  //bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = 0, IPAddress dns2 = 0);
  ETH.config(myIP, myGW, mySN, myDNS);

  ESP32_ENC_waitForConnect();

  ///////////////////////////////////

  server.on(F("/"), handleRoot);

  server.on(F("/login"), handleLogin);

  server.on(F("/inline"), []()
  {
    server.send(200, F("text/plain"), F("This works without need of authentication"));
  });

  server.onNotFound(handleNotFound);

  //here the list of headers to be recorded
  const char * headerkeys[] = {"User-Agent", "Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys) / sizeof(char*);

  //ask server to track these headers
  server.collectHeaders(headerkeys, headerkeyssize);
  server.begin();

  Serial.print(F("HTTP SimpleAuthentication is @ IP : "));
  Serial.println(ETH.localIP());
}

void loop()
{
  server.handleClient();
}
