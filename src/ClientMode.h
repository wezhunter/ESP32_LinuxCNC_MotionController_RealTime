#pragma once

#include "Types.hpp"
//#include "Config.h"
#include "ConfigManager.h"
#include <esp_now.h>     // ESP-NOW wifi point-to-point for another ESP32 to wireless send data to LinuxCNC
#include "ConsoleMenus.h"
#include <ESP32Console.h>
#include "ESP32Console/Helpers/PWDHelpers.h"
#include "Ticker.h"
#include <ESP32Encoder.h>


typedef struct espnow_message_mpg {
  char a[32];
  int b;
  float c;
  bool d;
  int mpg1;
} espnow_message_mpg;

inline espnow_message_mpg mpgData;
inline espnow_message myData; // Create a espnow_message struct called myData
//inline espnow_add_peer_msg espnowAddPeerMsg; // Used to send local mac address to motion controller. Gets added to its esp-now peer list enabling bi-directional communication
//inline esp_now_peer_info_t peerInfo;

inline long lastPacketRxTimeMs = 0;
inline Ticker sendTimer;
inline Ticker encoderTimer;

inline ESP32Encoder encoder;
inline int encoderLastCnt = 0;

inline uint8_t remotePeerAddress[6] = {0};

void setupClientMode();
void stopProcessingClient();
void espNowOnDataSentClientMode(const uint8_t *mac_addr, esp_now_send_status_t status);
void espNowOnDataRecvClientMode(const uint8_t * mac, const uint8_t *incomingData, int len);
void loop_Core0_EspNowSenderClientTask(void* parameter);
void checkEncoder();
void sendMPG();