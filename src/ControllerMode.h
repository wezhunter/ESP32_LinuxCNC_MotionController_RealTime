#pragma once

#ifndef CONTROLLERMODE_H
#define CONTROLLERMODE_H

#include "Types.hpp"
#include "Config.h"
#include "ConfigManager.h"
#include <esp_now.h>     // ESP-NOW wifi point-to-point for another ESP32 to wireless send data to LinuxCNC
#include "ConsoleMenus.h"
#include <ESP32Console.h>
#include "ESP32Console/Helpers/PWDHelpers.h"
#include "FastAccelStepper.h"

#if ESP32_RMII_ETHERNET
    #include "AsyncUDP.h"
    #include "AsyncTCP.h"
    inline AsyncUDP udpClient;
    inline AsyncUDP udpServer;
    inline AsyncServer telnetServer(23);
#elif ESP32SX_USB_ETHERNET // TODO - WIP
    #include "AsyncUDP.h"
    #include "AsyncTCP.h"
    //#include <SoftwareSerial.h>
    inline AsyncUDP udpClient;
    inline AsyncUDP udpServer;
    inline AsyncServer telnetServer(23);
#elif ESP32_SPI_ETHERNET
    #include <AsyncUDP_ESP32_Ethernet.h> /* W5500 SPI Native Ethernet lib */
    inline AsyncUDP udpClient;
    inline AsyncUDP udpServer;
    inline AsyncServer telnetServer(23);
#endif

#if (OUT_00_PIN != GPIO_NUM_NC)
#undef OUT_00_H
#define OUT_00_H digitalWrite(OUT_00_PIN, HIGH)
#endif
#if (OUT_00_PIN != GPIO_NUM_NC)
#undef OUT_00_L
#define OUT_00_L digitalWrite(OUT_00_PIN, LOW)
#endif

#if (OUT_01_PIN != GPIO_NUM_NC)
#undef OUT_01_H
#define OUT_01_H digitalWrite(OUT_01_PIN, HIGH)
#endif
#if (OUT_01_PIN != GPIO_NUM_NC)
#undef OUT_01_L
#define OUT_01_L digitalWrite(OUT_01_PIN, LOW)
#endif

#if (OUT_02_PIN != GPIO_NUM_NC)
#undef OUT_02_H
#define OUT_02_H digitalWrite(OUT_02_PIN, HIGH)
#endif
#if (OUT_02_PIN != GPIO_NUM_NC)
#undef OUT_02_L
#define OUT_02_L digitalWrite(OUT_02_PIN, LOW)
#endif
#if (OUT_03_PIN != GPIO_NUM_NC)
#undef OUT_03_H
#define OUT_03_H digitalWrite(OUT_03_PIN, HIGH)
#endif
#if (OUT_03_PIN != GPIO_NUM_NC)
#undef OUT_03_L
#define OUT_03_L digitalWrite(OUT_03_PIN, LOW)
#endif
#if (OUT_04_PIN != GPIO_NUM_NC)
#undef OUT_04_H
#define OUT_04_H digitalWrite(OUT_04_PIN, HIGH)
#endif
#if (OUT_04_PIN != GPIO_NUM_NC)
#undef OUT_04_L
#define OUT_04_L digitalWrite(OUT_04_PIN, LOW)
#endif
#if (OUT_05_PIN != GPIO_NUM_NC)
#undef OUT_05_H
#define OUT_05_H digitalWrite(OUT_05_PIN, HIGH)
#endif
#if (OUT_05_PIN != GPIO_NUM_NC)
#undef OUT_05_L
#define OUT_05_L digitalWrite(OUT_05_PIN, LOW)
#endif


inline FastAccelStepperEngine stepperEngine;

inline EventGroupHandle_t  eventUDPPacketStateGroup;
inline volatile bool motorsSetup = false;
inline FastAccelStepper *stepper[MAX_STEPPER];
inline hw_timer_t * timerServoCmds = NULL;
inline cmdPacket cmd = { 0 };
inline bool debugAxisMovements = false;
inline bool motorsMoving = false;
inline bool isMovementRunning[MAX_STEPPER] = { false };
inline bool prevIsMovementRunning[MAX_STEPPER] = { false };
inline volatile uint32_t udp_tx_seq_num = 0;
inline volatile uint32_t udp_rx_seq_num = 0;
inline volatile uint32_t udpPacketTxErrors = 0;
inline volatile uint32_t udpPacketRxErrors = 0;
inline volatile uint8_t prev_cmd_control = 0;

inline volatile uint32_t udpTxLoopCount = 0;
inline volatile uint32_t udpRxLoopCount = 0;
inline volatile uint32_t servoCmdsLoopCount = 0;

inline volatile unsigned long ul_dirSetup[MAX_STEPPER] = { 1000 }; 
/*==================================================================*/


inline volatile uint8_t prevRampState[MAX_STEPPER] = {0}; // Reserved for future use

inline const double axisVelScaleFactor = 1.02; // Reduce max accel of motors by 2% to ensure FAS command position buffer remains full

inline uint8_t prev_ctrl_ready = false;

inline volatile bool machineEnabled = false;
inline volatile bool manualMove = false;
inline volatile uint8_t axisState[MAX_STEPPER] = {0};
inline volatile uint8_t prev_axisState[MAX_STEPPER] = {0};

inline long lastMsg_ProfileStats = 0;

inline static uint8_t packetBufferTx[UDP_PACKET_BUF_SIZE]; // UDP buffer for sending data
inline static uint8_t packetBufferRx[UDP_PACKET_BUF_SIZE]; // UDP buffer for receiving data
inline volatile unsigned long ul_udptxrx_watchdog; /* Nothing RX/TX for 5s triggers watchdog clearing machine state */

inline IRAM_ATTR  xQueueHandle axisStateInterruptQueue;


void setupControllerMode();
void setupControllerModeCore0();
void stopProcessingController();
void loop_Core1_EspNowSenderControllerTask(void* parameter);
void espNowOnDataRecvController(const uint8_t * mac, const uint8_t *incomingData, int len);
void espnowOnDataSentController(const uint8_t *mac_addr, esp_now_send_status_t status);
void espNowOnDataRecvControllerMode(const uint8_t * mac, const uint8_t *incomingData, int len);
void IRAM_ATTR commandHandler();
void IRAM_ATTR outputHandler();
size_t IRAM_ATTR sendUDPFeedbackPacket();
void IRAM_ATTR onUDPRxPacketCallBack(AsyncUDPPacket packet);
void IRAM_ATTR loop_Core0_UDPSendTask(void* parameter);
void IRAM_ATTR loop_Core0_CommandHandlerTask(void* parameter);
void loop_Core1_ServoStatsTask(void* parameter);
void debugAxisState(uint8_t axisNum);
void IRAM_ATTR updateAxisState(uint8_t axisNum, uint8_t mask );
void IRAM_ATTR updateAxisState(uint8_t axisNum, uint8_t bit, bool new_value );
void IRAM_ATTR ServoMovementCmds_ISR();
bool fasExternalCallForPin(uint8_t pin, uint8_t value);
bool setupMotors();
size_t logMotorDebugMessage(uint8_t row, const char *format, ...);

#endif