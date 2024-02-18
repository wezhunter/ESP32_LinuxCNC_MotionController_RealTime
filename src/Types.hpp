#pragma once
#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include <esp_now.h>
#include "Config.h"
#include "FastAccelStepper.h"
#include "AsyncTCP.h"
#include <ESP32Console.h>
#include "ArduinoNvs.h"
#include "ConfigManager.h"
#include <WiFi.h>

#if CONFIG_IDF_TARGET_ESP32S2
    #define Serial Serial0
#endif


const String version_number =  "20240214dev1";

#define DRIVER_RMT 1
#define UDP_PACKET_BUF_SIZE 70 /* Need to update this if increasing MAX_STEPPER from default of 6 so that cmd and fb structs can be serialised  */

#define UDP_RECEIVE_PACKET_BIT  0b00000001
#define UDP_SEND_PACKET_BIT     0b00000011
#define ESPNOW_SEND_BIT         0b00000111


#define CTRL_DIRSETUP 0b00000001
#define CTRL_ACCEL    0b00000010
#define CTRL_RESTORE  0b00001000
#define CTRL_PWMFREQ  0b00000100
#define CTRL_READY    0b01000000
#define CTRL_ENABLE   0b10000000


#define AXIS_STATE_STOPPED            0b01111111 // Send to clear all bits
#define AXIS_STATE_MOVE_REQ           0b00000001 // 1=MOVE 0=STOP
#define AXIS_STATE_MOVE_REQ_DIR       0b00000010 // 1=FWD 0=REV
#define AXIS_STATE_MOVE_ACCEL_REQ     0b00000100 // 1=ACCEL 0=DECEL
#define AXIS_STATE_MOVE_COAST         0b00001000 // 1=AT SPEED 0=Not At Speed
#define AXIS_STATE_MOVING_DIR         0b00010000 // 1=FWD 0=REV
#define AXIS_STATE_ACCELERATING       0b00100000 // 1=ACCEL 0=NA
#define AXIS_STATE_DECELERATING       0b01000000 // 1=DECEL 0=NA
#define AXIS_STATE_COASTING           0b10000000 // 1=AT SPEED 0=NOT AT SPEED


struct cmdPacket {
    uint8_t control;
    uint8_t io;
    uint16_t pwm[MAX_OUTPUTS];
    volatile int32_t pos[MAX_STEPPER];
    volatile int32_t vel[MAX_STEPPER];
    volatile int32_t vel_limit[MAX_STEPPER];
    uint16_t feedrateoverride;
}; 

struct fbPacket {
    uint8_t control;
    volatile uint8_t io;
    volatile int32_t pos[MAX_STEPPER];
    volatile int32_t vel[MAX_STEPPER];
    uint32_t udp_seq_num;
};

typedef struct espnow_add_peer_msg {
  uint8_t mac_adddress[6];
} espnow_add_peer_msg;

typedef struct espnow_message {
  char a[32];
  int b;
  float c;
  bool d;
} espnow_message;

inline uint8_t telnetClientConnected = 0;
inline AsyncClient* telnetClient; // TODO handle multiple telnet client connections for debugging - single one for now
const char ctrlz_bytes[5] = {'\xff','\xed','\xff','\xfd','\x06'}; // CTRL+Z keypress bytes for linux telnet client so client can disconnect easily

inline bool consoleLogging = true;
inline ESP32Console::Console console;

inline volatile uint32_t espnowTxPackets = 0;
inline volatile uint32_t inputInterruptCounter = 0;

inline EventGroupHandle_t  xEventStateChangeGroup;
inline EventGroupHandle_t  xEventStateChangeGroupClient;

inline bool safeMode = false;
inline volatile bool runLoops = false;
inline bool startupStage1Complete = false;

inline bool pwm_enable[MAX_OUTPUTS] = { false, false, false, false, false, false };

inline uint8_t espnow_peer_configured = 0;

inline espnow_message espnowData;
inline espnow_add_peer_msg espnowAddPeerMsg;
inline esp_now_peer_info_t peerInfo;

inline fbPacket fb = { 0 };

inline uint8_t sendEspNowPacket = 0;


inline size_t logErrorMessage(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1) {
        buffer = new char[len + 1];
        if (!buffer) {
            return 0;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }

    if (telnetClientConnected > 0) {
        telnetClient->write(buffer);
    }

    len = printf("\n\e[F\e[1;91mERROR: %s\r\n\e[E\e[0m",buffer);

    if (buffer != temp) {
        delete[] buffer;
    }
    
    return len;  
}
inline size_t logWarningMessage(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1) {
        buffer = new char[len + 1];
        if (!buffer) {
            return 0;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }

    if (telnetClientConnected > 0) {
        telnetClient->write(buffer);
    }

    len = printf("\n\e[F\e[1;93mWARN: %s\r\n\e[E\e[0m",buffer);

    if (buffer != temp) {
        delete[] buffer;
    }
    
    return len;  
}


inline size_t logMessage(const char *format,...) {
    if (!consoleLogging)
        return 0;
    
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1) {
        buffer = new char[len + 1];
        if (!buffer) {
            return 0;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }

    if (telnetClientConnected > 0) {
        telnetClient->write(buffer);
    }

    len = printf("\e[0m\n\e[F%s\r\n\e[E",buffer);

    if (buffer != temp) {
        delete[] buffer;
    }
    return len;  
}


extern void telnetDisconnectAllClients();
extern void onTelnetClient(void *s, AsyncClient* c);
extern void onTelnetClientDisconnected(void *s, AsyncClient* c);
extern void onTelnetClientData(void *s, AsyncClient* c, void *buf, size_t len);

extern void otaUpdateStart();
extern void otaUpdateEnd();
extern void otaProgress(unsigned int progress, unsigned int total);

extern void setWifiState(bool newState);
extern void stopProcessing();
extern void IRAM_ATTR inputPinChangeISR(void* args);
extern void IRAM_ATTR outputHandler();
extern void IRAM_ATTR inputHandler();
extern void startSafeModeEthernet();

#endif