#pragma once

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "AsyncTCP.h"


#define DRIVER_RMT 1
#define UDP_PACKET_BUF_SIZE 70 /* Need to update this if increasing MAX_STEPPER from default of 6 so that cmd and fb structs can be serialised  */

#define UDP_RECEIVEPACKET_BIT   (1UL << 0UL) // zero shift for bit0

#define MAX_STEPPER 6
#define MAX_INPUTS 7
#define MAX_OUTPUTS 7

#define CTRL_DIRSETUP 0b00000001
#define CTRL_ACCEL    0b00000010
#define CTRL_RESTORE  0b00001000
#define CTRL_PWMFREQ  0b00000100
#define CTRL_READY    0b01000000
#define CTRL_ENABLE   0b10000000

#define IO_00 0b00000001
#define IO_01 0b00000010
#define IO_02 0b00000100
#define IO_03 0b00001000
#define IO_04 0b00010000
#define IO_05 0b00100000
#define IO_06 0b01000000
#define IO_07 0b10000000


#define AXIS_STATE_STOPPED            0b01111111 // Send to clear all bits
#define AXIS_STATE_MOVE_REQ           0b00000001 // 1=MOVE 0=STOP
#define AXIS_STATE_MOVE_REQ_DIR       0b00000010 // 1=FWD 0=REV
#define AXIS_STATE_MOVE_ACCEL_REQ     0b00001000 // 1=ACCEL 0=DECEL
#define AXIS_STATE_MOVE_COAST         0b00010000 // 1=AT SPEED 0=Not At Speed
#define AXIS_STATE_MOVING_DIR         0b00100000 // 1=FWD 0=REV
#define AXIS_STATE_ACCELERATING       0b01000000 // 1=ACCEL 0=DECEL
#define AXIS_STATE_COASTING           0b10000000 // 1=AT SPEED 0=NOT AT SPEED


struct cmdPacket {
    uint8_t control;
    uint8_t io;
    uint16_t pwm[MAX_OUTPUTS];
    volatile int32_t pos[MAX_STEPPER];
    volatile int32_t vel[MAX_STEPPER];
}; 

struct fbPacket {
    uint8_t control;
    volatile uint8_t io;
    volatile int32_t pos[MAX_STEPPER];
    volatile int32_t vel[MAX_STEPPER];
    uint32_t udp_seq_num;
};


struct stepper_config_s {
  uint8_t step;
  uint8_t enable_low_active;
  uint8_t enable_high_active;
  uint8_t direction;
  uint16_t dir_change_delay;
  bool direction_high_count_up;
  bool auto_enable;
  uint32_t on_delay_us;
  uint16_t off_delay_ms;
};

struct inputPin_Config_s {
  String name;
  uint8_t udp_in_num;
  uint8_t pin_mode;
  gpio_num_t gpio_number; /* Use GPIO_NUM_NC for not connected pins/unused inputs */ 
  int register_address;
  int register_bit;
  int fb_input_mask;
};

void IRAM_ATTR inputPinChangeISR(void* args);

inline uint8_t telnetClientConnected = 0;
inline AsyncClient* telnetClient; // TODO handle multiple telnet client connections for debugging - single one for now
const char ctrlz_bytes[5] = {'\xff','\xed','\xff','\xfd','\x06'}; // CTRL+Z keypress bytes for linux telnet client so client can disconnect easily

inline size_t logMessage(const char *format, ...) {
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
    len = Serial.print(buffer);
    if (buffer != temp) {
        delete[] buffer;
    }
    return len;  
}

inline void debugAxisState(uint8_t currState);
inline void ARDUINO_ISR_ATTR updateAxisState(uint8_t axisNum, uint8_t bit, bool new_value );
inline void telnetDisconnectAllClients();
inline void onTelnetClient(void *s, AsyncClient* c);
inline void onTelnetClientDisconnected(void *s, AsyncClient* c);
inline void onTelnetClientData(void *s, AsyncClient* c, void *buf, size_t len);

inline void otaUpdateStart();
inline void otaUpdateEnd();
inline void otaProgress(unsigned int progress, unsigned int total);