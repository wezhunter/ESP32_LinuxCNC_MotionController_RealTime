/*    External step generator and IO interface for LinuxCNC over Ethernet
 *    with dual-core ESP32 and built-in Ethernet RMIi PHY LAN8270 or SPI Ethernet W5500 modules
 *
 *    Copyright 2024 Wez Hunter <wez at hunter.to>
 *    Copyright 2022 Juhász Zoltán <juhasz.zoltan at freemail dot hu> (Original HAL2UDP project that this was based on but firmware no longer resembles much of it)
 * 
 *    version:20240119
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*    Hardware: TODO document all hardware, pins and available mappings for each use-case
 *
 *    ESP32-WROOM or WROVER modules - Two Cores 240Mhz each. 
 *    8 channel RMT peripheral so can drive 8 stepper motors with RMT peripheral hardware up-to 200khz total with native IO. 
 *    The MCPWM or PCNT (Pulse Counter) hardware peripherals are NOT used therefore leaving them free for hardware pulse counting Servo Encoder tracking later
 *    https://github.com/gin66/FastAccelStepper/tree/master?tab=readme-ov-file#overview
 *    https://github.com/gin66/FastAccelStepper/tree/master?tab=readme-ov-file#esp32-2
 *    
 * 
 *    Software setup - 
 *    Core 1 - Arduino loops, OTA Updates and Serial Stats Task (disable in config as needed - no impact measured if left enabled)
 *    Core 0 - Ethernet, UDP Client & Server, Motor commands, Servo Step Gen, Input handler Task, Telnet server
 *      
 *    GPIO pins - see hardware docs for each board and map pins to your needs. All settings in Config.h. Motor STEP pins must be on native GPIO not on a I2C/I2S bus for RMT peripheral to work. Everything else is free reign.
 *
 */


#include <Arduino.h>
#include "FastAccelStepper.h"
#include "Config.h"
#include "Types.hpp"

#ifdef ARDUINO_ESP32_MKS_DLC32
    #include "Pins.h"
    #include "I2SOut.h"
#endif

#include <WiFi.h>        // For connecting ESP32 to WiFi
#include <esp_now.h>     // ESP-NOW wifi point-to-point for another ESP32 to wireless send data to LinuxCNC
#include <ArduinoOTA.h>  // For enabling over-the-air updates via network either WiFi or Ethernet

#include <esp_timer.h>
#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>
#include "esp32-hal-log.h"

#ifdef ESP32_RMII_ETHERNET
    #include <ETH.h>
#elif defined(ESP32SX_USB_ETHERNET) // TODO - WIP
    //#include "bsp/board_api.h"
    // #include "tusb.h"
    // #include "dhserver.h"
    // #include "dnserver.h"
    #include "lwip/init.h"
    #include "lwip/timeouts.h"
    //#include "lwip/ethip6.h"
#elif defined(ESP32_SPI_ETHERNET) 
    #include <AsyncUDP_ESP32_Ethernet.h>
#endif


/*==================================================================*/
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
hw_timer_t * timerServoCmds = NULL;

EventGroupHandle_t  xEventUDPPacketStateGroup = xEventGroupCreate();
EventGroupHandle_t  xEventStateChangeGroup = xEventGroupCreate();

xQueueHandle IRAM_ATTR inputInterruptQueue;
xQueueHandle IRAM_ATTR axisStateInterruptQueue;
/*==================================================================*/
bool eth_connected = false;

uint8_t spinProgress = 0;
char spinChar = '|';

/*==================================================================*/

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

/*==================================================================*/

#ifdef ESP32_RMII_ETHERNET
    #include "AsyncUDP.h"
    #include "AsyncTCP.h"
    AsyncUDP udpClient;
    AsyncUDP udpServer;
    AsyncServer telnetServer(23);
#elif defined(ESP32SX_USB_ETHERNET) // TODO - WIP
    #include "AsyncUDP.h"
    #include "AsyncTCP.h"
    //#include <SoftwareSerial.h>
    AsyncUDP udpClient;
    AsyncUDP udpServer;
    AsyncServer telnetServer(23);
#elif defined(ESP32_SPI_ETHERNET)
    AsyncUDP udpClient;
    AsyncUDP udpServer;
    AsyncServer telnetServer(23);
#endif



volatile uint32_t udp_tx_seq_num = 0;
volatile uint32_t udp_rx_seq_num = 0;

uint8_t inputPinInterruptFired[MAX_INPUTS] = { 0 };

cmdPacket cmd = { 0 };
fbPacket fb = { 0 };

espnow_message espnowData;
espnow_add_peer_msg espnowAddPeerMsg;
esp_now_peer_info_t peerInfo;
bool espnow_peer_configured = false;

volatile uint8_t prev_cmd_control = 0;

volatile unsigned long ul_dirSetup[MAX_STEPPER] = { 1000 }; 
/*==================================================================*/

FastAccelStepper *stepper[CONF_NUM_STEPPERS];
volatile uint8_t prevRampState[MAX_STEPPER] = {0}; // Reserved for future use

const double axisVelScaleFactor = 1.02; // Reduce max accel of motors by 2% to ensure FAS command position buffer remains full

volatile bool runLoops = false;

const int8_t pwm_pin[MAX_OUTPUTS] = { OUT_00_PIN, OUT_01_PIN, OUT_02_PIN, OUT_03_PIN, OUT_04_PIN, OUT_05_PIN };
bool pwm_enable[MAX_OUTPUTS] = { false, false, false, false, false, false };

long lastMsg_ProfileStats = 0;

uint8_t prev_ctrl_ready = false;

volatile bool motorsSetup = false;
volatile bool machineEnabled = false;
volatile bool motorsMoving = false;
volatile bool manualMove = false;
bool debugAxisMovements = false;


volatile uint8_t axisState[MAX_STEPPER] = {0};
volatile uint8_t prev_axisState[MAX_STEPPER] = {0};

//volatile int8_t sendUdpPacket = 0;
//volatile int8_t udpRxPacket = 0;

uint8_t sendEspNowPacket = 0;

volatile uint32_t udpPacketTxErrors = 0;
volatile uint32_t udpPacketRxErrors = 0;
volatile uint32_t espnowTxPackets = 0;

volatile uint32_t inputInterruptCounter = 0;

volatile unsigned long ul_udptxrx_watchdog; /* Nothing RX/TX for 5s triggers watchdog clearing machine state */

uint8_t packetBufferTx[UDP_PACKET_BUF_SIZE]; // UDP buffer for sending data
uint8_t packetBufferRx[UDP_PACKET_BUF_SIZE]; // UDP buffer for receiving data


/*===============================================================================*/
/* TODO Warrants refactoring to use dynamic pin setup much like input pin config */

void IRAM_ATTR outputHandler()
{
    static int last_pwm[6] = { 0, 0, 0, 0, 0, 0 };

    bool enable = cmd.control & CTRL_ENABLE;

    if (pwm_enable[0]) {
        if (enable) {
            if (last_pwm[0] != cmd.pwm[0]) {
                last_pwm[0] = cmd.pwm[0];
                ledcWrite(0, last_pwm[0]);
            }
        } else {
            ledcWrite(0, 0);
            last_pwm[0] = 0;
        }
    } else {
        if (enable) {
            (cmd.io & IO_00) ? OUT_00_H : OUT_00_L;
        }
        else {
            OUT_00_L;
        }
    }

    if (pwm_enable[1]) {
        if (enable) {
            if (last_pwm[1] != cmd.pwm[1]) {
                last_pwm[1] = cmd.pwm[1];
                ledcWrite(2, last_pwm[1]);
            }
        } else {
            ledcWrite(2, 0);
            last_pwm[1] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_01) ? OUT_01_H : OUT_01_L;
        else
            OUT_01_L;
    }

    if (pwm_enable[2]) {
        if (enable) {
            if (last_pwm[2] != cmd.pwm[2]) {
                last_pwm[2] = cmd.pwm[2];
                ledcWrite(4, last_pwm[2]);
            }
        } else {
            ledcWrite(4, 0);
            last_pwm[2] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_02) ? OUT_02_H : OUT_02_L;
        else
            OUT_02_L;
    }

    if (pwm_enable[3]) {
        if (enable) {
            if (last_pwm[3] != cmd.pwm[3]) {
                last_pwm[3] = cmd.pwm[3];
                ledcWrite(6, last_pwm[3]);
            }
        } else {
            ledcWrite(6, 0);
            last_pwm[3] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_03) ? OUT_03_H : OUT_03_L;
        else
            OUT_03_L;
    }

    if (pwm_enable[4]) {
        if (enable) {
            if (last_pwm[4] != cmd.pwm[4]) {
                last_pwm[4] = cmd.pwm[4];
                ledcWrite(8, last_pwm[4]);
            }
        } else {
            ledcWrite(8, 0);
            last_pwm[4] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_04) ? OUT_04_H : OUT_04_L;
        else
            OUT_04_L;
    }

    if (pwm_enable[5]) {
        if (enable) {
            if (last_pwm[5] != cmd.pwm[5]) {
                last_pwm[5] = cmd.pwm[5];
                ledcWrite(10, last_pwm[5]);
            }
        } else {
            ledcWrite(10, 0);
            last_pwm[5] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_05) ? OUT_05_H : OUT_05_L;
        else
            OUT_05_L;
    }
}
/*==================================================================*/
void IRAM_ATTR inputHandler()
{
    for (uint8_t i = 0; i < MAX_INPUTS; i++) {
        const struct inputPin_Config_s *pin_config = &inputPinsConfig[i];
        if (pin_config->gpio_number != GPIO_NUM_NC) {
            bool registerBitState = false;
            if (i == 0) {
                registerBitState = (bool)REG_GET_BIT(pin_config->register_address, pin_config->register_bit);
                (registerBitState) ? fb.io |= pin_config->fb_input_mask : fb.io = 0;
            }
            else {      
                registerBitState = (bool)REG_GET_BIT(pin_config->register_address, pin_config->register_bit);
                if (registerBitState)
                    fb.io |= pin_config->fb_input_mask;
            }
        } else { // If pin not connected
            if (i == 0) // if pinindex is 0
                fb.io = 0; // set initial fb.io value
        }
    }
    xEventGroupSetBits(xEventStateChangeGroup, ESPNOW_SEND_BIT); // Send an async ESP-NOW packet as required
}
/*==================================================================*/

void IRAM_ATTR commandHandler()
{
    if (cmd.control & CTRL_READY) {
        for (int i = 0; i < CONF_NUM_STEPPERS; i++) { // Placeholder if anything needs to occur at 1khz RTOS tick relating to motors
        }
    }

    if (prev_cmd_control != cmd.control) {
        logMessage("cmd.control = 0x%02x\r\n", cmd.control);
        
        if (cmd.control & CTRL_ENABLE) { // Called when Machine turned ON in LinuxCNC (Ready + Enable bits)
            logMessage("CMD: Machine ON\r\n");
            udp_tx_seq_num = 0;
            
            for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
                fb.vel[i] = 0;                
                stepper[i]->setCurrentPosition(cmd.pos[i]);
                stepper[i]->enableOutputs();
                stepper[i]->forceStopAndNewPosition(cmd.pos[i]);
                stepper[i]->setCurrentPosition(cmd.pos[i]);
            }
            motorsMoving = false;
            machineEnabled = true;
            inputHandler(); /* Run once to get current state and let ISR handle the rest */
        } else if ((cmd.control & CTRL_ENABLE) == 0) { // Called when Machine turned OFF in LinuxCNC (!Ready + Enable bits)
            logMessage("CMD: Machine OFF\r\n");
            
            motorsMoving = false;
            machineEnabled = false;
            udp_tx_seq_num = 0;
            
            for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
                fb.pos[i] = cmd.pos[i];
                stepper[i]->forceStop();
                stepper[i]->disableOutputs();               
                fb.vel[i] = 0;
                stepper[i]->setCurrentPosition(cmd.pos[i]);
            }
            inputHandler(); /* Run once to get current state and let ISR handle the rest */
            outputHandler();
        }

        prev_cmd_control = cmd.control;
    }
   
    if (!(fb.control & CTRL_READY)) {
        for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
            stepper[i]->disableOutputs();
        }
        if ((fb.control & CTRL_DIRSETUP)
            && (fb.control & CTRL_ACCEL)
            && (fb.control & CTRL_RESTORE)
            && (fb.control & CTRL_PWMFREQ)) {
            fb.control |= CTRL_READY;
            logMessage("Setting FB Control Ready\r\n");
            

        } else if (cmd.control & CTRL_DIRSETUP) {
            fb.control |= CTRL_DIRSETUP;
        } else if (cmd.control & CTRL_ACCEL) {
            fb.control |= CTRL_ACCEL;
            
            logMessage("CTRL_ACCEL\r\n");
            

            for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
                int newAccel = cmd.pos[i] * 2.0;
                logMessage("Setting stepper[%d] Accel to: %d\r\n", i, newAccel);
                stepper[i]->setAcceleration(newAccel);
            }
        } else if (cmd.control & CTRL_RESTORE) {
            fb.control |= CTRL_RESTORE;
            logMessage("CTRL_RESTORE\r\n");
            for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
                stepper[i]->setCurrentPosition(cmd.pos[i]);
                fb.pos[i] = cmd.pos[i];
            }

        } else if (cmd.control & CTRL_PWMFREQ) {
            fb.control |= CTRL_PWMFREQ;
            logMessage("Setup output pins\r\n");
            for (int i = 0; i < 6; i++) {
                if (cmd.pwm[i]) {
                    logMessage("Setup PWM output [%d]\r\n", i);
                    ledcAttachPin(pwm_pin[i], i * 2);
                    ledcSetup(i * 2, cmd.pwm[i], 10);
                    ledcWrite(i * 2, 0);
                    pwm_enable[i] = true;
                } else {
                    logMessage("Setup output [%d]\r\n", i);
                    if (i == 0) {
                        if (OUT_00_PIN != GPIO_NUM_NC) {
                            pinMode(OUT_00_PIN, OUTPUT);
                            digitalWrite(OUT_00_PIN, 0);
                        }
                    } else if (i == 1) {
                        if (OUT_01_PIN != GPIO_NUM_NC) {
                        pinMode(OUT_01_PIN, OUTPUT);
                        digitalWrite(OUT_01_PIN, 0);
                        }
                    } else if (i == 2) {
                        if (OUT_02_PIN != GPIO_NUM_NC) {
                            pinMode(OUT_02_PIN, OUTPUT);
                            digitalWrite(OUT_02_PIN, 0);
                        }
                    } else if (i == 3) {
                        if (OUT_03_PIN != GPIO_NUM_NC) {
                            pinMode(OUT_03_PIN, OUTPUT);
                            digitalWrite(OUT_03_PIN, 0);
                        }
                    } else if (i == 4) {
                        if (OUT_04_PIN != GPIO_NUM_NC) {
                            pinMode(OUT_04_PIN, OUTPUT);
                            digitalWrite(OUT_04_PIN, 0);
                        }
                    } else if (i == 5) {
                        if (OUT_05_PIN != GPIO_NUM_NC) {
                            pinMode(OUT_05_PIN, OUTPUT);
                            digitalWrite(OUT_05_PIN, 0);
                        }
                    }
                }
            }
        }
    }
}

/*==================================================================*/
// ESP-NOW P2P WiFi callback when data is sent
void espnowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //logMessage("\r\nLast Packet Send Status:\t");
//   if (status != ESP_NOW_SEND_SUCCESS) {
//     logMessage(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\r\n" : "Delivery Fail\r\n");
//   }
  
}

// ESP-NOW P2P WiFi callback function that will be executed when data is received
void espnowOnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    //logMessage("ESP-NOW: RX: %d\r\n", len);
    if (len == sizeof(espnowAddPeerMsg)) {
        logMessage("ESP-NOW: Add new peer mac message received\r\n");
        memcpy(&espnowAddPeerMsg, incomingData, sizeof(espnowAddPeerMsg));
        //String mac_addr = String(espnowAddPeerMsg.mac_adddress);
        //logMessage("ESP-NOW: Adding peer MAC: %s\r\n", espnowAddPeerMsg.mac_adddress);
        logMessage("ESP-NOW: Adding peer MAC: '%02X:%02X:%02X:%02X:%02X:%02X'\r\n", 
            espnowAddPeerMsg.mac_adddress[0],espnowAddPeerMsg.mac_adddress[1],espnowAddPeerMsg.mac_adddress[2],espnowAddPeerMsg.mac_adddress[3],
            espnowAddPeerMsg.mac_adddress[4],espnowAddPeerMsg.mac_adddress[5],espnowAddPeerMsg.mac_adddress[6]);
        
        // Register peer
        memcpy(peerInfo.peer_addr, espnowAddPeerMsg.mac_adddress, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
        
        esp_now_del_peer(peerInfo.peer_addr); // Delete peer mac first to remove dups (lazy but easiest way)

        // Add peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK){
            logMessage("Failed to add ESP-NOW peer\r\n");
            espnow_peer_configured = false;
            return;
        } else {
            logMessage("Added ESP-NOW Peer\r\n");
            espnow_peer_configured = true;
        }
        

    } else if (len == sizeof(espnowData)) {
        memcpy(&espnowData, incomingData, sizeof(espnowData));

        // Serial.print("Bytes received: ");
        // Serial.println(len);
        // Serial.print("Char: ");
        // Serial.println(espnowData.a);
        // Serial.print("Int: ");
        // Serial.println(espnowData.b);
        // Serial.print("Float: ");
        // Serial.println(espnowData.c);
        // Serial.print("Bool: ");
        // Serial.println(espnowData.d);
        // Serial.println();
    }
}

void WiFiEvent(WiFiEvent_t event) 
{
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        logMessage("WiFi IP: %s\r\n", WiFi.localIP().toString().c_str());
        break; 
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        logMessage("WiFi Connected!\r\n");
        break; 
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        telnetDisconnectAllClients();
        logMessage("WiFi Disconnected!\r\n");
        break; 
#if CONFIG_IDF_TARGET_ESP32S2
#else
    case ARDUINO_EVENT_ETH_START:
        logMessage("Ethernet Started\r\n");
        ETH.setHostname("esp32-linuxcnc");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        logMessage("Ethernet Link UP\r\n");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        logMessage("Ethernet MAC: %s, IPv4: %s, %s Duplex, Speed: %dMbps\r\n", ETH.macAddress().c_str(), ETH.localIP().toString().c_str(), (ETH.fullDuplex()) ? "Full" : "Half",  ETH.linkSpeed());
        eth_connected = true;
        
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        telnetDisconnectAllClients();
        logMessage("Ethernet Link DOWN\r\n");
        eth_connected = false;
        break;
    case ARDUINO_EVENT_ETH_STOP:
        logMessage("Ethernet Stopped\r\n");
        eth_connected = false;
        break;
#endif
    default:
        break;
    }
}


/*==================================================================*/
size_t IRAM_ATTR sendUDPFeedbackPacket() 
{
    bool result = true;
    fb.udp_seq_num++;
    memcpy(&packetBufferTx, &fb, sizeof(fb));
    size_t res = udpClient.write(packetBufferTx, sizeof(fb));
    if (res != sizeof(fb)) {
        udpPacketTxErrors++;
    }
    
    memset(packetBufferTx, 0, sizeof(packetBufferTx));
    
    udp_tx_seq_num++;
    
    return res;
}

void IRAM_ATTR onUDPRxPacketCallBack(AsyncUDPPacket packet)
{
    if (!runLoops)
        return;
    
    if (packet.length() > 0) {
        uint8_t chk = 71;
        for (int i = 0; i < sizeof(cmd); i++) {
            chk ^= packet.data()[i];
        }

        if (packet.data()[sizeof(cmd)] == chk) {
            udp_rx_seq_num++;
            memcpy(&cmd, packet.data(), sizeof(cmd));
            xEventGroupSetBits(xEventUDPPacketStateGroup, UDP_RECEIVE_PACKET_BIT);
        } else {
            udpPacketRxErrors++;
            if (udpPacketRxErrors > 1000) {
                logMessage("**ERROR**: Excessive UDP RX errors!. Size: %d, RxLen:%d\r\n", sizeof(cmd), packet.length());
                udpPacketRxErrors = 0;
            }
            return;
        }
        
        xEventGroupSetBits(xEventUDPPacketStateGroup, UDP_SEND_PACKET_BIT); // Send an async ESP-NOW packet as required
    }
}

void IRAM_ATTR loop_Core0_UDPSendTask(void* parameter)
{
    logMessage("loop_Core0_UDPSendTask running...\r\n");
    
    logMessage("UDP Send connect to IP: %s, Port: %d, Result: %d\r\n", ip_host.toString().c_str(), udpClientPort, udpClient.connect(ip_host,udpClientPort));
    size_t sent_len = 0;
    const EventBits_t xBitsToWaitFor = (UDP_SEND_PACKET_BIT);
    EventBits_t xEventGroupValue;
    const TickType_t xTicksToWait = 1 / portTICK_PERIOD_MS; // Wait 1000ms
    
    while (runLoops) {
        if (!udpClient.connected()) {
            logMessage("UDP Client unable to connect. Retrying in 5s\r\n");
            udpClient.connect(ip_host, udpClientPort);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        
        xEventGroupValue  = xEventGroupWaitBits(xEventUDPPacketStateGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY); 

        if (( xEventGroupValue & ( UDP_SEND_PACKET_BIT ) ) == ( UDP_SEND_PACKET_BIT )) /* If UDP Packet TX bit is set */
        {
            sendUDPFeedbackPacket();
            xEventGroupClearBits(xEventUDPPacketStateGroup, UDP_SEND_PACKET_BIT);
        }
        
        //vTaskDelay(1);
    }
    
    logMessage("Exiting loop_Core0_UDPSendTask\r\n");
    vTaskDelete(NULL);
}

/*==================================================================*/
void IRAM_ATTR inputPinChangeISR(void *args) 
{
    inputInterruptCounter++;
    int inputIndex = (int)args;
    xQueueSendFromISR(inputInterruptQueue, &inputIndex, NULL);
}

void IRAM_ATTR loop_Core0_InputPinHandlerTask(void * parameter) 
{
    logMessage("loop_Core0_InputPinHandlerTask running...\r\n");
    uint8_t inputIndex;
    while (runLoops) {
        if (xQueueReceive(inputInterruptQueue, &inputIndex, portMAX_DELAY))
        {
            const struct inputPin_Config_s *pin_config = &inputPinsConfig[inputIndex];
            
            /* Updates the fb.io struct with the GPIO register state from the inputPin_config struct data in Config.h */
            bool registerBitState = (bool)REG_GET_BIT(pin_config->register_address, pin_config->register_bit);
            if ((fb.io & pin_config->fb_input_mask) != registerBitState) {
                (registerBitState) ? fb.io |= pin_config->fb_input_mask : fb.io &= ~pin_config->fb_input_mask;
                inputPinInterruptFired[inputIndex]++;

                if (inputPinInterruptFired[inputIndex] >= 50) {
                    logMessage("WARNING: Excessive input changes. Index: %d, GPIO: %d, Name: '%s'. Changed %d times. Resetting counter\r\n", inputIndex, pin_config->gpio_number, pin_config->name, inputPinInterruptFired[inputIndex]);
                    inputPinInterruptFired[inputIndex] = 0;
                }

                xEventGroupSetBits(xEventStateChangeGroup, (ESPNOW_SEND_BIT)); // Send an async ESP-NOW packet as required

                if (!motorsMoving) // Only send a udp feedback pkt if motors are stationary. When moving the feedback is done in realtime as per the fb.io struct
                    xEventGroupSetBits(xEventUDPPacketStateGroup, (UDP_SEND_PACKET_BIT));
            }
           
        }
    }
    logMessage("Exiting loop_Core0_InputPinHandlerTask\r\n");
    vTaskDelete(NULL);
}

/*==================================================================*/

void IRAM_ATTR loop_Core0_CommandHandlerTask(void* parameter)
{
    logMessage("loop_Core0_CommandHandlerTask running...\r\n");

    esp_task_wdt_delete(NULL); /* Disable the Task Watchdog (not core WD) for this task as the servo thread in FastAccelStepper can interfere with the it. No impact as the UDP packet based watchdog with catch any, if any, hung conditions */

    const EventBits_t xBitsToWaitFor = (UDP_RECEIVE_PACKET_BIT);
    EventBits_t xEventGroupValue;
    
    while(runLoops) {
        if (machineEnabled) { /* Handle UDP packet watchdog first */
            if (millis() - ul_udptxrx_watchdog > 5000) {
                logMessage("UDP Packet Watchdog triggered. Resetting machine state\r\n");
                fb.control = 0;
                cmd.control = 0;
                commandHandler();
                outputHandler();
                ul_udptxrx_watchdog = millis();
            }
        } else {
            ul_udptxrx_watchdog = millis();
        }
        /* Waits 50ms for an event from UDP packet from onUDPRxPacketCallBack(). Delay is only a few ticks if RX multiple packets */
        xEventGroupValue  = xEventGroupWaitBits(xEventUDPPacketStateGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY); 

        if (( xEventGroupValue & ( UDP_RECEIVE_PACKET_BIT ) ) == ( UDP_RECEIVE_PACKET_BIT )) /* If UDP Packet RX bit is set */
        {          
            ul_udptxrx_watchdog = millis();
            
            commandHandler(); /* No need to process inputHandler() as they're interrupt driven and sent asynchronously in realtime on a separate task */
            
            if (machineEnabled) {
                outputHandler();
                
            } else {
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            //xEventGroupClearBits(xEventStateChangeGroup, UDP_RECEIVE_PACKET_BIT);
        }
        
        /* No need for a vTaskDelay or yield context switch here since the event group delay above handles it */
    }

    logMessage("Exiting loop_Core0_CommandHandlerTask\r\n");
    vTaskDelete( NULL );
}

/*==================================================================*/

void loop_Core1_EspNowSenderTask(void* parameter) /* No need for IRAM since low speed background task */
{
    logMessage("loop_Core1_EspNowSenderTask running...\r\n");
    const EventBits_t xBitsToWaitFor = (ESPNOW_SEND_BIT);
    EventBits_t xEventGroupValue;  
    TickType_t xTicksToWait = 1000 / portTICK_PERIOD_MS; // Wait 1000ms

    while (runLoops) {
        if (espnow_peer_configured) {
            if (motorsMoving)
                xTicksToWait = 200 / portTICK_PERIOD_MS; // 200ms packet interval
            else
                xTicksToWait = 1000 / portTICK_PERIOD_MS; // 1 second packet interval
            
            xEventGroupValue  = xEventGroupWaitBits(xEventStateChangeGroup, xBitsToWaitFor, pdTRUE, pdTRUE, xTicksToWait);

            if (( xEventGroupValue & ( ESPNOW_SEND_BIT ) ) == ( ESPNOW_SEND_BIT )) /* If ESPNOW Send bit is set */
            {
                esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *) &fb, sizeof(fb)); // Send a current fb struct
            
                if (result == ESP_OK) {
                    espnowTxPackets++;
                } else {
                    logMessage("ESP-NOW: Error sending data to peer\r\n");
                }
                
                xEventGroupClearBits(xEventStateChangeGroup, ESPNOW_SEND_BIT);

            } else { // Send packet every 1000ms regardless
                
                /* Example struct that can be sent */

                // strcpy(espnowData.a, "THIS IS A CHAR");
                // espnowData.b = random(1,20);
                // espnowData.c = 1.2;
                // espnowData.d = false;
                //esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *) &espnowData, sizeof(espnowData)); // Example struct to send

                esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *) &fb, sizeof(fb)); // Send a current fb struct
            
                if (result == ESP_OK) {
                    espnowTxPackets++;
                } else {
                    logMessage("ESP-NOW: Error sending data to peer\r\n");
                }
                
            }

            vTaskDelay(1); /* Yield 1ms */

        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS); /* Yield 1s */
        }
        
    }
    
    logMessage("Exiting loop_Core1_EspNowSenderTask\r\n");
    vTaskDelete(NULL);
}

void IRAM_ATTR loop_Core1_ServoStatsTask(void* parameter) /* No need for IRAM since low speed background task */
{
    logMessage("loop_Core1_ServoStatsTask running...\r\n");

    while (runLoops) {
        if (serialConsoleEnabled) {
            long now_ProfileStats = millis();
            if (now_ProfileStats - lastMsg_ProfileStats > 1000) {
                logMessage("UDP: TX PPS: %d, RX PPS: %d, TX Errors: %d, RX Errors: %d, EspNowTX: %d, InputIntrCtr: %d\r\n", udp_tx_seq_num, udp_rx_seq_num, udpPacketTxErrors, udpPacketRxErrors, espnowTxPackets, inputInterruptCounter);
                udp_tx_seq_num = 0;
                udp_rx_seq_num = 0;
                inputInterruptCounter = 0;

                if (motorsSetup) {                
                    bool printStepperStats = false;
                    for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
                        if (stepper[i]->isRampGeneratorActive()) {
                            printStepperStats = true;
                            logMessage("Stepper[%d]: Pos Diff: %d, Freq Hz: %d, ", i, (cmd.pos[i] - fb.pos[i]), stepper[i]->getCurrentSpeedInMilliHz()/1000);
                        }
                    }
                    if (printStepperStats)
                        logMessage("\r\n");
                }
                
                lastMsg_ProfileStats = now_ProfileStats;
            }
        }

        if (debugAxisMovements) {
            uint8_t axisNum;
            if (xQueueReceive(axisStateInterruptQueue, &axisNum, 0))
            {
                debugAxisState(axisNum);
            }
            vTaskDelay(1); // No need to delay the task for 1000ms
        } else {
            vTaskDelay(800 / portTICK_PERIOD_MS); // Task Sleep 1000ms
        }
    }

    logMessage("Exiting loop_Core1_ServoStatsTask\r\n");
    vTaskDelete(NULL);

}


/*==================================================================*/

#ifdef DEBUG_AXIS_MOVEMENTS
void debugAxisState(uint8_t axisNum) {

    if (!debugAxisMovements)
        return;
    
    bool bitVals[8] = {false};

    for (uint8_t i = 0; i < CONF_NUM_STEPPERS; i++)
    {
        /* Quick and dirty for now... */
        for (uint8_t j = 0; j < 8; j++)
        {
            bitVals[j] = bitRead(axisState[i],j);
        }
        char active = (i==axisNum) ? '*' : '-';
        char req_move = (bitVals[0]) ? 'R' : 'S';
        char req_move_dir = (bitVals[1]) ? 'F' : 'R';
        char req_accel = (bitVals[2]) ? 'A' : 'D';
        char req_coast = (bitVals[3]) ? 'C' : '-';
        char actual_dir = (bitVals[4]) ? 'F' : 'R';
        char actual_accel = (bitVals[5]) ? 'A' : (bitVals[6]) ? 'D' : '-';
        char actual_coast = (bitVals[7]) ? 'C' : '-';

        /* 
            Prints out to bottom of serial console as axis stats are changing in real-time.
            See AXIS_STATE_* defines in Types

             <----  Active Axis Moving - [*] active, [-] not active
                <---- Axis Index
                   <---- Request stationary or running - [S] stopped, [R] running
                      <---- Request movement direction - [F] Forward, [R] Reverse
                        <---- Actual movement direction - [F] Forward, [R] Reverse
                           <---- Request acceleration - [A] Accelerate, [D] Decelerate
                             <---- Actual current ramp state - [A] Accelerate, [D] Decelerate
                                <---- Request velocity coasting - [C] Coasting/at full Speed, [-] Not coasting/at full speed
                                   <---- Actual velocity coasting - [C] Coasting/at full Speed, [-] Not coasting/at full speed
            [-][0] S: R|R, D|-, -|-
            [*][1] S: R|R, D|-, -|-
            [-][2] S: R|R, D|-, -|-
            
        */
        
        logMotorDebugMessage(i+1,"[%c][%d] %c: %c|%c, %c|%c, %c|%c", active, i, req_move, req_move_dir,actual_dir, req_accel,actual_accel, req_coast, actual_coast);
    }
    
}

void IRAM_ATTR updateAxisState(uint8_t axisNum, uint8_t mask ) {    
    axisState[axisNum] = (mask);
    if (axisState[axisNum] != prev_axisState[axisNum]) {
        if (debugAxisMovements)
            xQueueSendFromISR(axisStateInterruptQueue, &axisNum, NULL);    
        prev_axisState[axisNum] = axisState[axisNum];
    } 
}
void IRAM_ATTR updateAxisState(uint8_t axisNum, uint8_t bit, bool new_value ) {   
    if (bit == AXIS_STATE_STOPPED && new_value == 1)
    {
        axisState[axisNum] = 0;
        if (debugAxisMovements)
            xQueueSendFromISR(axisStateInterruptQueue, &axisNum, NULL);
        return;
    }

    if (new_value)
        axisState[axisNum] |= (bit);
    else
        axisState[axisNum] &= ~(bit);
    
    if (axisState[axisNum] != prev_axisState[axisNum]) {
        if (debugAxisMovements)
            xQueueSendFromISR(axisStateInterruptQueue, &axisNum, NULL);
        prev_axisState[axisNum] = axisState[axisNum];
    }    
}
#else
void debugAxisState(uint8_t currState) { }
void updateAxisState(uint8_t axisNum, uint8_t bit, bool new_value ) { }
#endif


/* FastAccellStepper Motor API movement command & feedback of position, accel and speed handled in single hardware timer. Non-blocking calls. Debug optional (slows cmd.pos queue fractionally )*/
void IRAM_ATTR ServoMovementCmds_ISR() 
{
    if (machineEnabled) {
        bool isMovementRunning = false;
        uint8_t moveResult = 0;
        
        /* 
            Uses the FastAccelStepper moveTo() command to ensure that any missed or out of sync UDP packets the position is always handled correctly. 
            Position is a 32bit integer of motor step position and not pulses and is accurate.
        */
        
        for (uint8_t i = 0; i < CONF_NUM_STEPPERS; i++) {
            uint32_t newVel = abs(cmd.vel[i]); // cmd.vel in milliHz positive int abs value
            uint32_t velLimit = 0;
            
            if (newVel > 0) { // Move is wanted (always a positive value regardless of direction)               
                velLimit = (abs(cmd.vel_limit[i])) / axisVelScaleFactor; // cmd.vel_limit in milliHz positive int abs value. Reduce by axisScalingFactor percent as per above
                
                const bool moveDir = cmd.vel[i] > 0 ? 1 : 0; // cmd.vel signed. Negative = reverse movement. Positive = forward movement.
                
                bool isRampActive = stepper[i]->isRampGeneratorActive();
                if (!isRampActive && newVel > 0) { // Initial move request ramping (stationary -> moving)
                    updateAxisState(i, AXIS_STATE_MOVE_REQ, 1);
                    (moveDir) ? updateAxisState(i, AXIS_STATE_MOVE_REQ_DIR, 1) : updateAxisState(i, AXIS_STATE_MOVE_REQ_DIR, 0);
                    stepper[i]->setLinearAcceleration(2000); // Initial linear acceleration but reduced to 100 once hits coasting speed
                    stepper[i]->setSpeedInMilliHz(newVel / axisVelScaleFactor); // Set to current axis velocity limit initially
                    updateAxisState(i, AXIS_STATE_MOVE_ACCEL_REQ, 1);
                } else if (isRampActive) { // rampGenerator is active so is already moving
                        newVel = newVel / axisVelScaleFactor;
                        const int32_t velDiff = (newVel - (abs(stepper[i]->getCurrentSpeedInMilliHz(true) / axisVelScaleFactor)));
                        if (velDiff > 10000 ) { // accelerating
                            stepper[i]->setLinearAcceleration(0); // Initial linear acceleration but reduced to 100 once hits coasting speed
                            stepper[i]->setSpeedInMilliHz(newVel); // Repeated call whilst moving to ensure changes in velocity is tracked accordingly.
                            updateAxisState(i, AXIS_STATE_MOVE_ACCEL_REQ, 1);
                        } else if (velDiff > -10000 ) { // decelerating but ignore anything less than -1000mHz
                            stepper[i]->setLinearAcceleration(0); // Initial linear acceleration but reduced to 100 once hits coasting speed
                            stepper[i]->setSpeedInMilliHz(newVel); // Repeated call whilst moving to ensure changes in velocity is tracked accordingly.
                            updateAxisState(i, AXIS_STATE_MOVE_ACCEL_REQ, 0);
                        } 
                        /*  Must ignore cmd.velocity state less than threshold, especially when decelerating downwards towards -5khz or below - 
                            this maintains existing vel value from the previous command to ensure motor stops in time without lag (UDP latencies)
                         */
                    
                }
                /* applySpeedAcceleration() not required. Performed during moveTo call */
                moveResult = stepper[i]->moveTo(cmd.pos[i], false); // Important non-blocking call, not in IRAM. cmd.pos includes LinuxCNC scale multiplier from HAL component to ensure correct steps per unit
            }

            /* For safety always check within loop to update a stationary axis regardless if moveTo is called or not */

            if (moveResult != MOVE_OK) {
                /* 
                    A failed moveTo() call will set the feedback position to the FastAccelStepper current motor position so it will throw a follower error in LinuxCNC if the library fails to generate any step pulses.
                    Tested & confirmed realtime position works using a physical machine with units and axis motor scaling set correctly. 
                    There are no steps lost during any jog steps, MDI commands or whilst running a job with adjusted feedrates
                    The following return codes can be used and added later for completeness sake
                    MOVE_ERR_NO_DIRECTION_PIN
                    MOVE_ERR_SPEED_IS_UNDEFINED
                    MOVE_ERR_ACCELERATION_IS_UNDEFINED
                */
                logMessage("Move Error %d for axis %d\r\n", moveResult, 0); // TODO WARNING Repeated calls to this could cause a crash
                fb.pos[i] = stepper[i]->getCurrentPosition();
                fb.vel[i] = stepper[i]->getCurrentSpeedInMilliHz(false);

            } else { // Moving success
                /*
                    See https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md#stepper-position
                */
                
                if (stepper[i]->isRampGeneratorActive()) { /* Moving (fastest atomic call to the FAS lib to check if axis is moving) */
                    isMovementRunning = true;
                    
                    fb.pos[i] = stepper[i]->targetPos(); // Set cmd.pos to targetPos since it's where it will be after this movement in queue is complete
                    
                    uint8_t rampState = stepper[i]->rampState();
                    
                    if (rampState != prevRampState[i]) {
                        if (rampState & RAMP_STATE_ACCELERATE) { // Motor is accelerating
                            updateAxisState(i, (AXIS_STATE_COASTING | AXIS_STATE_DECELERATING), 0);
                            updateAxisState(i, AXIS_STATE_ACCELERATING, 1);
                        } else if ((rampState & RAMP_STATE_COAST)) { // Motor is coasting/at-speed
                            updateAxisState(i, (AXIS_STATE_DECELERATING | AXIS_STATE_ACCELERATING | AXIS_STATE_MOVE_ACCEL_REQ), 0);
                            updateAxisState(i, AXIS_STATE_COASTING, 1);
                            stepper[i]->setLinearAcceleration(100);
                            stepper[i]->applySpeedAcceleration();
                        } else if ((rampState & RAMP_STATE_DECELERATE)) { // Motor is decelerating
                            updateAxisState(i, (AXIS_STATE_ACCELERATING | AXIS_STATE_COASTING), 0);
                            updateAxisState(i, AXIS_STATE_DECELERATING, 1);
                        }
                        if ((rampState & RAMP_DIRECTION_COUNT_UP)) { // Motor is moving forwards
                            updateAxisState(i, AXIS_STATE_MOVING_DIR, 1);
                        } else if ((rampState & RAMP_DIRECTION_COUNT_DOWN)) { // Motor is moving backwards
                            updateAxisState(i, AXIS_STATE_MOVING_DIR, 0);
                        }
                        prevRampState[i] = rampState; // update rampState cache to reduce calls during loops
                    }

                    fb.vel[i] = stepper[i]->getCurrentSpeedInMilliHz(true) * axisVelScaleFactor; // Update realtime feedback velocity using scaled value as the above. Ensuring a match with LinuxCNC is expecting for PID control aspects
                    
                } else { /* Motor stationary */
                    fb.pos[i] = stepper[i]->getCurrentPosition();
                    stepper[i]->setSpeedInMilliHz(0);
                    stepper[i]->applySpeedAcceleration();
                    fb.vel[i] = 0;
                    if (axisState[i] != 0) {
                        updateAxisState(i, AXIS_STATE_STOPPED, 1);
                    }
                }
            }
        }
        if (motorsMoving != isMovementRunning)
            xEventGroupSetBits(xEventStateChangeGroup, (ESPNOW_SEND_BIT)); // Send an async ESP-NOW packet to ensure motorsMoving is detected
        
        motorsMoving = isMovementRunning; /* Set globally - if ANY axes are moving */
    } else {
        motorsMoving = false; /* Set globally - Machine is currently disabled */
    }
}

/*==================================================================*/
/*================= Setup sections =================================*/

void setupInputPin(int inputIndex) /* See Config.h for inputPin board specific configurations */
{
    const struct inputPin_Config_s *pin_config = &inputPinsConfig[inputIndex];
    if (pin_config->gpio_number != GPIO_NUM_NC) {
        logMessage("INPUT[%d]: Configuring: '%s', GPIO: %d\r\n", inputIndex, pin_config->name, pin_config->gpio_number);
        pinMode(pin_config->gpio_number, pin_config->pin_mode);
        /* Use ESP-IDF gpio isr attachment as it allows to pass an argument to the ISR function (inputIndex)*/
        gpio_set_intr_type(pin_config->gpio_number, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(pin_config->gpio_number, inputPinChangeISR, (void*)inputIndex);
    } else {
        logMessage("INPUT[%d] Config GPIO unconfigured\r\n", inputIndex);    
    }
}

void setup_Core0(void* parameter) /* Anything that wants to run on Core0 should be setup in this one-shot task */
{
    logMessage("setup_Core0()\r\n");

    //xEventUdpReceiveGroup = xEventGroupCreate(); // Create event group
    inputInterruptQueue = xQueueCreate(30, sizeof(uint8_t)); // Create queue here

    logMessage("Starting Ethernet...\r\n");
#ifdef ESP32_RMII_ETHERNET
    
    #ifdef ARDUINO_ESP32_WT32_ETH01
        ETH.begin(1,16,23,18,ETH_PHY_LAN8720, ETH_CLOCK_GPIO0_IN, false);
    #elif defined(ARDUINO_ESP32_POE)
        ETH.begin(0,12,23,18,ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT, false);
    #elif defined(ARDUINO_ESP32_EVB)
        ETH.begin(0,-1,23,18,ETH_PHY_LAN8720, ETH_CLOCK_GPIO0_IN, false);
    #endif
    ETH.config(my_ip, gw, subnetmask);

#elif defined(ESP32SX_USB_ETHERNET)
#elif defined(ESP32_SPI_ETHERNET) 

    ESP32_W5500_onEvent();

    bool eth_result = ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, W5500_CS_PIN, W5500_INT_PIN, SPI_CLOCK_MHZ, ETH_SPI_HOST, (uint8_t*) mac );

    if (!eth_result)
        logMessage("**ERROR** SPI Ethernet could not be initialised. Please check wiring.\r\n");
    else
        ETH.config(my_ip, gw, subnetmask);
    
#endif
    delay(200);

    logMessage("Starting Telnet server on port 23\r\n");
    telnetServer.onClient(onTelnetClient, NULL);
    telnetServer.begin();
        
    if(udpServer.listen(my_ip, udpServerPort)) {
        logMessage("UDP Server Listening on IP: %s, Port: %d\r\n", my_ip.toString().c_str(), udpServerPort);
        udpServer.onPacket(onUDPRxPacketCallBack);
    } else {
        logMessage("**ERROR**: UDP Server failed to listen on IP: %s, Port: %d\r\n", my_ip.toString().c_str(), udpServerPort);
    }

    
    

    xTaskCreatePinnedToCore(
        loop_Core0_UDPSendTask, // Task function.
        "loop_Core0_UDPSendTask", // name of task.
        2048, // Stack size of task
        NULL, // parameter of the task
        6, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0

    xTaskCreatePinnedToCore(
        loop_Core0_InputPinHandlerTask, // Task function.
        "loop_Core0_InputPinHandlerTask", // name of task.
        2048, // Stack size of task
        NULL, // parameter of the task
        4, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0

    if (motorsSetup) {
        xTaskCreatePinnedToCore(
            loop_Core0_CommandHandlerTask, // Task function.
            "loop_Core0_CommandHandlerTask", // name of task.
            4096, // Stack size of task
            NULL, // parameter of the task
            5, // priority of the task
            NULL, // Task handle to keep track of created task
            0); // pin task to core 0
    }

    logMessage("Attaching hw timer for ServoMovementCmds ISR\r\n");
    
    timerServoCmds = timerBegin(3, 80, true);
    timerAttachInterrupt(timerServoCmds, &ServoMovementCmds_ISR, true);
    timerAlarmWrite(timerServoCmds, 500, true); // Runs faster than the pre-compiled 1khz FreeRTOS tick to ensure no movement command buffering issues
    timerAlarmEnable(timerServoCmds);  

    /* Must be done after Ethernet due to SPI interrupt global isr service install */
    logMessage("Configuring inputs...\r\n");
    
    for (uint8_t i = 0; i < MAX_INPUTS; i++) {
        setupInputPin(i);
    }

    logMessage("Core0 task setup complete\r\n");
    serialConsoleEnabled = true;
    vTaskDelete(NULL);
}

/*==================================================================*/

void stopProcessing() 
{
    if (runLoops) {
        logMessage("Stop Processing!\r\n");
        runLoops = false;
        for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
            stepper[i]->forceStop();
            stepper[i]->disableOutputs();
        }
        timerDetachInterrupt(timerServoCmds);
        timerStop(timerServoCmds);
        
        udpClient.close();
        udpServer.close();
        for (int i = 0; i < MAX_INPUTS; i++) {
            const struct inputPin_Config_s *pin_config = &inputPinsConfig[i];
            if (pin_config->gpio_number != GPIO_NUM_NC)
                detachInterrupt(pin_config->gpio_number);
        }

    } else {
        logMessage("Already stopped\r\n");
    }

}

void otaUpdateStart() 
{
    serialConsoleEnabled = true;
    logMessage("OTA Update Starting....\r\n");
    disableCore0WDT();
    disableCore1WDT();
    stopProcessing(); 
    
    logMessage("\r\n");
}

void otaUpdateEnd() 
{
    logMessage("OTA Update completed\r\n");
}

void otaProgress(unsigned int progress, unsigned int total) 
{
    logMessage("OTA Update Progress: %u%%\r", (progress / (total / 100)));
}

void parseTelnetMessage(AsyncClient* c, String message)
{   
    if (message.equals("r\r\n")) {
        logMessage("R key pressed. Restart..\r\n");
        telnetDisconnectAllClients();
        ESP.restart();
    } else if (message.equals("s\r\n")) {
        logMessage("S key pressed\r\n");
        stopProcessing();
    }   else if (message.equals("w\r\n")) {
        logMessage("W key pressed\r\n");
        if (WiFi.getMode() == WIFI_OFF) {
            setWifiState(WIFI_MODE_STA);
        } else {
            setWifiState(WIFI_OFF);
        }
    }   else if (message.equals("d\r\n")) {
        serialConsoleEnabled = !serialConsoleEnabled;
        logMessage("Serial Console: %d", serialConsoleEnabled);
    }
    
}

void telnetDisconnectAllClients() 
{
    if (telnetClientConnected > 0) {
        telnetClient->close();
        telnetClient = NULL;
        if (telnetClientConnected > 0)
            telnetClientConnected--;
    }
}

void onTelnetClientData(void *s, AsyncClient* c, void *buf, size_t len)
{
    char *str = static_cast<char*>(buf);
    if (len == 5) {
        if (memcmp(str, ctrlz_bytes, sizeof(ctrlz_bytes)) == 0)
        {
            telnetDisconnectAllClients();
        }
    }
    
    String rxMessage(str);
    parseTelnetMessage(c, rxMessage);
}

void onTelnetClientDisconnected(void *s, AsyncClient* c)
{
    if (telnetClientConnected > 0)
        telnetClientConnected--;
    telnetClient = NULL;
    logMessage("Telnet: client disconnected\r\n");
}

void onTelnetClient(void *s, AsyncClient* c) 
{
    telnetClientConnected++;
    telnetClient = c;
    c->onDisconnect(onTelnetClientDisconnected);
    c->onData(onTelnetClientData);
    IPAddress client_address = c->getRemoteAddress();
    logMessage("Telnet: '%s' client connected\r\n", client_address.toString().c_str() );
    String welcomeMsg = String("Welcome. Version: " + version_number + ". Ctrl+Z to quit\r\n");
    c->write("\eSP F");  // tell to use 7-bit control codes
    c->write("\e[?25l"); // hide cursor
    c->write("\e[?12l"); // disable cursor highlighting
    c->write(welcomeMsg.c_str());
}

/*==================================================================*/

////////////////////////////////////////////////////////
#ifdef ARDUINO_ESP32_MKS_DLC32
bool fasExternalCallForPin(uint8_t pin, uint8_t value) {
  // This example returns the previous value of the output.
  // Consequently, FastAccelStepper needs to call setExternalPin twice
  // in order to successfully change the output value.
  bool oldValue = digitalRead(pin);
  digitalWrite(pin, value);
  return oldValue;
}
#endif

bool setupMotors() 
{
    bool result = true;
    
    logMessage("Configuring motors\r\n");

    for (int i = 0; i < CONF_NUM_STEPPERS; i++) {
        cmd.vel[i] = 0;
        fb.vel[i] = 0;
    }
    
    stepperEngine.init(0); /* Use Core0 for stepper engine */
#ifdef ARDUINO_ESP32_MKS_DLC32
    stepperEngine.setExternalCallForPin(fasExternalCallForPin);
#endif
    for (uint8_t i = 0; i < CONF_NUM_STEPPERS; i++) {
        logMessage("Creating stepper %d\r\n", i);
        FastAccelStepper *s = NULL;
        const struct stepper_config_s *config = &stepper_config[i];
        if (config->step != PIN_UNDEFINED) {
            logMessage("configuring stepper %d\r\n", i);
#if CONFIG_IDF_TARGET_ESP32S2
            s = stepperEngine.stepperConnectToPin(config->step);
#else
            s = stepperEngine.stepperConnectToPin(config->step, DRIVER_RMT);
#endif
            if (s) {
                logMessage("attached stepper %d\r\n", i);
                s->setDirectionPin(config->direction, config->direction_high_count_up,
                                config->dir_change_delay);
                s->setEnablePin(config->enable_low_active, true);
                s->setEnablePin(config->enable_high_active, false);
                s->setAutoEnable(config->auto_enable);
                s->setDelayToEnable(config->on_delay_us);
                s->setDelayToDisable(config->off_delay_ms);
                s->setSpeedInHz(1000); /* Low inital values for safety */
                s->setAcceleration(1000); /* Low inital values for safety */
                s->applySpeedAcceleration();
                s->disableOutputs();
                
            } else {
                logMessage("**ERROR** Attaching stepper %d to RMT Driver\r\n", i);
                result = false;
            }
        }
        stepper[i] = s;
        logMessage("stepper %d created successfully\r\n", i);
    }
    return result;
}

void setWifiState(bool newState) {
    logMessage("Setting WiFi state: %d\r\n", newState);
    if (newState) {
        WiFi.mode(WIFI_MODE_STA); 
        WiFi.begin();
    }
    else
        WiFi.mode(WIFI_OFF); 
    
        
}

void setup()
{
    serialConsoleEnabled = true;
    //esp_log_level_set("*", ESP_LOG_VERBOSE);
#ifndef ESP32_SPI_ETHERNET // Only do this once and before everything if not using SPI ethernet. SPI ethernet driver installs this automatically
    gpio_install_isr_service(0);
#endif
    
    Serial.begin(BAUD_RATE);
    Serial.setDebugOutput(false);
    Serial.print("\eSP F");  // tell to use 7-bit control codes
    Serial.print("\e[?25l"); // hide cursor
    Serial.print("\e[?12l"); // disable cursor highlighting
    Serial.println();
    logMessage("Version: %s\r\n", version_number);
    logMessage("Setup started..\r\n");
    logMessage("APBFreq: %d\r\n", getApbFrequency());

    if (!setupMotors()){
        logMessage("**Error** setting up motors. Check motor pin mappings, input pin interrupt assignments and output pins do not overlap any motor pins\r\n");
        motorsSetup = false;
    } else {
        motorsSetup = true;
    }

    WiFi.onEvent(WiFiEvent); /* Always map networking event handler irrespective of WiFi enabled/disabled */

#ifdef ENABLE_WIFI
    /* WiFi Enabled */
#ifdef WIFI_CLIENTMODE
        logMessage("Connecting to WiFi SSID: %s\r\n", CONF_WIFI_SSID);
        WiFi.mode(WIFI_MODE_APSTA);
        WiFi.begin(CONF_WIFI_SSID, CONF_WIFI_PWD);
#endif
#ifdef WIFI_ACCESSPOINTMODE
        WiFi.mode(WIFI_MODE_APSTA);
        //WiFi.begin(CONF_WIFI_SSID, CONF_WIFI_PWD);
        WiFi.softAP(CONF_WIFI_SSID, CONF_WIFI_PWD,0,0,4); // WiFi needs to be enabled 
#endif
    
        logMessage("WiFi MAC: %s\r\n", WiFi.macAddress().c_str());
        
#ifdef WIFI_CLIENTMODE
        int connectCount = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            connectCount++;
            if (connectCount > 10) {
                logMessage("Timeout for 5s whilst connecting to wifi network. Skipping...\r\n");
                break;
            }
        }
#endif
        // Init ESP-NOW
        if (esp_now_init() != ESP_OK) {
            logMessage("Error initializing ESP-NOW\r\n");
        }

        // Once ESPNow is successfully Init, we will register for recv CB to
        // get recv packer info
        esp_now_register_recv_cb(espnowOnDataRecv);

        // Once ESPNow is successfully Init, we will register for Send CB to
        // get the status of Transmitted packet
        //esp_now_register_send_cb(espnowOnDataSent);
  
        
#else
    /* WiFi Disabled */
    WiFi.softAP("NULL","NULL123465",0,1,1); // WiFi needs to be enabled then disabled after Arduino OTA has been setup (lwip etc)
#endif
    
    ArduinoOTA.begin();  /* Enable OTA Service */
    ArduinoOTA.onStart(otaUpdateStart);
    ArduinoOTA.onEnd(otaUpdateEnd);
    ArduinoOTA.onProgress(otaProgress);
    
    //WiFi.mode(WIFI_OFF); /* shutdown wifi */

#ifndef ENABLE_WIFI
    /* WiFi Disabled */
    WiFi.disconnect(true,false);  
    WiFi.mode(WIFI_OFF); /* shutdown wifi */
#endif
           

#ifdef ARDUINO_ESP32_MKS_DLC32
    i2s_out_init();
    //i2s_out_set_pulse_callback(stepper_pulse_func);
#endif
    axisStateInterruptQueue = xQueueCreate(30, sizeof(uint8_t)); // Create queue here

    runLoops = true; // Important to ensure background tasks exec in a loop

    
    xTaskCreatePinnedToCore(
        setup_Core0, // Task function.
        "setup_Core0Task", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        tskIDLE_PRIORITY, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0

#ifdef ENABLE_SERIAL_STATS
    /* For Debugging purposes - prints to serial port motor stats and UDP packet counters - Disable as required */
    xTaskCreatePinnedToCore(
        loop_Core1_ServoStatsTask, // Task function.
        "loop_Core1_ServoStatsTask", // name of task.
        2048, // Stack size of task
        NULL, // parameter of the task
        2, // priority of the task
        NULL, // Task handle to keep track of created task
        1); // pin task to core 1
#endif

    xTaskCreatePinnedToCore(
        loop_Core1_EspNowSenderTask, // Task function.
        "loop_Core1_EspNowSenderTask", // name of task.
        2048, // Stack size of task
        NULL, // parameter of the task
        3, // priority of the task
        NULL, // Task handle to keep track of created task
        1); // pin task to core 1

    logMessage("Setup completed successfully\r\n");
}


/*==================================================================*/
void loop()
{
    if (eth_connected || WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();  // Handles a code update request
    }
#ifdef ENABLE_SERIAL_STATS
    /* Basic spinner on serial out to show MCU is still active */
    if (runLoops && serialConsoleEnabled) {
        if (spinProgress == 1) {
            spinChar = '\\';
        } else if (spinProgress == 2) {        
            spinChar = '|';
        } else if (spinProgress == 3) {
            spinChar = '/';
        } else {
            spinProgress = 0;
            spinChar = '-';
        }
        
        Serial.printf("\e[F\e[4A\e[0G%c\e[K\r\n\e[4B\e", spinChar);
        
        spinProgress++;
    }
    
#endif
    if (Serial.available() > 0) {
        byte incomingByte = Serial.read();
        if ((char) incomingByte == 'r') {
            logMessage("R key pressed. Restart..\r\n");
            telnetDisconnectAllClients();
            ESP.restart();
        } else if ((char) incomingByte == 's') {
            logMessage("S key pressed\r\n");
            stopProcessing();
        } else if ((char) incomingByte == 'w') {
            logMessage("W key pressed\r\n");
            if (WiFi.getMode() == WIFI_OFF) {
                setWifiState(WIFI_MODE_STA);
            } else {
                setWifiState(WIFI_OFF);
            }
        } else if ((char) incomingByte == 'm') {
            debugAxisMovements = !debugAxisMovements;
            logMessage("Debug Axis Movements: %d", debugAxisMovements);
        } else if ((char) incomingByte == 'd') {
            serialConsoleEnabled = !serialConsoleEnabled;
            logMessage("Serial Console: %d", serialConsoleEnabled);
        }
    
    

    }
 
    delay(250);
   
}
