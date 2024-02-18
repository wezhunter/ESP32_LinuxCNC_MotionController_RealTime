#include <Arduino.h>
#include "ClientMode.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>


void setupClientMode()
{
    xEventStateChangeGroupClient = xEventGroupCreate();
    if (configWifiMode == WIFI_MODE_APSTA) {
        logMessage("ESP-NOW Setup");
        // Init ESP-NOW
        if (esp_now_init() != ESP_OK) {
            logErrorMessage("Unable to initialise ESP-NOW");
        } else {
            // Once ESPNow is successfully Init, we will register for recv CB to
            // get recv packer info
            esp_now_register_send_cb(espNowOnDataSentClientMode);
            esp_now_register_recv_cb(espNowOnDataRecvClientMode);
            logMessage("ESP-NOW setup completed successfully");
        }
    }
    xTaskCreatePinnedToCore(
        loop_Core0_EspNowSenderClientTask, // Task function.
        "t_C0_EspNowClnt", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        1, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0
    if (!configEspNowEnabled) {
        logWarningMessage("ESP-NOW client is currently disabled in config");
    }

    ESP32Encoder::useInternalWeakPullResistors = puType::UP;
    encoder.attachHalfQuad(32, 25);
    encoderTimer.attach_ms(10, checkEncoder);

    logMessage("Client mode setup complete");
}

void stopProcessingClient()
{
    if (configEspNowEnabled) {
        xEventGroupSetBits(xEventStateChangeGroupClient, ESPNOW_SEND_BIT); // To wake any sleeping tasks to stop on next loop
    }
}

// callback when esp-now data is sent including status
void espNowOnDataSentClientMode(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  if (status != ESP_NOW_SEND_SUCCESS) {
    logErrorMessage("ESP-NOW Unable to send data to peer. Resetting configured flag");
    espnow_peer_configured = 0;
  }
}

// callback function that will be executed when new esp-now data is received
void espNowOnDataRecvClientMode(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    lastPacketRxTimeMs = millis();
    //Serial.printf("ESP-NOW RX: %d\r\n", len);
    espnow_peer_configured = true;
    if (len == sizeof(myData)) { // if rx example data struct
      memcpy(&myData, incomingData, sizeof(myData));

    } else if (len == sizeof(fb)) { // if rx feedback packet struct. Print it for testing
      memcpy(&fb, incomingData, sizeof(fb));
      printf("FB: Control: %d, IO: %d, UDPSeq: %d\r\n", fb.control, fb.io, fb.udp_seq_num);
      printf("FB: POS0: %d, POS1: %d, POS2: %d\r\n", fb.pos[0], fb.pos[1], fb.pos[2]);
      printf("FB: VEL0: %d, VEL1: %d, VEL2: %d\r\n", fb.vel[0], fb.vel[1], fb.vel[2]);
    }
}

void loop_Core0_EspNowSenderClientTask(void* parameter) 
{
    logMessage("loop_Core0_EspNowSenderClientTask running...\r\n");

    while (runLoops) {
      
      if (espnow_peer_configured == 0 && configEspNowEnabled) { // Register peer. Send the local wifi mac to the remote motion controller

            if (configRemotePeerAddress.length() == 0) {
                logErrorMessage("ESP-NOW: No peer MAC address is configured to connect to. See 'setcontrollermac' command");
                vTaskDelay(30000 / portTICK_PERIOD_MS); // Task Sleep 30s
                continue;
            }
            String strs[20];
            int stringCount = 0;
            String tmpAddr = String(configRemotePeerAddress);

            while (tmpAddr.length() > 0) {
                int index = tmpAddr.indexOf(':');
                if (index == -1) { // No : found
                    strs[stringCount++] = tmpAddr;
                    break;
                } else {
                    strs[stringCount++] = tmpAddr.substring(0, index);
                    tmpAddr = tmpAddr.substring(index+1);
                }
            }
            if (stringCount != 6) {
                logErrorMessage("ESP-NOW: Peer MAC address has wrong length please check config");
                espnow_peer_configured = 0;
                continue;
            }
            
            for (int i = 0; i < stringCount; i++)
            {
                char * p;
                remotePeerAddress[i] = strtol( strs[i].c_str(), &p, 16 );
            }
            

            memcpy(peerInfo.peer_addr, remotePeerAddress, 6);
            peerInfo.channel = 0;  
            peerInfo.encrypt = false;
            
            esp_now_del_peer(peerInfo.peer_addr); // clear old peers before adding new one
            
            // Add peer
            if (esp_now_add_peer(&peerInfo) != ESP_OK){
                logErrorMessage("Failed to add peer. Retry in 5s...");
                continue;
            }
            
            WiFi.macAddress(espnowAddPeerMsg.mac_adddress); // Get local wifi mac and store in esp-now peer struct

            printf("ESP-NOW: Sending AddPeerMessage with MAC: '%02X:%02X:%02X:%02X:%02X:%02X' to Controller MAC: '%02X:%02X:%02X:%02X:%02X:%02X'\r\n", 
                espnowAddPeerMsg.mac_adddress[0],espnowAddPeerMsg.mac_adddress[1],espnowAddPeerMsg.mac_adddress[2],
                espnowAddPeerMsg.mac_adddress[3],espnowAddPeerMsg.mac_adddress[4],espnowAddPeerMsg.mac_adddress[5],
                peerInfo.peer_addr[0],peerInfo.peer_addr[1],peerInfo.peer_addr[2],
                peerInfo.peer_addr[3],peerInfo.peer_addr[4],peerInfo.peer_addr[5]
            );

            esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t*) &espnowAddPeerMsg, sizeof(espnowAddPeerMsg)); // send esp-now addPeerMsg

            if (result == ESP_OK) {
                logMessage("Successfully sent an addPeerMessage to remote node");
                espnow_peer_configured++;
            }
            else {
                logErrorMessage("ERROR: Could not send an addPeerMessage to remote node. Retry in 5s...");
                espnow_peer_configured = 0;
            }
        
        } else { // esp-now peer is configured - send test data struct every 5s. Check if last rx packet >3s then mark as stale connection and retry adding peer every 5s
          if (!configEspNowEnabled) {
            vTaskDelay(100 / portTICK_PERIOD_MS); // Task Sleep 100mss
            continue;
          }
          long now_LastRxPktTime = millis();
          if (now_LastRxPktTime - lastPacketRxTimeMs > 3000) {
              logErrorMessage("Timeout expired receiving message from peer. Reconnecting...");
              espnow_peer_configured = 0;
          } else {
            esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t*) &myData, sizeof(myData));
            if (result != ESP_OK) {
              logErrorMessage("ERROR: Could not send an ping to remote node. Retry in 5s...");
              espnow_peer_configured = 0;
            }
          }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // Task Sleep 100ms
    }
    
    logMessage("Exiting loop_Core0_EspNowSenderClientTask\r\n");
    vTaskDelete(NULL);
}

void sendMPG()
{
    if (espnow_peer_configured > 0) { // Register peer. Send the local wifi mac to the remote motion controller
        esp_err_t result = esp_now_send(remotePeerAddress, (uint8_t*) &mpgData, sizeof(mpgData));
        if (result != ESP_OK) {
            logErrorMessage("ERROR: Could not send MPG update...");
            espnow_peer_configured = 0;
        }
    }
}

void checkEncoder()
{
  int cnt = encoder.getCount();
  if(cnt != encoderLastCnt){
    printf("\n\t %i \n",cnt);
    encoderLastCnt = cnt;
    // TODO, fix this, cast 64 bit int to 32
    mpgData.mpg1 = (int)encoder.getCount();
    sendMPG();
  }
}