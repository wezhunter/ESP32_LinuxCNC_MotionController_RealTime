
#include <Arduino.h>
#include "ControllerMode.h"



/*==================================================================*/



// ESP-NOW P2P WiFi callback when data is sent. Unused in controller usually
void espnowOnDataSentController(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
//  logMessage("\r\nLast Packet Send Status:\t");
//  if (status != ESP_NOW_SEND_SUCCESS) {
//    logMessage(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  }
}

// ESP-NOW P2P WiFi callback function that will be executed when data is received
void espNowOnDataRecvController(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    //logMessage("ESP-NOW: RX: %d", len);
    espNowOnDataRecvControllerMode(mac, incomingData, len);   
}

void setupControllerMode()
{
    if (configWifiMode == WIFI_MODE_APSTA) {
        logMessage("ESP-NOW Setup");
        if (esp_now_init() != ESP_OK) { // Init ESP-NOW
            logErrorMessage("Unable to initialise ESP-NOW");
        } else {
            // Once ESPNow is init, we will register for recv CB to get recv packer info
            esp_now_register_recv_cb(espNowOnDataRecvController);
            logMessage("ESP-NOW setup completed successfully");
        }
    }

    axisStateInterruptQueue = xQueueCreate(30, sizeof(uint8_t)); // Create queue here. Avoiding any stack alloc issues

    if (!setupMotors()){
        if (doMotorConfig)
            logErrorMessage("Configuring motors. Check motor pin mappings, input pin interrupt assignments and output pins do not overlap any motor pins");
        else
            logErrorMessage("Motors are NOT configured. Config using 'setmotor', 'saveconfig', 'enablemotorconfig on' commands. See 'help' or docs");
        motorsSetup = false;
    } else {
        logMessage("*** Motors configured successfully ****");
        motorsSetup = true;
    }

    xTaskCreatePinnedToCore(
        loop_Core1_EspNowSenderControllerTask, // Task function.
        "t_C1_EspNowCtrll", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        3, // priority of the task
        NULL, // Task handle to keep track of created task
        1); // pin task to core 1
    
    if (!configEspNowEnabled) {
        logWarningMessage("ESP-NOW server is currently disabled in config");
    }
    
   /* For Debugging purposes - prints to serial port motor stats and UDP packet counters - Disable as required */
    xTaskCreatePinnedToCore(
        loop_Core1_ServoStatsTask, // Task function.
        "t_C1_Stats", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        2, // priority of the task
        NULL, // Task handle to keep track of created task
        1); // pin task to core 1
}

void setupControllerModeCore0() 
{
    eventUDPPacketStateGroup = xEventGroupCreate();
    xEventStateChangeGroup = xEventGroupCreate();

    if(udpServer.listen(udpServerPort)) {
        logMessage("UDP Server Listening on Port: %d", udpServerPort);
        udpServer.onPacket(onUDPRxPacketCallBack);
    } else {
        logErrorMessage("UDP Server failed to listen on Port: %d", udpServerPort);
    }
    xTaskCreatePinnedToCore(
        loop_Core0_UDPSendTask, // Task function.
        "t_C0_UdpTx", // name of task.
        2048, // Stack size of task
        NULL, // parameter of the task
        19, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0

    if (motorsSetup) {
        xTaskCreatePinnedToCore(
            loop_Core0_CommandHandlerTask, // Task function.
            "t_C0_CmdHdlr", // name of task.
            4096, // Stack size of task
            NULL, // parameter of the task
            5, // priority of the task
            NULL, // Task handle to keep track of created task
            0); // pin task to core 0
    }
    logMessage("Attaching hw timer for ServoMovementCmds ISR");
    
    timerServoCmds = timerBegin(3, 80, true);
    timerAttachInterrupt(timerServoCmds, &ServoMovementCmds_ISR, true);
    timerAlarmWrite(timerServoCmds, 500, true); // Runs faster than the pre-compiled 1khz FreeRTOS tick to ensure no commanded movement buffering issues
    timerAlarmEnable(timerServoCmds);  
}

void stopProcessingController()
{
    xEventGroupSetBits(eventUDPPacketStateGroup, UDP_SEND_PACKET_BIT); // Send an async ESP-NOW packet as required
    delay(5);
    udpClient.close();
    udpServer.close();
    

    if (motorsSetup) {
        for (int i = 0; i < configNumSteppers; i++) {
                stepper[i]->forceStop();
                stepper[i]->disableOutputs();
            }
    }
    if (configEspNowEnabled) {
        xEventGroupSetBits(xEventStateChangeGroup, ESPNOW_SEND_BIT); // Send a bit to wake up thread
    }
    timerDetachInterrupt(timerServoCmds);
    timerStop(timerServoCmds);
    
}



void loop_Core1_EspNowSenderControllerTask(void* parameter) /* No IRAM */
{
    logMessage("loop_Core1_EspNowSenderControllerTask running...");
    const EventBits_t xBitsToWaitFor = (ESPNOW_SEND_BIT);
    EventBits_t xEventGroupValue;  
    TickType_t xTicksToWait = 1000 / portTICK_PERIOD_MS; // Wait 1000ms
    espnow_peer_configured = 0;

    while (runLoops) {
        if (espnow_peer_configured > 0 && configEspNowEnabled) {
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
                    logErrorMessage("ESP-NOW error sending data to peer");
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
                    logErrorMessage("ESP-NOW error sending data to peer, %d",espnow_peer_configured);
                }
                
            }

            vTaskDelay(1); /* Yield 1ms */

        } else { /* No Peer is configured or disabled in config so no need to run */
            vTaskDelay(100 / portTICK_PERIOD_MS); /* Yield 100mss */
        }
        
    }
    
    logMessage("Exiting loop_Core1_EspNowSenderControllerTask");
    vTaskDelete(NULL);
}

void espNowOnDataRecvControllerMode(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    if (len == sizeof(espnowAddPeerMsg)) {
        printf_P(PSTR("ESP-NOW: Add new peer mac message received\r\n"));
        memcpy(&espnowAddPeerMsg, incomingData, sizeof(espnowAddPeerMsg));
        printf("ESP-NOW: Adding peer MAC: '%02X:%02X:%02X:%02X:%02X:%02X'\r\n", 
            espnowAddPeerMsg.mac_adddress[0],espnowAddPeerMsg.mac_adddress[1],espnowAddPeerMsg.mac_adddress[2],espnowAddPeerMsg.mac_adddress[3],
            espnowAddPeerMsg.mac_adddress[4],espnowAddPeerMsg.mac_adddress[5],espnowAddPeerMsg.mac_adddress[6]);
        
        // Copy peer mac into peerInfo struct
        memcpy(peerInfo.peer_addr, espnowAddPeerMsg.mac_adddress, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
        
        esp_now_del_peer(peerInfo.peer_addr); // Delete peer mac first to remove dups (lazy but easiest way)

        // Add peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK){
            logErrorMessage("Failed to add ESP-NOW peer");
            espnow_peer_configured = 0;
            return;
        } else {
            printf_P(PSTR("********* Added ESP-NOW Peer **********\r\n"));
            espnow_peer_configured++;
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

void IRAM_ATTR commandHandler()
{
    if (cmd.control & CTRL_READY) {
        for (int i = 0; i < configNumSteppers; i++) { // Placeholder if anything needs to occur at 1khz RTOS tick relating to motors
        }
    }

    if (prev_cmd_control != cmd.control) {
        logMessage("cmd.control = 0x%02x", cmd.control);
        
        if (cmd.control & CTRL_ENABLE) { // Called when Machine turned ON in LinuxCNC (Ready + Enable bits)
            logMessage("CMD: Machine ON");
            fb.udp_seq_num = 0;
            
            for (int i = 0; i < configNumSteppers; i++) {
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
            logMessage("CMD: Machine OFF");
            
            motorsMoving = false;
            machineEnabled = false;
            fb.udp_seq_num = 0;
            
            for (int i = 0; i < configNumSteppers; i++) {
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
        for (int i = 0; i < configNumSteppers; i++) {
            stepper[i]->disableOutputs();
        }
        if ((fb.control & CTRL_DIRSETUP)
            && (fb.control & CTRL_ACCEL)
            && (fb.control & CTRL_RESTORE)
            && (fb.control & CTRL_PWMFREQ)) {
            fb.control |= CTRL_READY;
            logMessage("Setting FB Control Ready");
            

        } else if (cmd.control & CTRL_DIRSETUP) {
            fb.control |= CTRL_DIRSETUP;
        } else if (cmd.control & CTRL_ACCEL) {
            fb.control |= CTRL_ACCEL;
            
            logMessage("CTRL_ACCEL");
            

            for (int i = 0; i < configNumSteppers; i++) {
                int newAccel = cmd.pos[i] * 2.0;
                logMessage("Setting stepper[%d] Accel to: %d", i, newAccel);
                stepper[i]->setAcceleration(newAccel);
            }
        } else if (cmd.control & CTRL_RESTORE) {
            fb.control |= CTRL_RESTORE;
            logMessage("CTRL_RESTORE");
            for (int i = 0; i < configNumSteppers; i++) {
                stepper[i]->setCurrentPosition(cmd.pos[i]);
                fb.pos[i] = cmd.pos[i];
            }

        } else if (cmd.control & CTRL_PWMFREQ) {
            fb.control |= CTRL_PWMFREQ;
            logMessage("Setup output pins");

            for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
                const outputpin_config_t *pin_config = &board_pin_config.outputConfigs[i];
                if (pin_config->gpio_number != GPIO_NUM_NC) {
                    logMessage("Setup output [%d] Name: '%s'", i, pin_config->name.c_str());
                    if (cmd.pwm[i]) {
                        logMessage("Setup PWM output [%d]", i);
                        ledcAttachPin(pin_config->gpio_number, i * 2);
                        ledcSetup(i * 2, cmd.pwm[i], 10);
                        ledcWrite(i * 2, 0);
                        pwm_enable[i] = true;
                    } else {
                        pinMode(pin_config->gpio_number, OUTPUT);
                        digitalWrite(pin_config->gpio_number, 0);
                    }
                }
            }
        }
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
            xEventGroupSetBits(eventUDPPacketStateGroup, UDP_RECEIVE_PACKET_BIT);
        } else {
            udpPacketRxErrors++;
            if (udpPacketRxErrors > 1000) {
                logErrorMessage("Excessive UDP RX errors!. Size: %d, RxLen:%d", sizeof(cmd), packet.length());
                udpPacketRxErrors = 0;
            }
            return;
        }
        
        xEventGroupSetBits(eventUDPPacketStateGroup, UDP_SEND_PACKET_BIT); // Send an async ESP-NOW packet as required
    }
}

void IRAM_ATTR loop_Core0_UDPSendTask(void* parameter)
{
    logMessage("loop_Core0_UDPSendTask running...");
    
    logMessage("UDP Send connect to IP: %s, Port: %d, Result: %d", ethernet_ip_host.toString().c_str(), udpClientPort, udpClient.connect(ethernet_ip_host,udpClientPort));
    size_t sent_len = 0;
    const EventBits_t bitsToWaitFor = (UDP_SEND_PACKET_BIT);
    EventBits_t xEventGroupValue;
    //const TickType_t ticksToWait = 1 / portTICK_PERIOD_MS; // Wait 1ms
    
    while (runLoops) {
        if (!udpClient.connected()) {
            logMessage("UDP Client unable to connect. Retrying in 5s");
            udpClient.connect(ethernet_ip_host, udpClientPort);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        
        xEventGroupValue  = xEventGroupWaitBits(eventUDPPacketStateGroup, bitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY); 
        udpTxLoopCount++;
        if (( xEventGroupValue & ( UDP_SEND_PACKET_BIT ) ) == ( UDP_SEND_PACKET_BIT )) /* If UDP Packet TX bit is set */
        {
            sendUDPFeedbackPacket();
        }
        /* vTaskDelay or yield context switch not required as the event group delay above handles it */
    }
    
    logMessage("Exiting loop_Core0_UDPSendTask");
    vTaskDelete(NULL);
}


void IRAM_ATTR loop_Core0_CommandHandlerTask(void* parameter)
{
    logMessage("loop_Core0_CommandHandlerTask running...");

    esp_task_wdt_delete(NULL); /* Disable Task Watchdog (not core WD) for this task as the servo thread in FastAccelStepper can interfere. No impact as the UDP packet watchdog should catch a stalled instance, if any */

    const EventBits_t xBitsToWaitFor = (UDP_RECEIVE_PACKET_BIT);
    EventBits_t xEventGroupValue;
    //const TickType_t ticksToWait = 1 / portTICK_PERIOD_MS; // Wait 1ms

    while(runLoops) {
        if (machineEnabled) { /* Handle UDP packet watchdog first */
            if (millis() - ul_udptxrx_watchdog > 5000) {
                logMessage("UDP Packet Watchdog triggered. Resetting machine state");
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
        xEventGroupValue  = xEventGroupWaitBits(eventUDPPacketStateGroup, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY); 
        udpRxLoopCount++;
        if (( xEventGroupValue & ( UDP_RECEIVE_PACKET_BIT ) ) == ( UDP_RECEIVE_PACKET_BIT )) /* If UDP Packet RX bit is set */
        {   
            ul_udptxrx_watchdog = millis();
            
            commandHandler(); /* No need to process inputHandler() as they're interrupt driven and sent asynchronously in realtime on a separate task */
            
            if (machineEnabled) {
                outputHandler();
                
            } else {
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        /* vTaskDelay or yield context switch not required as the event group delay above handles it */
    }

    logMessage("Exiting loop_Core0_CommandHandlerTask");
    vTaskDelete( NULL );
}


void loop_Core1_ServoStatsTask(void* parameter) /* IRAM_ATTR not required - low speed background task */
{
    logMessage("loop_Core1_ServoStatsTask running...");

    while (runLoops) {
        if (consoleLogging && startupStage1Complete) {
            long now_ProfileStats = millis();
            if (now_ProfileStats - lastMsg_ProfileStats > 1000) {
                printf("Moving: %d, LoopTx: %d, LoopRx: %d, UDP: TX PPS: %d, RX PPS: %d, TX Errors: %d, RX Errors: %d, EspNowTX: %d, InputIntrCtr: %d\r\n", motorsMoving, udpTxLoopCount, udpRxLoopCount, fb.udp_seq_num, udp_rx_seq_num, udpPacketTxErrors, udpPacketRxErrors, espnowTxPackets, inputInterruptCounter);
                udpTxLoopCount = 0;
                udpRxLoopCount = 0;
                fb.udp_seq_num = 0;
                udp_rx_seq_num = 0;
                inputInterruptCounter = 0;

                if (motorsSetup) {                
                    bool printStepperStats = false;
                    for (int i = 0; i < configNumSteppers; i++) {
                        if (stepper[i]->isRampGeneratorActive()) {
                            printStepperStats = true;
                            printf("Stepper[%d]: Pos Diff: %d, Freq Hz: %d, ", i, (cmd.pos[i] - fb.pos[i]), stepper[i]->getCurrentSpeedInMilliHz()/1000);
                        }
                    }
                    if (printStepperStats)
                        printf("\n");
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
            vTaskDelay(1); // No need to delay the task for 100ms
        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Task Sleep 1000ms
        }
    }

    logMessage("Exiting loop_Core1_ServoStatsTask");
    vTaskDelete(NULL);

}


/*==================================================================*/

void debugAxisState(uint8_t axisNum) 
{

    if (!debugAxisMovements)
        return;
    
    bool bitVals[8] = {false};

    for (uint8_t i = 0; i < configNumSteppers; i++)
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

void IRAM_ATTR updateAxisState(uint8_t axisNum, uint8_t mask ) 
{    
    axisState[axisNum] = (mask);
    if (axisState[axisNum] != prev_axisState[axisNum]) {
        if (debugAxisMovements)
            xQueueSendFromISR(axisStateInterruptQueue, &axisNum, NULL);    
        prev_axisState[axisNum] = axisState[axisNum];
    } 
}

void IRAM_ATTR updateAxisState(uint8_t axisNum, uint8_t bit, bool new_value ) 
{   
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



/* FastAccellStepper Motor API movement command & feedback of position, accel and speed handled in single hardware timer. Non-blocking calls. Debug optional (slows cmd.pos queue fractionally )*/
void IRAM_ATTR ServoMovementCmds_ISR() 
{
    if (machineEnabled) {
        uint8_t moveResult = 0;
        bool anyMoving = false;
        
        /* 
            Uses the FastAccelStepper moveTo() command to ensure that any missed or out of sync UDP packets the position is always handled correctly. 
            Position is a 32bit integer of motor step position and not pulses and is accurate.
        */
        
        for (uint8_t i = 0; i < configNumSteppers; i++) {
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
                        if (velDiff > 10000 ) { // accelerating > 1000mHz (1Hz)
                            stepper[i]->setLinearAcceleration(0); // Initial linear acceleration but reduced to 100 once hits coasting speed
                            stepper[i]->setSpeedInMilliHz(newVel); // Repeated call whilst moving to ensure changes in velocity is tracked accordingly.
                            updateAxisState(i, AXIS_STATE_MOVE_ACCEL_REQ, 1);
                        } else if (velDiff > -10000 ) { // decelerating but ignore anything less than -1000mHz (-1Hz)
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
            } else {
                isMovementRunning[i] = false;
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
                logErrorMessage("Move Error %d for axis %d", moveResult, 0); // TODO WARNING Repeated calls to this could cause a crash
                fb.pos[i] = stepper[i]->getCurrentPosition();
                fb.vel[i] = stepper[i]->getCurrentSpeedInMilliHz(false);

            } else { // Moving success
                /*
                    See https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md#stepper-position
                */
                
                if (stepper[i]->isRampGeneratorActive()) { /* Moving (fastest atomic call to the FAS lib to check if axis is moving) */
                    uint8_t rampState = stepper[i]->rampState();
                    
                    isMovementRunning[i] = true;
                    fb.pos[i] = stepper[i]->targetPos(); // Set cmd.pos to targetPos since it's where it will be after this movement in queue is complete
                    
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
                    fb.vel[i] = 0;
                    if (isMovementRunning[i]) {    
                        isMovementRunning[i] = false;
                        if (axisState[i] != 0) {
                            updateAxisState(i, AXIS_STATE_STOPPED, 1);
                        }
                    }
                }
            }
        } /* End stepper loop */
        for (int i = 0; i < MAX_STEPPER; i++) {
            if (isMovementRunning[i]) {
                anyMoving = true;
                motorsMoving = true; /* Set globally  */
            }
            if (prevIsMovementRunning[i] != isMovementRunning[i]) {
                xEventGroupSetBits(xEventStateChangeGroup, (ESPNOW_SEND_BIT)); // Send an async ESP-NOW packet to ensure motorsMoving is detected
                prevIsMovementRunning[i] = isMovementRunning[i];
            }
        }
        if (!anyMoving)
            motorsMoving = false;
    } else {
        motorsMoving = false; /* Set globally - Machine is currently disabled */
    }
}


bool fasExternalCallForPin(uint8_t pin, uint8_t value) /* Used for I2S output boards - DIR/EN pins e.g MKS-DLC32. Not STEP pins (RMT only) */
{
  // This example returns the previous value of the output.
  // Consequently, FastAccelStepper needs to call setExternalPin twice
  // in order to successfully change the output value.
  bool oldValue = digitalRead(pin);
  digitalWrite(pin, value);
  return oldValue;
}


bool setupMotors() 
{
    if (!doMotorConfig)
        return false;
    if (configNumSteppers == 0) {
        configNumSteppers = board_pin_config.num_steppers;
        if (configNumSteppers == 0) {
            logErrorMessage("Config number of steppers is 0. Use 'boardconfig' to set");
            return false;
        }
    }
    bool result = true;
    logMessage("*** Configuring %d motors  ***", configNumSteppers);
    *stepper[configNumSteppers];

    for (int i = 0; i < configNumSteppers; i++) {
        cmd.vel[i] = 0;
        fb.vel[i] = 0;
    }
    stepperEngine = FastAccelStepperEngine();
    stepperEngine.init(0); /* Use Core0 for stepper engine */

    if (configBoardType == BOARD_TYPE_ESP32_MKSDLC32)
        stepperEngine.setExternalCallForPin(fasExternalCallForPin);
    
    if (configVersion == 0) {
        configNumSteppers = board_pin_config.num_steppers; // Set to predefined template if no config 
    }

    for (uint8_t i = 0; i < configNumSteppers; i++) {
        logMessage("Creating stepper %d", i);
        FastAccelStepper *s = NULL;
        //const stepper_config_t *config = &stepper_config[i];
        const stepper_config_t *config = &board_pin_config.stepperConfig[i];
        if (config->step != PIN_UNDEFINED) {
            logMessage("configuring stepper %d", i);
#if CONFIG_IDF_TARGET_ESP32S2
            s = stepperEngine.stepperConnectToPin(config->step);
#else
            s = stepperEngine.stepperConnectToPin(config->step, DRIVER_RMT);
#endif
            if (s) {
                logMessage("attached stepper %d", i);
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
                logErrorMessage("Attaching stepper %d to RMT Driver", i);
                result = false;
            }
        }
        stepper[i] = s;
        logMessage("stepper %d created successfully", i);
    }
    
    return result;
}

size_t logMotorDebugMessage(uint8_t row, const char *format, ...) { //TODO fix this with new serial console
    if (!consoleLogging)
        return 0;
    
    va_list arg;
    uint8_t col = 0;

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
    if (row > 0) {
        //(row == 1) ? col = 5 : col = (row * 20)+5; // Column spacing for each axis debug output
    } else {
        row = 3;
    }

    //len = Serial.print(buffer);
    //len = printf("\e[F\e[%dA\e[%dG%s\r\e[%dB\n",row, col,buffer, row);
    len = printf("%s\n",buffer);

    if (buffer != temp) {
        delete[] buffer;
    }
    return len;  
}

 /* Used once in commandHandler task when machine turned ON. Purely for inital input state, ISR handles any future changes */
void IRAM_ATTR inputHandler()
{
    for (uint8_t i = 0; i < MAX_INPUTS; i++) {
        const inputpin_config_t *pin_config = &board_pin_config.inputConfigs[i];
        if (pin_config->gpio_number != GPIO_NUM_NC) {
            const uint8_t io_mask = getIONumber(i);
            bool registerBitState = false;
            if (i == 0) {
                registerBitState = (bool)REG_GET_BIT(pin_config->register_address, pin_config->register_bit);
                (registerBitState) ? fb.io |= io_mask : fb.io = 0;
            }
            else {      
                registerBitState = (bool)REG_GET_BIT(pin_config->register_address, pin_config->register_bit);
                if (registerBitState)
                    fb.io |= io_mask;
            }
        } else { // If pin not connected
            if (i == 0) // if pinindex is 0
                fb.io = 0; // set initial fb.io value
        }
    }
    xEventGroupSetBits(xEventStateChangeGroup, ESPNOW_SEND_BIT); // Send an async ESP-NOW packet as required
}


/* Called on each packet within commandHandler task loop */
void IRAM_ATTR outputHandler() 
{
    static int last_pwm[MAX_OUTPUTS] = { 0 };
    static int last_state[MAX_OUTPUTS] = { 0 };

    bool enable = cmd.control & CTRL_ENABLE;

    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) {
        const outputpin_config_t *pin_config = &board_pin_config.outputConfigs[i];
        if (pin_config->gpio_number != GPIO_NUM_NC) {
            if (pwm_enable[i]) // PWM output pin configured
            {
                if (enable) 
                { // machine on
                    if (last_pwm[i] != cmd.pwm[i]) { // Turn on PWM
                        last_pwm[i] = cmd.pwm[i];
                        ledcWrite(i, last_pwm[i]);
                        logMessage("mON: Output[%d]: PWM GPIO: %d, Name: '%s', State:%d", i, pin_config->gpio_number, pin_config->name.c_str(), last_pwm[i]);
                    }
                } else { // machine off - Turn Off PWM
                    ledcWrite(i, 0);
                    last_pwm[i] = 0;
                    logMessage("mOFF: Output[%d]: PWM GPIO: %d, Name: '%s', State:%d", i, pin_config->gpio_number, pin_config->name.c_str(), 0);
                }

            } else { // On|Off output pin configured
                const uint8_t io_mask = getIONumber(i);
                if (last_state[i] != (cmd.io & io_mask)) {
                    if (enable) { // machine on
                            if (cmd.io & io_mask) {
                                logMessage("mON: Output[%d]: GPIO: %d, Name: '%s' State:%d", i, pin_config->gpio_number, pin_config->name.c_str(), 1);
                                digitalWrite(pin_config->gpio_number, HIGH);
                            }  else {
                                logMessage("mON: Output[%d]: GPIO: %d, Name: '%s', State:%d", i, pin_config->gpio_number, pin_config->name.c_str(), 0);
                                digitalWrite(pin_config->gpio_number, LOW);
                            } 
                        
                    } else { // machine off
                        logMessage("mOFF: Output[%d]: GPIO: %d, Name: '%s', State:%d", i, pin_config->gpio_number, pin_config->name.c_str(), 0);
                        digitalWrite(pin_config->gpio_number, LOW);
                    }
                    last_state[i] = (cmd.io & io_mask);
                }
            }
        }
    }

}