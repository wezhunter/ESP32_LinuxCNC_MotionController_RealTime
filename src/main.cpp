/*    External step generator and IO interface for LinuxCNC over Ethernet
 *    with dual-core ESP32 and built-in Ethernet RMIi PHY LAN8270 or SPI Ethernet W5500 modules
 *
 *    Copyright 2024 Wez Hunter <wez at hunter.to>
 *    Copyright 2022 Juhász Zoltán <juhasz.zoltan at freemail dot hu> (Original HAL2UDP project that this was based on but firmware no longer resembles much of it)
 * 
 *    version:20240218
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
 *    GPIO pins - see hardware docs for each board and map pins to your needs. All settings in Config.h. 
 *    Motor STEP pins must be on native GPIO not on a I2C/I2S bus for RMT peripheral to work. Everything else is free reign.
 *
 */

#include <Arduino.h>
#include <ESP32Console.h>
#include "ESP32Console/Helpers/PWDHelpers.h"
#include "ArduinoNvs.h"
#include <WiFi.h>        // For connecting ESP32 to WiFi
#include <ArduinoOTA.h>  // For enabling over-the-air updates via network either WiFi or Ethernet
#include <esp_timer.h>
#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>
#include "esp32-hal-log.h"

#include "Types.hpp"
#include "ConsoleMenus.h"
#include "Config.h"
#include "ConfigManager.h"
#include "ControllerMode.h"
#include "ClientMode.h"
#include "Pins.h"
#include "I2SOut.h"


#if ESP32_RMII_ETHERNET
    #include <ETH.h> /* Espressif RMII Native Ethernet lib */
#elif ESP32_SPI_ETHERNET
    #include <AsyncUDP_ESP32_Ethernet.h> /* W5500 SPI Native Ethernet lib */
#elif ESP32SX_USB_ETHERNET // TODO - USB ECM/RNDIS Ethernet for S2/S3 series - WIP
    //#include "tusb_config.h"
    //#include "bsp/board_api.h"
    // #include "tusb.h"
    // #include "dhserver.h"
    // #include "dnserver.h"
    #include "lwip/init.h"
    #include "lwip/timeouts.h"
    #include "tinyusb.h"
    #include "tusb_cdc_acm.h"
    #include "tusb_console.h"
#endif

/*==================================================================*/

xQueueHandle IRAM_ATTR inputInterruptQueue;
bool eth_connected = false;
bool wifi_connected = false;
uint8_t inputPinInterruptFired[MAX_INPUTS] = { 0 };


/*==================================================================*/

void WiFiEvent(WiFiEvent_t event) 
{
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        printf("WiFi IP: %s\r\n", WiFi.localIP().toString().c_str());
        break; 
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        printf_P(PSTR("WiFi Connected!\r\n"));
        wifi_connected = true;
        break; 
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        telnetDisconnectAllClients();
        printf_P(PSTR("WiFi Lost Connection!\r\n"));
        wifi_connected = false;
        break; 
    case ARDUINO_EVENT_WIFI_STA_STOP:
        telnetDisconnectAllClients();
        printf_P(PSTR("WiFi Station Stopped!\r\n"));
        wifi_connected = false;
        break; 
    case ARDUINO_EVENT_WIFI_STA_START:
        printf_P(PSTR("WiFi Station Started\r\n"));
        break; 
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
        printf_P(PSTR("WiFi Connected!\r\n"));
        wifi_connected = true;
        break; 
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
        telnetDisconnectAllClients();
        printf_P(PSTR("WiFi Lost Connection!\r\n"));
        wifi_connected = false;
        break; 
    case ARDUINO_EVENT_WIFI_AP_STOP:
        //printf_P(PSTR("WiFi AP Stopped!\r\n"));
        wifi_connected = false;
        break;
    case ARDUINO_EVENT_WIFI_AP_START:
        //printf_P(PSTR("WiFi AP Started\r\n"));
        break;
#if CONFIG_IDF_TARGET_ESP32S2
#else
    case ARDUINO_EVENT_ETH_START:
        printf_P(PSTR("Ethernet Started\r\n"));
        ETH.setHostname("esp32-linuxcnc");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        printf_P(PSTR("Ethernet Link UP\r\n"));
        eth_connected = true;
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        printf("Ethernet MAC: %s, IPv4: %s, %s Duplex, Speed: %dMbps\r\n", ETH.macAddress().c_str(), ETH.localIP().toString().c_str(), (ETH.fullDuplex()) ? "Full" : "Half",  ETH.linkSpeed());
        eth_connected = true;
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        telnetDisconnectAllClients();
        printf_P(PSTR("Ethernet Link DOWN\r\n"));
        eth_connected = false;
        break;
    case ARDUINO_EVENT_ETH_STOP:
        printf_P(PSTR("Ethernet Stopped\r\n"));
        eth_connected = false;
        break;
#endif
    default:
        break;
    }
}

/*==================================================================*/
void IRAM_ATTR inputPinChangeISR(void *args) 
{
    //printf(".");
    inputInterruptCounter++;
    int inputIndex = (int)args;
    xQueueSendFromISR(inputInterruptQueue, &inputIndex, NULL);
}

void IRAM_ATTR loop_Core0_InputPinHandlerTask(void * parameter) 
{
    logMessage("loop_Core0_InputPinHandlerTask running...");
    uint8_t inputIndex;
    while (runLoops) {
        if (xQueueReceive(inputInterruptQueue, &inputIndex, portMAX_DELAY))
        {
            const inputpin_config_t *pin_config = &board_pin_config.inputConfigs[inputIndex];
            
            /* Updates the fb.io struct with the GPIO register state from the inputPin_config struct data in Config.h */
            bool registerBitState = (bool)REG_GET_BIT(pin_config->register_address, pin_config->register_bit);
            const uint8_t io_mask = getIONumber(inputIndex);
            if ((fb.io & io_mask) != registerBitState) {
                (registerBitState) ? fb.io |= io_mask : fb.io &= ~io_mask;
                inputPinInterruptFired[inputIndex]++;

                if (inputPinInterruptFired[inputIndex] >= 50) {
                    logMessage("WARNING: Excessive input changes. Index: %d, GPIO: %d. Changed %d times. Resetting counter", inputIndex, pin_config->gpio_number, inputPinInterruptFired[inputIndex]);
                    inputPinInterruptFired[inputIndex] = 0;
                }

                xEventGroupSetBits(xEventStateChangeGroup, (ESPNOW_SEND_BIT)); // Send an async ESP-NOW packet as required
                
                if (configMode == MODE_CONTROLLER) {
                    if (!motorsMoving) // Only send a udp feedback pkt if motors are stationary. When moving, feedback sent in realtime as per the fb.io struct
                        xEventGroupSetBits(eventUDPPacketStateGroup, (UDP_SEND_PACKET_BIT));
                }   
            }
           
        }
    }
    logMessage("Exiting loop_Core0_InputPinHandlerTask");
    vTaskDelete(NULL);
}

/*==================================================================*/
/*================= Setup sections =================================*/
/*==================================================================*/

void setupInputPin(int inputIndex) /* See Config.h for inputPin board specific configurations */
{   
    const inputpin_config_t *pin_config = &board_pin_config.inputConfigs[inputIndex];
    if (GPIO_IS_VALID_GPIO(pin_config->gpio_number)) {
        logMessage("INPUT[%d] GPIO: %d, Name: '%s' PullUp: %d, PullDown: %d", inputIndex, pin_config->gpio_number, pin_config->name.c_str(), pin_config->pullup, pin_config->pulldown);
        gpio_reset_pin(pin_config->gpio_number);
        gpio_set_direction(pin_config->gpio_number, GPIO_MODE_INPUT);
        if (pin_config->pullup)
            gpio_pullup_en(pin_config->gpio_number);
        if (pin_config->pulldown)
            gpio_pulldown_en(pin_config->gpio_number);
        
        /* Use ESP-IDF gpio isr attachment as it permits passing an argument to the ISR function (inputIndex)*/
        gpio_set_intr_type(pin_config->gpio_number, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(pin_config->gpio_number, inputPinChangeISR, (void*)inputIndex);
    } else {
        logMessage("INPUT[%d] Config GPIO unconfigured", inputIndex);    
    }
}


bool startEthernet()
{
    bool ethernet_setup = false;
#if ESP32_RMII_ETHERNET
    const ethernet_phy_pinconfig_t phy_pin_config = eth_phy_configs[configBoardType];
    ethernet_setup = ETH.begin(phy_pin_config.address,phy_pin_config.power_enable_pin,phy_pin_config.phy_mdc_pin,phy_pin_config.phy_mdio_pin,phy_pin_config.phy_type, phy_pin_config.phy_clk_mode, false);

    if (ethernet_setup) {
        logMessage("Ethernet driver setup successfully");
    } else {
        logErrorMessage("Failed to install Ethernet driver. Ensure board type is correct!");
    }
#elif ESP32_SPI_ETHERNET
    if (configBoardType == BOARD_TYPE_ESP32_MKSDLC32) {
        ESP32_W5500_onEvent();

        ethernet_setup = ETH.begin( configSpiMisoPin, configSpiMosiPin, configSpiSckPin, configSpiCsPin, configSpiIntPin, SPI_CLOCK_MHZ, ETH_SPI_HOST, (uint8_t*) ethernet_mac );

        if (!ethernet_setup)
            logErrorMessage("SPI Ethernet could not be initialised. Please ensure wiring, board type or 'spiethconf' is correct");
        else {
            logMessage("SPI Ethernet driver setup successfully");
        }

    }
#endif

    if (configBoardType != BOARD_TYPE_NONE && ethernet_setup)
        ETH.config(ethernet_ip, ethernet_gw, ethernet_subnetmask);
    
    return ethernet_setup;
}

void startSafeModeEthernet() 
{
    WiFi.onEvent(WiFiEvent); /* Always map networking event handler irrespective of WiFi enabled/disabled */
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP("NULLSSID","NULLPWD123456",1,1,1);
    
    logMessage("OTA Setup");
    ArduinoOTA.begin();  /* Enable OTA Service */
    ArduinoOTA.onStart(otaUpdateStart);
    ArduinoOTA.onEnd(otaUpdateEnd);
    ArduinoOTA.onProgress(otaProgress);
    logMessage("OTA Setup Done");
#ifndef ESP32_WOKWI_SIMULATOR
    startEthernet();
#endif
    WiFi.disconnect(true,false);  
    WiFi.mode(WIFI_OFF); /* Shutdown WiFi */
}

void wifiConnectionTask(void* parameter) /* One-shot background WiFi connection task */
{
    if (configWifiMode == WIFI_MODE_STA || configWifiMode == WIFI_MODE_APSTA) {
        uint8_t connectCount = 0;
        while (WiFi.status() != WL_CONNECTED) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            connectCount++;
            if (connectCount > 10) {
                logErrorMessage("Timeout for 5s whilst connecting to WiFi network '%s'. Skipping...", configWifiSSID.c_str());
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void setup_Core0(void* parameter) /* Anything requiring exec on Core0 should be setup in this one-shot task */
{
    logMessage("setup_Core0()");

    inputInterruptQueue = xQueueCreate(30, sizeof(uint8_t)); // Create queue on Core0 where input ISR execs

#ifndef ESP32_WOKWI_SIMULATOR // Not for simulator. Uses WiFi and Woki Private Gateway to perform UDP port forwarding to LinuxCNC
    logMessage("Starting Ethernet...");
    
    startEthernet();
    delay(200);
#endif

    logMessage("Starting Telnet server on port 23");
    telnetServer.onClient(onTelnetClient, NULL);  /* Uses WIFI & Private gateway on Wokwi simulator*/
    telnetServer.begin();
        
    if (configMode == MODE_CONTROLLER) {
        setupControllerModeCore0();
    }
    delay(10); // Wait for xEventStateChangeGroup to be created;

    xTaskCreatePinnedToCore(
        loop_Core0_InputPinHandlerTask, // Task function.
        "t_C0_InPin", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        4, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0


    /* Must be performed post Ethernet setup due to SPI interrupt global isr service install */
    logMessage("Configuring inputs...");
    
    for (uint8_t i = 0; i < MAX_INPUTS; i++) {
        setupInputPin(i);
    }

    logMessage("Core0 setup completed successfully");
    
    startupStage1Complete = true;
    vTaskDelete(NULL);
}

/*==================================================================*/

void stopProcessing() 
{

    if (runLoops) {
        logMessage("Stop Processing!");
        runLoops = false;
        if (configMode == MODE_CONTROLLER) {
            stopProcessingController();
        } else if (configMode == MODE_CLIENT) {
            stopProcessingClient();
        }
        for (int i = 0; i < MAX_INPUTS; i++) {
            const inputpin_config_t *pin_config = &board_pin_config.inputConfigs[i];
            if (pin_config->gpio_number != GPIO_NUM_NC)
                detachInterrupt(pin_config->gpio_number);
        }
        logMessage("Stop Processing Done");

    } else {
        logMessage("Already stopped");
    }
}

void otaUpdateStart() 
{   
    consoleLogging = true; // Enable logging to serial console
    stopProcessing(); 
    console.~Console(); // Stop console commands
    delay(150); // Wait for console to stop its task
    printf_P(PSTR("OTA Update Starting....\r\n")); // Use printf to control CRLF
    
    //disableCore0WDT(); // Testing indicates its ok to keep disabled
#ifndef CONFIG_FREERTOS_UNICORE // For ESP32-S2 single core
    //disableCore1WDT(); // Testing indicates its ok to keep disabled
#endif
    
}

void otaUpdateEnd() 
{
    printf_P(PSTR("OTA Update completed\r\n")); 
}

void otaProgress(unsigned int progress, unsigned int total) 
{
    printf("OTA Update Progress: %u%%\r", (progress / (total / 100))); 
}

void initUsbCdc() //TODO Future USB device mode
{
    // const tinyusb_config_t tusb_cfg = {
    //     .descriptor = NULL,
    //     .string_descriptor = NULL,
    //     .external_phy = false, // In the most cases you need to use a `false` value
    // };

    // esp_err_t esp_err = tinyusb_driver_install(&tusb_cfg);
    
    // tinyusb_config_cdcacm_t acm_cfg = {
    // .usb_dev = TINYUSB_USBDEV_0,
    // .cdc_port = TINYUSB_CDC_ACM_0,
    // .rx_unread_buf_sz = 64,
    // .callback_rx = NULL,
    // .callback_rx_wanted_char = NULL,
    // .callback_line_state_changed = NULL,
    // .callback_line_coding_changed = NULL
    // };

    // tusb_cdc_acm_init(&acm_cfg);

    // esp_tusb_init_console(TINYUSB_CDC_ACM_0);
}

void parseTelnetMessage(AsyncClient* c, String message)
{   
    if (message.equals("r\r\n")) {
        logMessage("R key pressed. Restart..");
        telnetDisconnectAllClients();
        ESP.restart();
    } else if (message.equals("s\r\n")) {
        logMessage("S key pressed");
        stopProcessing();        
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
    logMessage("Telnet: client disconnected");
}

void onTelnetClient(void *s, AsyncClient* c) 
{
    telnetClientConnected++;
    telnetClient = c;
    c->onDisconnect(onTelnetClientDisconnected);
    c->onData(onTelnetClientData);
    IPAddress client_address = c->getRemoteAddress();
    logMessage("Telnet: '%s' client connected", client_address.toString().c_str() );
    String welcomeMsg = String("Welcome. Version: " + version_number + ". Ctrl+Z to quit\r\n");
    
    c->write("\eSP F");  // Use 7-bit control codes
    c->write("\e[?25l"); // Hide cursor
    c->write("\e[?12l"); // Disable cursor highlighting
    c->write(welcomeMsg.c_str());
}

/*==================================================================*/
void enterSafeMode(bool basic_commands=false) 
{
    safeMode = true;
    
    console.begin(BAUD_RATE);
    console.registerSystemCommands();
    console.registerNetworkCommands();
    logErrorMessage("BOOT LOOP DETECTED. ENTERING SAFE MODE");
    if (basic_commands) {
        logErrorMessage("BASIC COMMANDS DUE TO EXCESSIVE BOOT LOOPS");
        extraSafeModeConsolePrompt();
    } else {
        safeModeConsolePrompt();
        registerSafeModeConsoleCmds();
        logWarningMessage("MUST use 'getconfig' to setup environment before running 'saveconfig' or 'startethernet'");
        logWarningMessage("Use 'startethernet' safemode command to start Etherner for OTA firmware updates");
    }
    
    NVS.setInt("BOOT_COUNT", 0); // Clear boot loop counter to retry on next boot

}
bool safeModeHandler()    /* Returns true if in safemode. Setup() exits if true   */
{
    uint8_t bootCounter = NVS.getInt("BOOT_COUNT");
    NVS.setInt("BOOT_COUNT", bootCounter+1);
    logMessage("Boot Count: %d", bootCounter);
    if (bootCounter > 10){
        enterSafeMode(true);
        return true;
    }
    if (bootCounter > 2){
        enterSafeMode();
        return true;
    }
    return false;
 
}

void setup()
{
    consoleLogging = true; // Set to show startup setup messages
    
    initUsbCdc(); // Temporary

    esp_log_level_set("*", ESP_LOG_WARN);

    Serial.begin(BAUD_RATE);

    Serial.print("\eSP F");  // tell to use 7-bit control codes
    Serial.print("\e[?25l"); // hide cursor
    Serial.print("\e[?12l"); // disable cursor highlighting
    Serial.println();
    delay(50);
    Serial.println("=================================================");
    Serial.println("==================== STARTUP ====================");
    Serial.println("=================================================");

#ifdef ESP32_WOKWI_SIMULATOR
    logWarningMessage("\n==================== SIMULATOR DEBUG BUILD ====================\n");
#endif
#ifdef ESP32_RMII_ETHERNET
    logMessage("\n============ RMII ETH RELEASE BUILD =============\n");
#endif
#ifdef ESP32_SPI_ETHERNET
    logMessage("\n============ SPI ETH RELEASE BUILD ==============\n");
#endif

    logMessage("Version: %s", version_number);
    logMessage("Setup started..");
    logMessage("APBFreq: %d", getApbFrequency());

    logMessage("Total heap: %d", ESP.getHeapSize());
    logMessage("Free heap: %d", ESP.getFreeHeap());
    logMessage("Total PSRAM: %d", ESP.getPsramSize());
    logMessage("Free PSRAM: %d", ESP.getFreePsram());

    #ifndef ESP32_SPI_ETHERNET // Only do this once when not SPI ethernet. SPI ethernet driver installs this automatically
    logMessage("Installing Global GPIO ISR Service");
    gpio_install_isr_service(0);
    #endif

    NVS.begin(); // init non-volatile flash storage for config store
    
    if (safeModeHandler()) 
        return; // Checks for boot loops. Before readNVSConfig in the event of a config related issue
    
    readNvsConfig(); // Main call to get configuration from NVS

    WiFi.onEvent(WiFiEvent); /* Always map networking event handler irrespective of WiFi enabled/disabled */
    WiFi.mode(configWifiMode); // from config store
    
    if (configWifiMode == WIFI_MODE_STA) {
        logMessage("WiFi Station: Connecting to WiFi SSID: %s", configWifiSSID);
        WiFi.begin(configWifiSSID, configWifiPwd);
    } else if (configWifiMode == WIFI_MODE_AP) {
        logMessage("WiFi AP: Creating SSID: %s", configWifiSSID);
        WiFi.softAP(configWifiSSID, configWifiPwd, configWifiHide, 1, 4); 
    } else if (configWifiMode == WIFI_MODE_APSTA) {
        WiFi.begin(configWifiSSID, configWifiPwd);
    } else {
        logWarningMessage("WiFi is DISABLED in config");
        WiFi.softAP("NULLSSID","NULLPWD123456",1,1,1);
    }
    
    logMessage("WiFi MAC: %s", WiFi.macAddress().c_str());

    xTaskCreatePinnedToCore(
        wifiConnectionTask, // Task function.
        "t_C1_WL_Conn", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        tskIDLE_PRIORITY, // priority of the task
        NULL, // Task handle to keep track of created task
        1); // pin task to core 1
    
    logMessage("OTA Setup");
    ArduinoOTA.begin();  /* Enable OTA Service */
    ArduinoOTA.onStart(otaUpdateStart);
    ArduinoOTA.onEnd(otaUpdateEnd);
    ArduinoOTA.onProgress(otaProgress);
    logMessage("OTA Setup Done");
    
    if (configWifiMode == WIFI_OFF) {
        WiFi.disconnect(true,false);  
        WiFi.mode(WIFI_OFF); /* shutdown wifi */
    }
    if (configBoardType == BOARD_TYPE_ESP32_MKSDLC32) {
        i2s_out_init();
    }

    runLoops = true; // Important to ensure background tasks exec in a loop
    
    xTaskCreatePinnedToCore(
        setup_Core0, // Task function.
        "t_C0_Setup", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        tskIDLE_PRIORITY, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0

    if (configMode == MODE_CONTROLLER) {
        setupControllerMode();
    } else if (configMode == MODE_CLIENT) {
        setupClientMode();
    }

    logMessage("Setup completed successfully");

    while (!startupStage1Complete) { // Wait for all required tasks to finish starting before enabling console
        delay(200);
    }

    consoleLogging = false; // Disable logging to console once startup is complete
    
    defaultConsolePrompt();
    console.begin(BAUD_RATE);
    console.registerSystemCommands();
    console.registerNetworkCommands();
    console.registerGPIOCommands();
    
    registerConsoleCmds(); // In ConsoleMenus.cpp
    
    if (configBoardType == BOARD_TYPE_NONE)
        logErrorMessage("No board type configured. See 'boardconfig' in 'help'");
    if (configMode == MODE_CONTROLLER && !doMotorConfig)
        logWarningMessage("Controller mode selected but motor configuration is not enabled. See 'setmotor','enablemotorconfig' commands in 'help'\r\n");
    if (configMode == MODE_CONTROLLER && configVersion == 0)
        logWarningMessage("Controller mode is default. To use Client mode see 'mode'\r\n");
    if (configMode == MODE_CLIENT && configRemotePeerAddress.equals(""))
        logWarningMessage("Client mode selected but Controller MAC address unset. See 'setcontrollermac' in 'help'\r\n");
    if (configEspNowEnabled && configWifiMode != WIFI_MODE_APSTA)
        logWarningMessage("ESP-NOW is enabled but WiFi mode not set to AP+STA. See 'wificonfig' mode 3 in 'help'\r\n");

    NVS.setInt("BOOT_COUNT", 0); // Reset boot loop detection
}


/*==================================================================*/
void loop()
{

#ifndef ESP32_WOKWI_SIMULATOR // Not for Wokwi Simulator
    if (eth_connected || wifi_connected)
        ArduinoOTA.handle();  // Handles a code update request
#endif

    delay(200);
}
