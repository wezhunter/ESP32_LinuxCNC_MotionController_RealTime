#include <Arduino.h>
#include "ConfigManager.h"

void setConfigEnvVars(bool silent) 
{
    char tmpStr[3];
    if (!silent) {
        printf("[Mode]: '%s'\r\n", ((configMode == MODE_CONTROLLER) ? "Controller" : "Client"));
        printf("[Board Config] Type: %d, Name: '%s'\r\n", (uint8_t)configBoardType, configBoardName.c_str());
        printf("[WiFi Config] Mode: (%d) '%s', SSID: '%s', PWD: '%s', Hidden: %d\r\n", (uint8_t)configWifiMode, getWifiConfigMode().c_str(), configWifiSSID.c_str(), configWifiPwd.c_str(), configWifiHide);
#if ESP32_RMII_ETHERNET
        const ethernet_phy_pinconfig_t phy_pin_config = eth_phy_configs[configBoardType];
        printf("[Ethernet] Type: 'RMII' { address: %d, pwr_pin: %d, mdc_pin: %d, mdio_pin: %d, phy_type: %d, clk_mode: %d }\r\n", 
            phy_pin_config.address, phy_pin_config.power_enable_pin, phy_pin_config.phy_mdc_pin, phy_pin_config.phy_mdio_pin, phy_pin_config.phy_type, phy_pin_config.phy_clk_mode);
#elif CONFIG_ETH_SPI_ETHERNET_W5500
        printf("[Ethernet] Type: 'SPI', GPIO: { miso: %d, mosi: %d, sck: %d, cs: %d, int: %d }\r\n", configSpiMisoPin, configSpiMosiPin, configSpiSckPin, configSpiCsPin, configSpiIntPin);
#endif
        printf("[ESP-NOW] State: %d, PeerMAC: '%s'\r\n", configEspNowEnabled, configRemotePeerAddress.c_str());
        if (configMode == MODE_CONTROLLER) {
            printf("[UDP-SERVER] State: %d, Port: %d\r\n", runLoops, udpServerPort);
            printf("[UDP-CLIENT] State: %d, Port: %d\r\n", runLoops, udpClientPort);
        }
    }

    itoa(configMode, tmpStr, 10);
    setenv("MODE", tmpStr, 1); // overwrite existing var

    itoa(configBoardType, tmpStr, 10);
    setenv("BOARD_TYPE", tmpStr, 1); // overwrite existing var
    setenv("BOARD_NAME", configBoardName.c_str(), 1); // overwrite existing var

    setenv("ESP_NOW_EN", (configEspNowEnabled) ? "true" : "false", 1); // overwrite existing var
    if (configMode == MODE_CLIENT)
        setenv("ESP_NOW_PEER_ADDR", configRemotePeerAddress.c_str(), 1); // overwrite existing var
    
    itoa(configNumSteppers, tmpStr, 10);
    setenv("NUM_STEPPERS", tmpStr, 1); // overwrite existing var
    
    
    itoa(configWifiMode, tmpStr, 10);
    setenv("WIFI_MODE", tmpStr, 1); // overwrite existing var
    setenv("WIFI_SSID", configWifiSSID.c_str(), 1); // overwrite existing var
    setenv("WIFI_PWD", configWifiPwd.c_str(), 1); // overwrite existing var
    setenv("WIFI_HIDE", (configWifiHide) ? "true" : "false", 1); // overwrite existing var

    if (configMode == MODE_CONTROLLER) {
        printf("\r\n");
        bool hasI2SOutputs = false;
        static char conf_name[20] = "";
        static char tmp_value[5] = "";
        if (!silent) {
            printf("\r\n==========Steppers==========\n");
            printf("Number of Steppers: %d\r\n", configNumSteppers);
        }
        /* Stepper configs */
        for (uint8_t i = 0; i < configNumSteppers; i++) 
        {
            const stepper_config_t *config = &board_pin_config.stepperConfig[i];
            
            if (!silent) {
                printf("Motor[%d]: { 'StepPin': %d, 'DirPin': %d, 'EnHighPin': %d, 'EnLowPin': %d, 'AutoEn': %d, 'DirDelay': %d, 'OnDelayUs': %d, 'OffDelayMs': %d }\r\n", i, config->step, config->direction, (config->enable_high_active == PIN_UNDEFINED) ? -1 : config->enable_high_active , (config->enable_low_active == PIN_UNDEFINED) ? -1 : config->enable_low_active, config->auto_enable, config->dir_change_delay, config->on_delay_us, config->off_delay_ms);
                if ((config->direction > I2S_OUT_PIN_BASE && config->direction < PIN_UNDEFINED) || (config->enable_high_active > I2S_OUT_PIN_BASE && config->enable_high_active < PIN_UNDEFINED)  || (config->enable_low_active > I2S_OUT_PIN_BASE && config->enable_low_active < PIN_UNDEFINED)) {
                    hasI2SOutputs = true;
                }
            }
        }
        if (hasI2SOutputs)
            printf("\e[0;93m* Uses I2S output for pins. Values > 128 are I2S!\e[0m\r\n\r\n");
    }
    if (!silent) {
        printf("\r\n==========Inputs==========\r\n");
    }
    /* Input Pin Configs - Any device mode */
    for (uint8_t i = 0; i < MAX_INPUTS; i++) 
    {
        const inputpin_config_t *config = &board_pin_config.inputConfigs[i];
        if (!silent) {
            printf("Input[%d]: { 'GPIO': %d, 'UDPInNum': %d, 'Name': '%s', 'PullUp': %d, 'PullDown': %d, 'RegAddr': '%s', 'RegBit': 0x%02X }\r\n", i, config->gpio_number, i, config->name.c_str(), config->pullup, config->pulldown, (config->register_address == GPIO_IN_REG ? "GPIO_IN_REG" : "GPIO_IN1_REG"), config->register_bit);
        }
    }
    /* Output Pin Configs - Any device mode */
    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) 
    {
        const outputpin_config_t *config = &board_pin_config.outputConfigs[i];
        if (!silent) {
            printf("Output[%d]: { 'GPIO': %d, 'UDPOutNum': %d, 'Name': '%s' }\r\n", i, config->gpio_number, i, config->name.c_str());
        }
    }
}

bool resetNvsConfig() {
    bool result = true;
    printf("Resetting NVS Config...\r\n");
    NVS.eraseAll();

    return result;
}

bool saveNvsConfig() {
    bool result = false;
    printf("Saving NVS Config...\r\n");
    esp_err_t res = 0;
    static char conf_name[20] = "";

    // if (configVersion == 0) { // Always initialise In/Out default structs on first config. Restored from NVS after
         //initOutputPinsConfig();
    //     initIntputPinsConfig();
    // }
    
    NVS.setInt("BOARD_TYPE", configBoardType);
    NVS.setInt("NUM_STPPRS", configNumSteppers);

    for (uint8_t i = 0; i < configNumSteppers; i++) 
    {   
        sprintf(conf_name,"NV_MCONF%d",i);
        size_t size = sizeof(stepper_config_t);
        stepper_config_t *config = &board_pin_config.stepperConfig[i];
        res = nvs_set_blob(NVS._nvs_handle, conf_name, (void*)config, size);

        if (res == ESP_OK) {
            result = true;
        } else {
            logErrorMessage("Unable to save motor: %d nvs config", i);
            result = false;
        }
    }
    for (uint8_t i = 0; i < MAX_INPUTS; i++) 
    {   
        sprintf(conf_name,"NV_INPCFG%d",i);
        size_t size = sizeof(inputpin_config_t);
        inputpin_config_t *config = &board_pin_config.inputConfigs[i];
        res = nvs_set_blob(NVS._nvs_handle, conf_name, (void*)config, size);

        if (res == ESP_OK) {
            result = true;
        } else {
            logErrorMessage("Unable to save input pin config: %d nvs config", i);
            result = false;
        }
    }
    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) 
    {   
        sprintf(conf_name,"NV_OUTPCFG%d",i);
        size_t size = sizeof(outputpin_config_t);
        outputpin_config_t *config = &board_pin_config.outputConfigs[i];
        res = nvs_set_blob(NVS._nvs_handle, conf_name, (void*)config, size);

        if (res == ESP_OK) {
            result = true;
        } else {
            logErrorMessage("Unable to save output pin config: %d nvs config", i);
            result = false;
        }
    }
    configVersion = NVS.getInt("configVer") +1;
    NVS.setInt("configVer", configVersion);
    NVS.setInt("NUM_STPPRS", configNumSteppers);
    printf("NVS Saved. Version: %d\r\n", configVersion);
    setConfigEnvVars();
    return result;
}

void readNvsConfig(bool silent)
{
    printf("Reading NVS Config...\r\n");
    
    configVersion = NVS.getInt("configVer");
    printf("NVS Config Version: %d\r\n", configVersion);
    
    configMode = (config_mode_t) (NVS.getInt("MODE") >= 1) ? MODE_CLIENT : MODE_CONTROLLER;

    if (configVersion == 0) {
        printf("ERROR: No config defined. Run 'boardconfig' first\r\n");
        return;
    }


#ifndef ESP32_WOKWI_SIMULATOR // NOT SIMULATOR
    doMotorConfig = (NVS.getInt("doMConf") >= 1) ? true : false;
    configBoardType = (board_type_t) NVS.getInt("BOARD_TYPE");
    configBoardName = getBoardName(configBoardType);
    configWifiMode = (wifi_mode_t) NVS.getInt("WIFI_M", 0);
    configWifiSSID = NVS.getString("WIFI_SSID");
    configWifiPwd = NVS.getString("WIFI_PWD");
    configWifiHide = NVS.getInt("WIFI_HIDE", 0);
#else // SIMULATOR
    configWifiMode = WIFI_MODE_APSTA;
    configWifiSSID = "Wokwi-GUEST";
    configWifiPwd = "";
    configBoardType = BOARD_TYPE_ESP32_WOKWI_SIMUL;
    configBoardName = "Wowki Simulator ESP32";
    doMotorConfig = true;
    configNumSteppers = 3;
    configVersion = 0; // Force to 0 on every restart to ensue pinmaps are defaults
#endif        
    
    board_pin_config = board_pin_configs[configBoardType]; // Set to default values initially. Override from NVS below

    //initOutputPinsConfig();
    //initIntputPinsConfig();
    
    printf("BOARD TYPE: %d\n", board_pin_config.board_type);
    printf("BOARD NAME: %s\n", getBoardName(board_pin_config.board_type).c_str());
    // printf("M0 STEP: %d\n", board_pin_config.stepperConfig[0].step);
    // printf("M1 STEP: %d\n", board_pin_config.stepperConfig[1].step);
    // printf("M2 STEP: %d\n", board_pin_config.stepperConfig[2].step);


#ifndef ESP32_WOKWI_SIMULATOR // NOT SIMULATOR
    configNumSteppers = NVS.getInt("NUM_STPPRS",0);
    if (configNumSteppers == 0) {
        configNumSteppers = board_pin_config.num_steppers;
    }
#endif
    
    configRemotePeerAddress = NVS.getString("ESPNOW_PEER_MAC");
    
    configEspNowEnabled = (NVS.getInt("ESPNOW_ENABLE") >=1) ? true : false;

    configSpiMisoPin = NVS.getInt("SPI_MISO", 19);
    configSpiMosiPin = NVS.getInt("SPI_MOSI", 23);
    configSpiSckPin = NVS.getInt("SPI_SCK", 18);
    configSpiCsPin = NVS.getInt("SPI_CS", 0);
    configSpiIntPin = NVS.getInt("SPI_INT", 4);

    
    static char conf_name[20] = "";
    static char tmp_value[5] = "";
    
    /* Load stepper motor configs from NVS 'NV_MCONFn' key blobs */
    for (uint8_t i = 0; i < configNumSteppers; i++) 
    {
        sprintf(conf_name, "NV_MCONF%d",i);
        size_t size = sizeof(stepper_config_t);
        std::vector<stepper_config_t> *config;
        esp_err_t res = nvs_get_blob(NVS._nvs_handle, conf_name, (void*)&config, &size);

        if (res != ESP_OK && size < 5) {
            logErrorMessage("Unable to read NVS config for Motor: %d. Clear NVS and retry", i);
        } else {
            memcpy(&board_pin_config.stepperConfig[i], &config, size);
        }
    }
    /* Load input pin configs from NVS 'NV_INPCFGn' key blobs */
    for (uint8_t i = 0; i < MAX_INPUTS; i++) 
    {
        sprintf(conf_name, "NV_INPCFG%d",i);
        size_t size = sizeof(inputpin_config_t);
        std::vector<inputpin_config_t> *config;
        esp_err_t res = nvs_get_blob(NVS._nvs_handle, conf_name, (void*)&config, &size);

        if (res != ESP_OK && size < 5) {
            logErrorMessage("Unable to read NVS config for InputPin: %d. Clear NVS and retry", i);
        } else {
            memcpy(&board_pin_config.inputConfigs[i], &config, size);
        }
    }
    /* Load output pin configs from NVS 'NV_INPCFGn' key blobs */
    for (uint8_t i = 0; i < MAX_OUTPUTS; i++) 
    {
        sprintf(conf_name, "NV_OUTPCFG%d",i);
        size_t size = sizeof(outputpin_config_t);
        std::vector<outputpin_config_t> *config;
        esp_err_t res = nvs_get_blob(NVS._nvs_handle, conf_name, (void*)&config, &size);

        if (res != ESP_OK && size < 5) {
            logErrorMessage("Unable to read NVS config for OutputPin: %d. Clear NVS and retry", i);
        } else {
            memcpy(&board_pin_config.outputConfigs[i], &config, size);
        }
    }
    setConfigEnvVars(silent);
}