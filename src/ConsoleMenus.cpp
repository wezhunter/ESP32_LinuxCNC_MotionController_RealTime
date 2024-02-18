#include "ConsoleMenus.h"
#include "Types.hpp"
#include "Config.h"
#include "ConfigManager.h"
#include "ControllerMode.h"
#include "ClientMode.h"

using namespace ESP32Console;


void safeModeConsolePrompt() { console.setPrompt("\e[1;31mSAFE MODE >\e[0m \0"); }

void extraSafeModeConsolePrompt() { console.setPrompt("\e[1;31mEXTRA SAFE MODE >\e[0m \0"); }

void defaultConsolePrompt()
{   
    if (configMode == MODE_CONTROLLER)
        sprintf(prompt_text, "\e[1;32m%s >\e[0m ", controller_mode_name);
    else if (configMode == MODE_CLIENT)
        sprintf(prompt_text, "\e[1;32m%s >\e[0m ", client_mode_name);
    else
        sprintf(prompt_text, "\e[1;32m%s >\e[0m ", unknown_mode_name);
    
    console.setPrompt(prompt_text);
}

void setConsolePrompt(prompt_colour colour, const String new_status) 
{
    uint8_t colour_code = 0;
    switch (colour) {
        case ORANGE:
            colour_code = 33;
            break;
        case YELLOW:
            colour_code = 93;
            break;
        case GREEN:
            colour_code = 32;
            break;
        case BLUE:
            colour_code = 34;
            break;
        case PURPLE:
            colour_code = 35;
            break;
        case WHITE:
            colour_code = 97;
            break;
        case RED:
            colour_code = 31;
            break;
        default:
            colour_code = 0;
            break;
    }
    
    if (configMode == MODE_CONTROLLER)
        sprintf(prompt_text, "\e[1;%dm%s %s>\e[0m ", colour_code, controller_mode_name, new_status.c_str());
    else if (configMode == MODE_CLIENT)
        sprintf(prompt_text, "\e[1;%dm%s %s>\e[0m ", colour_code, client_mode_name, new_status.c_str());
    else
        sprintf(prompt_text, "\e[1;%dm%s %s>\e[0m ", colour_code, unknown_mode_name, new_status.c_str());
    
    console.setPrompt(prompt_text);

}

void registerSafeModeConsoleCmds()
{
    console.registerCommand(ESP32Console::ConsoleCommandD("startethernet", [](int argc, char **argv) -> int {
        printf("Starting Safe Mode Ethernet...\r\n");
        startSafeModeEthernet();
        return EXIT_SUCCESS;
    }, PSTR("Start safe mode Ethernet for OTA or firmware updates")));
    registerConsoleCmds();
}

void registerDebuggingCmds()
{
     console.registerCommand(ConsoleCommandD("motormovetest", [](int argc, char **argv) -> int {
        if (argc != 2) { printf_P(PSTR("Missing 'forwards' or 'backwards'\n")); return EXIT_FAILURE; }
        auto arg = String(argv[1]);
        int32_t newpos0 = 0;
        int32_t newpos1 = 0;
        int32_t newpos2 = 0;
        if (arg == "forwards") {
            printf_P(PSTR("Issuing 'forwards' motor movement test command\n"));
            newpos0 = newpos0 + random(100, 500);
            newpos1 = newpos1 + random(100, 500);
            newpos2 = newpos2 + random(100, 500);
        } else { 
            newpos0 = newpos0 - random(100, 500);
            newpos1 = newpos1 - random(100, 500);
            newpos2 = newpos2 - random(100, 500);
            printf_P(PSTR("Issuing 'backwards' motor movement test command\n"));
        }
        stepper[0]->enableOutputs();
        stepper[1]->enableOutputs();
        stepper[2]->enableOutputs();
        stepper[0]->setSpeedInHz(50000);
        stepper[1]->setSpeedInHz(50000);
        stepper[2]->setSpeedInHz(50000);
        machineEnabled = true;
        cmd.pos[0] = cmd.pos[0] + newpos0;
        cmd.pos[1] = cmd.pos[1] + newpos1;
        cmd.pos[2] = cmd.pos[2] + newpos2;
        cmd.vel[0] = random(200000, 500000); // millhertz
        cmd.vel[1] = random(200000, 500000);
        cmd.vel[2] = random(200000, 500000);
        
        return EXIT_SUCCESS;
    }, PSTR("Send debug motor movement for axis for either 'forwards' or 'backwards' moves")));
}

void registerConsoleCmds()
{

    if (configMode == MODE_CONTROLLER) {
        /* Controller mode only commands */
        OptionsConsoleCommand  setmotor("setmotor", [](int argc, char **argv, ParseResult result, Options options) -> int {
            int m_index = -1;
            if (result.count("m")) {
                m_index = result["m"].as<int>();
            }
            if (m_index < 0) { printf_P(PSTR("Invalid or missing motor index (0>5)\n")); return EXIT_FAILURE;}
            if (argc < 3) { printf_P(PSTR("Missing setting to update\n")); return EXIT_FAILURE; }
            
            printf("Motor %d: ", m_index);
            if (result.count("s")) {
                board_pin_config.stepperConfig[m_index].step = result["s"].as<int>();
                printf("Step: %d ", board_pin_config.stepperConfig[m_index].step);
            }
            if (result.count("d")) {
                board_pin_config.stepperConfig[m_index].direction = result["d"].as<int>();
                printf("Direction: %d ", board_pin_config.stepperConfig[m_index].direction);
            }
            if (result.count("h")) {
                int value = result["h"].as<int>();
                board_pin_config.stepperConfig[m_index].enable_high_active = (value == 0) ? PIN_UNDEFINED : value;
                printf("EnableHighActive: %d ", board_pin_config.stepperConfig[m_index].enable_high_active);
            }
            if (result.count("l")) {
                int value = result["l"].as<int>();
                board_pin_config.stepperConfig[m_index].enable_low_active = (value == 0) ? PIN_UNDEFINED : value;
                printf("EnableLowActive: %d ", board_pin_config.stepperConfig[m_index].enable_low_active);
            }
            if (result.count("a")) {
                board_pin_config.stepperConfig[m_index].auto_enable = result["a"].as<bool>();
                printf("AutoEnable: %d ", board_pin_config.stepperConfig[m_index].auto_enable);
            }
            if (result.count("1")) {
                int value = result["1"].as<int>();
                board_pin_config.stepperConfig[m_index].on_delay_us = (value > 120000) ? 120000 : value;
                printf("OnDelayUs: %d ", board_pin_config.stepperConfig[m_index].on_delay_us);
            }
            if (result.count("2")) {
                int value = result["2"].as<int>();
                board_pin_config.stepperConfig[m_index].off_delay_ms = (value > 120) ? 120 : value;
                printf("OffDelayMs: %d ", board_pin_config.stepperConfig[m_index].off_delay_ms);
            }
            if (result.count("c")) {
                int value = result["c"].as<int>();
                board_pin_config.stepperConfig[m_index].dir_change_delay = (value > 4095) ? 4095 : value;
                printf("DirChangeDelayMs: %d ", board_pin_config.stepperConfig[m_index].dir_change_delay);
            }

            printf(" updated. Must 'saveconfig' to apply\r\n",m_index);
            setConsolePrompt(PURPLE, "*");

            return EXIT_SUCCESS;
        }, PSTR("Configure stepper motors pins and parameters"));

        setmotor.options.add_options()("m,motor", "Motor index number (0-6)", cxxopts::value<int>());
        setmotor.options.add_options()("s,steppin", "Step GPIO (int)", cxxopts::value<int>());
        setmotor.options.add_options()("d,dirpin", "Direction GPIO (int)", cxxopts::value<int>());
        setmotor.options.add_options()("h,enablehighpin", "Enable active high GPIO (int)", cxxopts::value<int>());
        setmotor.options.add_options()("l,enablelowpin", "Enable active low GPIO (int)", cxxopts::value<int>());
        setmotor.options.add_options()("a,autoenable", "Auto enable (true|false)", cxxopts::value<bool>());
        setmotor.options.add_options()("1,ondelayus", "On delay time in us (0>120000)", cxxopts::value<int>());
        setmotor.options.add_options()("2,offdelayms", "Off delay time in ms (0>120)", cxxopts::value<int>());
        setmotor.options.add_options()("c,dirchangedelayus", "Direction change delay in us (0>4095)", cxxopts::value<int>());
        console.registerCommand(setmotor);

        console.registerCommand(ConsoleCommandD("enablemotorconfig", [](int argc, char **argv) -> int {
            if (argc != 2) { printf_P(PSTR("Missing 'on' or 'off'\n")); return EXIT_FAILURE; }
            auto arg = String(argv[1]);
            if (arg == "on") {
                doMotorConfig = true;
            } else { 
                arg = "off";
                doMotorConfig = false;
            }
            NVS.setInt("doMConf", (doMotorConfig) ? 1 : 0);
            setConsolePrompt(PURPLE, "R!");
            printf("Motor config is '%s'. Restart to apply\r\n", arg);
            return EXIT_SUCCESS;
        }, PSTR("Enable/Disable motor config at startup")));


        OptionsConsoleCommand  getmotor("getmotor", [](int argc, char **argv, ParseResult result, Options options) -> int {
            int m_index = -1;
            if (result.count("m")) {
                m_index = result["m"].as<int>();
            }
            if (m_index < 0) { printf_P(PSTR("Invalid or missing motor index (0>5)\n")); return EXIT_FAILURE;}
            const stepper_config_t *config = &board_pin_config.stepperConfig[m_index];

            printf("Motor[%d]: { 'StepPin': %d, 'DirPin': %d, 'EnHighPin': %d, 'EnLowPin': %d, 'AutoEn': %d, 'DirDelay': %d, 'OnDelayUs': %d, 'OffDelayMs': %d }\r\n", m_index, config->step, config->direction, (config->enable_high_active == PIN_UNDEFINED) ? -1 : config->enable_high_active , (config->enable_low_active == PIN_UNDEFINED) ? -1 : config->enable_low_active, config->auto_enable, config->dir_change_delay, config->on_delay_us, config->off_delay_ms);
            if ((config->direction > I2S_OUT_PIN_BASE && config->direction < PIN_UNDEFINED) || (config->enable_high_active > I2S_OUT_PIN_BASE && config->enable_high_active < PIN_UNDEFINED)  || (config->enable_low_active > I2S_OUT_PIN_BASE && config->enable_low_active < PIN_UNDEFINED)) {
                printf("\e[0;93m* Uses I2S output for pins. Values > 128 are I2S!\e[0m\r\n");
            }
            return EXIT_SUCCESS;
        }, PSTR("Get a stepper motors current config"));
        
        getmotor.options.add_options()("m,motor", "Motor index number (0-6)", cxxopts::value<int>());
        console.registerCommand(getmotor);

        console.registerCommand(ConsoleCommandD("stats", [](int argc, char **argv) -> int {
            printf("{\"Processing\": %d, UDP:{\"ServerState\": %d, \"TX PPS\": %d, \"RX PPS\": %d, \"TX Errors\": %d, \"RX Errors\": %d}, \"EspNowTX\": %d, \"InputIntrCtr\": %d}\r\n", runLoops, runLoops, udp_tx_seq_num, udp_rx_seq_num, udpPacketTxErrors, udpPacketRxErrors, espnowTxPackets, inputInterruptCounter);
            return EXIT_SUCCESS;
        }, PSTR("Loop task communications statistics")));
    /* End Controller mode only commands */
    
    } else if (configMode == MODE_CLIENT) {
        /* Client mode only commands */
        console.registerCommand(ConsoleCommandD("setcontrollermac", [](int argc, char **argv) -> int {
            if (argc != 2) { printf_P(PSTR("Missing MAC address format 00:00:00:00:00:00\n")); return EXIT_FAILURE; }
            String arg = String(argv[1]);
            if (arg.length() == 17) {
                NVS.setString("ESPNOW_PEER_MAC", arg.c_str());
                configRemotePeerAddress = arg;
                printf("Successfully set ESP-NOW Peer MAC address to '%s'", arg.c_str());
            } else {
                printf_P(PSTR("Invalid length for mac_address. Format should be 00:00:00:00:00:00\r\n"));
                return EXIT_FAILURE;
            }
            return EXIT_SUCCESS;
        }, PSTR("Set Controller WiFI MAC address to connect to via ESP-NOW.\n 'setcontrollermac [mac_address]'\nmac_address = 00:00:00:00:00:00")));
    }
    /* End Client mode only commands */

    /* Any mode global commands */
    console.registerCommand(ConsoleCommandD("mode", [](int argc, char **argv) -> int {
        if (argc != 2) { printf_P(PSTR("Missing 'controller' or 'client'\n")); return EXIT_FAILURE; }
        auto arg = String(argv[1]);
        if (arg == "controller") {
            configMode = MODE_CONTROLLER; 
            printf_P(PSTR("'Controller' mode set\n"));
        } else { 
            configMode = MODE_CLIENT;
            printf_P(PSTR("'Client' mode set\n")); 
        }
        NVS.setInt("MODE", configMode);
        printf_P(PSTR("Restart required\n"));
        setConsolePrompt(PURPLE, "R!");
        return EXIT_SUCCESS;
    }, PSTR("Set device mode to Client or Controller. mode = 'controller' | 'client'.\nRestart required")));
    console.registerCommand(ConsoleCommandD("espnow", [](int argc, char **argv) -> int {
        if (argc != 2) { printf_P(PSTR("Missing 'on' or 'off'\n")); return EXIT_FAILURE; }
        auto arg = String(argv[1]);
        if (arg == "on") { configEspNowEnabled = true; } else {  arg = "off"; configEspNowEnabled = false; }
        NVS.setInt("ESPNOW_ENABLE", (configEspNowEnabled) ? 1 : 0);

        printf("ESP-NOW config is '%s'\r\n", arg);
        return EXIT_SUCCESS;
    }, PSTR("Enable/Disable ESP-NOW wireless P2P link between controller and client ESP32 devices")));

    console.registerCommand(ConsoleCommandD("log", [](int argc, char **argv) -> int {
        if (argc != 2) { printf_P(PSTR("Missing 'on' or 'off'\n")); return EXIT_FAILURE; }
        auto arg = String(argv[1]);
        if (arg == "on") { consoleLogging = true; printf_P(PSTR("Logging enabled\n")); } else { consoleLogging = false; printf_P(PSTR("Logging disabled\n")); }
        return EXIT_SUCCESS;
    }, PSTR("Enable / Disable informational logging. ")));

    console.registerCommand(ConsoleCommandD("debug", [](int argc, char **argv) -> int {
        if (argc != 2) { printf_P(PSTR("Missing 'on' or 'off'\n")); return EXIT_FAILURE; }
        auto arg = String(argv[1]);
        if (arg == "on") { consoleLogging = true; debugAxisMovements = true; printf_P(PSTR("Debug enabled\n")); } else { consoleLogging = false; debugAxisMovements = false; printf_P(PSTR("Debug disabled\n")); }
        return EXIT_SUCCESS;
    }, PSTR("Enable / Disable debug logging. ")));

    console.registerCommand(ConsoleCommandD("getconfig", [](int argc, char **argv) -> int {
        readNvsConfig(false);
        return EXIT_SUCCESS;
    }, PSTR("Read environment configuration from NVS storage")));

    console.registerCommand(ConsoleCommandD("saveconfig", [](int argc, char **argv) -> int {
        if(saveNvsConfig()){
            setConsolePrompt(PURPLE, "R!");
            printf_P(PSTR("'restart' required to apply new config\r\n"));
            return EXIT_SUCCESS;
        }
        setConsolePrompt(RED, "!!");
        return EXIT_FAILURE;
    }, PSTR("Save environment configuration to NVS storage")));
    console.registerCommand(ConsoleCommandD("resetconfig", [](int argc, char **argv) -> int {
        if (argc != 2) { printf_P(PSTR("Missing 'yes' parameter\n")); return EXIT_FAILURE; }
        auto arg = String(argv[1]);
        if (arg == "yes") {
            if(resetNvsConfig()){
                setConsolePrompt(PURPLE, "R!");
                printf_P(PSTR("'restart' required to apply default config\r\n"));
                return EXIT_SUCCESS;
            }
        } else {
            printf("Arg '%s' is not valid\r\n", arg);
        }
        //setConsolePrompt(RED, "!!");
        return EXIT_FAILURE;
    }, PSTR("Clear/reset NVS configuration to defaults")));
    

    /* boardconfig command */
    OptionsConsoleCommand  boardconf("boardconfig", [](int argc, char **argv, ParseResult result, Options options) -> int {
            int type_index = -1;
            int num_steppers = -1;
            if (result.count("t")) {
                type_index = result["t"].as<int>();
            }
            if (type_index < 0 || (type_index > MAX_BOARD_TYPES) ) { printf("Invalid or missing board type (0>%d)\n", MAX_BOARD_TYPES); return EXIT_FAILURE;}
            
            if (result.count("n")) { num_steppers = result["n"].as<int>(); }
            if (num_steppers < 0 || num_steppers > 5) { printf_P(PSTR("Invalid number of steppers (0>6)\n")); return EXIT_FAILURE;}

            configBoardType = (board_type_t)type_index;            
            configBoardName = getBoardName(type_index);
            board_pin_config = board_pin_configs[configBoardType]; // Set to default values for new board type
            configNumSteppers = num_steppers;
            
            printf("Configuring Board Type: %d, Name: '%s', Num Steppers: %d\r\n", type_index, configBoardName.c_str(), num_steppers);

            saveNvsConfig(); // Save new values to config
            readNvsConfig(); // board_pin_config is set must read config to get global values
            
            setConsolePrompt(PURPLE, "R!");
            printf_P(PSTR("'restart' required to apply new config\r\n"));

            return EXIT_SUCCESS;
        }, PSTR("Configure stepper motors pins and parameters"));
    
    char board_help_string[512] = "Board type number = \n";
    for (uint8_t i = 0; i <= MAX_BOARD_TYPES; i++) {
        sprintf(board_help_string + strlen(board_help_string), " %d = '%s'\n",i,getBoardName(i).c_str());        
    }
    sprintf(board_help_string + strlen(board_help_string), ">");

    boardconf.options.add_options()("t,type", board_help_string, cxxopts::value<int>());
    boardconf.options.add_options()("n,numsteppers", "Number of enabled steppers (0-6)", cxxopts::value<int>());
    console.registerCommand(boardconf);
    /* end boardconf command */
    

    /* wificonfig command */
    OptionsConsoleCommand  wificonf("wificonfig", [](int argc, char **argv, ParseResult result, Options options) -> int {
            int mode = -1;
            std::string ssid;
            std::string password;
            bool hidessid = false;

            if (result.count("m")) { mode = result["m"].as<int>(); }
            if (mode < 0 || mode > 3 ) { printf_P(PSTR("Invalid or missing wifi mode (0>3)\n")); return EXIT_FAILURE;}
            
            if (result.count("s")) { ssid = result["s"].as<std::string>(); } 
            if (ssid.length()  < 2) { printf_P(PSTR("Invalid SSID. Length must be > 2\n")); return EXIT_FAILURE;}
            
            if (result.count("p")) { password = result["p"].as<std::string>(); }
            if (password.length()  == 0) { password = ""; }
            
            if (result.count("h")) { hidessid = result["h"].as<bool>(); }

            if (mode == 0) { // Wifi OFF
                printf_P(PSTR("Disabling WiFI\r\n"));
                WiFi.mode(WIFI_OFF);
                NVS.setInt("WIFI_M", mode);
                return EXIT_SUCCESS;
            } else if (mode == 1) { // Wifi Station
                printf_P(PSTR("WiFI Station Mode\r\n"));
                WiFi.mode(WIFI_MODE_STA);
            } else if (mode == 2) { // Wifi AP
                printf_P(PSTR("WiFI AP Mode\r\n"));
                WiFi.mode(WIFI_MODE_AP);
                WiFi.softAP(ssid.c_str(), password.c_str(),1,hidessid,4);
            } else if (mode == 3) { // Wifi Station + AP
                printf_P(PSTR("WiFI Station+AP Mode\r\n"));
                WiFi.mode(WIFI_MODE_APSTA);
            } else { 
                printf("Invalid wifi mode: '%d' \n", mode); return EXIT_FAILURE; 
            }
            if (mode == 1 || mode == 3) // STA or APSTA
                WiFi.begin(ssid.c_str(), password.c_str());
            
            NVS.setInt("WIFI_M", mode);
            NVS.setString("WIFI_SSID", ssid.c_str());
            NVS.setString("WIFI_PWD", password.c_str());
            NVS.setInt("WIFI_HIDE", hidessid);

            return EXIT_SUCCESS;
        }, PSTR("Configure WiFi mode and AP/STA settings"));


    wificonf.options.add_options()("m,mode", "WiFi Mode =\n 0 = OFF,\n 1 = STATION,\n 2 = AP,\n 3 = STATION+AP", cxxopts::value<int>());
    wificonf.options.add_options()("s,ssid", "SSID to connect to (STA) or host (AP)", cxxopts::value<std::string>());
    wificonf.options.add_options()("p,password", "Password", cxxopts::value<std::string>());
    wificonf.options.add_options()("h,hidessid", "SSID is hidden when in AP mode", cxxopts::value<bool>());
    console.registerCommand(wificonf);
    /* end wificonf command */

    /* inputconfig command */
    OptionsConsoleCommand  inputconf("inputconfig", [](int argc, char **argv, ParseResult result, Options options) -> int {
        int i_index = -1;
        if (result.count("i")) {
            i_index = result["i"].as<int>();
        }
        if (i_index < 0) { printf_P(PSTR("Invalid or missing input index (0>6)\n")); return EXIT_FAILURE;}
        if (argc < 4) { printf_P(PSTR("Missing setting to update\n")); return EXIT_FAILURE; }
        
        printf("Input %d: ", i_index);
        if (result.count("n")) {
            board_pin_config.inputConfigs[i_index].name = String(result["n"].as<std::string>().c_str());
            printf("Name: '%s' ", board_pin_config.inputConfigs[i_index].name.c_str());
        }
        if (result.count("g")) {
            board_pin_config.inputConfigs[i_index].gpio_number = (gpio_num_t) ((result["g"].as<int>() <= -1) ? GPIO_NUM_NC : result["g"].as<int>());
            printf("GPIO: %d ", board_pin_config.inputConfigs[i_index].gpio_number);
        }
        if (result.count("u")) {
            board_pin_config.inputConfigs[i_index].pullup = result["u"].as<bool>();
            printf("Pull Up: %d ", result["g"].as<bool>());
        }
        if (result.count("d")) {
            board_pin_config.inputConfigs[i_index].pulldown = result["d"].as<bool>();
            printf("Pull Down: %d ", result["d"].as<bool>());
        }
        if (result.count("r")) {
            board_pin_config.inputConfigs[i_index].register_address = (result["g"].as<int>() == 0) ? GPIO_IN_REG : GPIO_IN1_REG;
            printf("Register Bank: %s ", (board_pin_config.inputConfigs[i_index].register_address == GPIO_IN_REG ? "GPIO_IN_REG" : "GPIO_IN1_REG"));
        }
        if (result.count("b")) {
            board_pin_config.inputConfigs[i_index].register_bit = result["b"].as<int>();
            printf("Register Bit: %d ", board_pin_config.inputConfigs[i_index].register_bit);
        }

        printf(" updated. Must 'saveconfig' to apply\r\n", i_index);
        setConsolePrompt(PURPLE, "*");

        return EXIT_SUCCESS;
    }, PSTR("Configure Input pins and parameters"));

    inputconf.options.add_options()("i,index", "Input index number (0-6)", cxxopts::value<int>());
    inputconf.options.add_options()("n,name", "Input name (string)", cxxopts::value<std::string>());
    inputconf.options.add_options()("g,gpiopin", "GPIO Pin (-1=Disable, 0->128)", cxxopts::value<int>());
    inputconf.options.add_options()("u,pullup", "Enable Pullup Resistors (0|1)", cxxopts::value<bool>());
    inputconf.options.add_options()("d,pulldown", "Enable Down Resistors (0|1)", cxxopts::value<bool>());
    inputconf.options.add_options()("r,registerbank", "Input Register Bank (0=IN_REG, 1=IN1_REG)", cxxopts::value<int>());
    inputconf.options.add_options()("b,registerbit", "Input Register Bit Number (int)", cxxopts::value<int>());

    console.registerCommand(inputconf);
    /* end inputconf command */

    /* outputconfig command */
    OptionsConsoleCommand  outputconf("outputconfig", [](int argc, char **argv, ParseResult result, Options options) -> int {
        int i_index = -1;
        if (result.count("i")) {
            i_index = result["i"].as<int>();
        }
        if (i_index < 0) { printf_P(PSTR("Invalid or missing output index (0>6)\n")); return EXIT_FAILURE;}
        if (argc < 4) { printf_P(PSTR("Missing setting to update\n")); return EXIT_FAILURE; }
        
        printf("Output %d: ", i_index);
        if (result.count("n")) {
            board_pin_config.outputConfigs[i_index].name = String(result["n"].as<std::string>().c_str());
            printf("Name: '%s' ", board_pin_config.outputConfigs[i_index].name.c_str());
        }
        if (result.count("g")) {
            board_pin_config.outputConfigs[i_index].gpio_number = (gpio_num_t) ((result["g"].as<int>() <= -1) ? GPIO_NUM_NC : result["g"].as<int>());
            printf("GPIO: %d ", board_pin_config.outputConfigs[i_index].gpio_number);
        }

        printf(" updated. Must 'saveconfig' to apply\r\n", i_index);
        setConsolePrompt(PURPLE, "*");

        return EXIT_SUCCESS;
    }, PSTR("Configure Output pins and parameters"));

    outputconf.options.add_options()("i,index", "Output index number (0-6)", cxxopts::value<int>());
    outputconf.options.add_options()("n,name", "Output name (string)", cxxopts::value<std::string>());
    outputconf.options.add_options()("g,gpiopin", "GPIO Pin (-1=Disable, 0->128)", cxxopts::value<int>());


    console.registerCommand(outputconf);
    /* end outputconf command */

    console.registerCommand(ConsoleCommandD("firmwareupdate", [](int argc, char **argv) -> int {
        printf("Coming soon!\r\n");
        return EXIT_SUCCESS;
    }, "Perform firmware update via WiFi from online repository"));

    console.registerCommand(ConsoleCommandD("uptime", [](int argc, char **argv) -> int {
        printf("Uptime: %ds\r\n", esp_log_timestamp()/1000);
        return EXIT_SUCCESS;
    }, "System uptime in seconds"));

    console.registerCommand(ConsoleCommandD("stop", [](int argc, char **argv) -> int {
        consoleLogging = true;
        stopProcessing();
        setConsolePrompt(YELLOW, "STOP");
        return EXIT_SUCCESS;
    }, PSTR("Stop processing task loops")));
    
#if ESP32_SPI_ETHERNET

    /* spiethconfig command */

    OptionsConsoleCommand  spiethconf("spiethconfig", [](int argc, char **argv, ParseResult result, Options options) -> int {
            uint8_t miso_pin = -1;
            uint8_t mosi_pin = -1;
            uint8_t sck_pin = -1;
            uint8_t cs_pin = -1;
            uint8_t int_pin = -1;

            if (result.count("s")) { miso_pin = result["s"].as<int>(); }
            if (miso_pin < 0 || miso_pin > 128 ) { printf_P(PSTR("Invalid or missing MISO pin (0>128)\n")); return EXIT_FAILURE;}
            
            if (result.count("i")) { miso_pin = result["i"].as<int>(); }
            if (mosi_pin < 0 || mosi_pin > 128 ) { printf_P(PSTR("Invalid or missing MOSI pin (0>128)\n")); return EXIT_FAILURE;}

            if (result.count("c")) { miso_pin = result["c"].as<int>(); }
            if (sck_pin < 0 || sck_pin > 128 ) { printf_P(PSTR("Invalid or missing SCK pin (0>128)\n")); return EXIT_FAILURE;}

            if (result.count("e")) { miso_pin = result["e"].as<int>(); }
            if (cs_pin < 0 || cs_pin > 128 ) { printf_P(PSTR("Invalid or missing Chip Select pin (0>128)\n")); return EXIT_FAILURE;}

            if (result.count("t")) { miso_pin = result["t"].as<int>(); }
            if (int_pin < 0 || int_pin > 128 ) { printf_P(PSTR("Invalid or missing Interrupt pin (0>128)\n")); return EXIT_FAILURE;}

            configSpiMosiPin = mosi_pin;
            configSpiMisoPin = miso_pin;
            configSpiSckPin = sck_pin;
            configSpiCsPin = cs_pin;
            configSpiIntPin = int_pin;

            NVS.setInt("SPI_MISO", configSpiMisoPin);
            NVS.setInt("SPI_MOSI", configSpiMosiPin);
            NVS.setInt("SPI_SCK", configSpiSckPin);
            NVS.setInt("SPI_CS", configSpiCsPin);
            NVS.setInt("SPI_INT", configSpiIntPin);
            
            printf("Set SPI Ethernet Pin config to MISO: %d, MOSI: %d, SCK: %d, CS: %d, INT: %d\r\n", miso_pin, mosi_pin, sck_pin, cs_pin, int_pin);
            
            setConsolePrompt(PURPLE, "R!");
            printf_P(PSTR("'restart' required to apply new config\r\n"));

            return EXIT_SUCCESS;
        }, PSTR("SPI Ethernet Pin Configuration."));


    spiethconf.options.add_options()("s,miso", "MISO pin", cxxopts::value<int>());
    spiethconf.options.add_options()("i,mosi", "MOSI pin", cxxopts::value<int>());
    spiethconf.options.add_options()("c,sck", "SCK pin", cxxopts::value<int>());
    spiethconf.options.add_options()("e,csenable", "Chip Select Enable pin", cxxopts::value<int>());
    spiethconf.options.add_options()("t,interrupt", "Interrupt pin", cxxopts::value<int>());
    console.registerCommand(spiethconf);
    /* end spiethconfig command */

#endif
    registerDebuggingCmds();
    if (!doMotorConfig && configMode == MODE_CONTROLLER && !safeMode)
        setConsolePrompt(YELLOW, "!MOTORS");
}

