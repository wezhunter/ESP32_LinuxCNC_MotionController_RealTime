#pragma once

#ifndef CONFIG_H
#define CONFIG_H


#include <Arduino.h>
#include "FastAccelStepper.h"
#include "I2SOut.h"

#if ESP32_RMII_ETHERNET
  #include "Eth.h"
#endif


/*==================================================================*/
// Serial baud rate
// OK to change, but the ESP32 boot text is 115200, so you will not see that is your
// serial monitor, sender, etc uses a different value than 115200
#define BAUD_RATE 115200
#ifdef MAX_STEPPER
  #undef MAX_STEPPER
#endif
#define MAX_STEPPER 6
#define MAX_BOARD_TYPES 7
#define MAX_INPUTS 7
#define MAX_OUTPUTS 7

//#define CONF_NUM_STEPPERS 3  /* Set how many motors you want to enable and update the stepper_config struct below with the motors you require */
/* !!!!Important!!!! Do not update the MAX_STEPPER define - this is set to 6 max and the size of the UDP RX/TX buffer is set accordingly to support up-to 6 max */

/* Ethernet config and IP Addressing */
const static uint8_t   ethernet_mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
const static IPAddress ethernet_ip(192, 168, 111, 1); /* ESP32 Ethernet IP Address */
const static IPAddress ethernet_ip_host(192, 168, 111, 2); /* LinuxCNC Ethernet adapter IP must be configured as this */
const static IPAddress ethernet_gw(192, 168, 111, 254); /* Only useful if you connect ESP32 and LinuxCNC Host on same network segment with a router or another network */
const static IPAddress ethernet_subnetmask(255, 255, 255, 0);

/* Async UDP Client and Server is used to ensure bi-directional low-latency data streaming between ESP32 and LinuxCNC Host */
const static uint16_t udpServerPort = 58000;  /* UDP Server port the ESP32 listens on. LinuxCNC HAL driver sends data to this port */
const static uint16_t udpClientPort = 58001; /* UDP Client port the ESP32 connects to. LinuxCNC HAL driver runs a UDP server */


#define IO_00 0b00000001
#define IO_01 0b00000010
#define IO_02 0b00000100
#define IO_03 0b00001000
#define IO_04 0b00010000
#define IO_05 0b00100000
#define IO_06 0b01000000
#define IO_07 0b10000000


#if ESP32_SPI_ETHERNET
  #define ASYNC_UDP_ESP32_ETHERNET_DEBUG_PORT      Serial
  
  #define ESP32_Ethernet_onEvent            ESP32_W5500_onEvent
  #define ESP32_Ethernet_waitForConnect     ESP32_W5500_waitForConnect
  #define ETH_SPI_HOST                      VSPI_HOST

  #define W5500_CS_PIN                GPIO_NUM_0 
  #define W5500_INT_PIN               GPIO_NUM_4 

#endif


/* I2S pins set. ** Only used if configBoardType == MKS-DLC32. Not wrapped in #ifdef as used at runtime based on 'boardconfig' type console config */
#define I2S_OUT_BCK                 GPIO_NUM_16 /* ONLY used if configBoardType == MKS-DLC32 */
#define I2S_OUT_WS                  GPIO_NUM_17 /* ONLY used if configBoardType == MKS-DLC32 */
#define I2S_OUT_DATA                GPIO_NUM_21 /* ONLY used if configBoardType == MKS-DLC32 */

typedef enum  { 
    MODE_CONTROLLER = 0, 
    MODE_CLIENT 
} config_mode_t;


typedef enum {
    BOARD_TYPE_NONE = 0,            /* Basic board, no ethernet, wifi only */
    BOARD_TYPE_ESP32_WOKWI_SIMUL,   /* WOKWI Simulator */
    BOARD_TYPE_ESP32_POE,           /* POE */
    BOARD_TYPE_ESP32_EVB,           /* EVB */
    BOARD_TYPE_ESP32_GATEWAY,       /* GATEWAY */
    BOARD_TYPE_ESP32_WT32_ETH01,    /* WT32_ETH01 */
    BOARD_TYPE_ESP32_MKSDLC32       /* MKS_DLC32 */
} board_type_t;

typedef struct {
  uint8_t step = PIN_UNDEFINED;
  uint8_t direction = PIN_UNDEFINED;
  uint8_t enable_low_active = PIN_UNDEFINED;
  uint8_t enable_high_active = PIN_UNDEFINED;
  uint16_t dir_change_delay = 1000;
  bool direction_high_count_up = true;
  bool auto_enable = false;
  uint32_t on_delay_us = 2000;
  uint16_t off_delay_ms = 5;
} stepper_config_t;

typedef struct  {
    String name = "Unused";
    bool pullup = true;
    bool pulldown = false;
    gpio_num_t gpio_number = GPIO_NUM_NC; /* Use GPIO_NUM_NC for not connected pins/unused inputs */ 
    int register_address = GPIO_IN_REG;
    int register_bit = 0;
} inputpin_config_t;

typedef struct  {
    String name = "Unused";
    int16_t gpio_number = GPIO_NUM_NC; /* Use GPIO_NUM_NC for not connected pins/unused outputs */ 
} outputpin_config_t;

typedef struct  {
    board_type_t board_type = BOARD_TYPE_NONE;
    uint8_t num_steppers = 0;
    stepper_config_t stepperConfig[MAX_STEPPER] = { 0 };
    inputpin_config_t inputConfigs[MAX_INPUTS] = {  };
    outputpin_config_t outputConfigs[MAX_OUTPUTS] = {  };

} board_pinconfig_t;


/* Global vars */
inline board_pinconfig_t board_pin_config; // Global for current board configuration store


/* ETH PHY RESERVED GPIO
     12, 17, 18, 19, 21, 22, 23, 25, 26
    INPUT ONLY GPIO
     34, 35, 36, 39
     BOOTSTRAP GPIO
     0, 2
*/


inline const board_pinconfig_t board_pin_configs[MAX_BOARD_TYPES] = {
    { .board_type=BOARD_TYPE_NONE},
    { .board_type=BOARD_TYPE_ESP32_WOKWI_SIMUL, .num_steppers=3,
        .stepperConfig = {  
                            { .step=23, .direction=22, .enable_low_active=5, .enable_high_active=PIN_UNDEFINED }, 
                            { .step=19, .direction=18,  .enable_low_active=5, .enable_high_active=PIN_UNDEFINED }, 
                            { .step=17, .direction=16, .enable_low_active=5, .enable_high_active=PIN_UNDEFINED }, 
                        },
        .inputConfigs = {
                            {  },
                            {  },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC  },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                            {  .pullup=true, .gpio_number=GPIO_NUM_34, .register_address=GPIO_IN1_REG, .register_bit=BIT2 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_35, .register_address=GPIO_IN1_REG, .register_bit=BIT3 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                        },
        .outputConfigs = {  
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                        }
    },

    { .board_type=BOARD_TYPE_ESP32_POE, .num_steppers=3,
        .stepperConfig = {  
                            { .step=33, .direction=32, .enable_low_active=13, .enable_high_active=PIN_UNDEFINED }, 
                            { .step=14, .direction=4,  .enable_low_active=13, .enable_high_active=PIN_UNDEFINED }, 
                            { .step=15, .direction=5,  .enable_low_active=13, .enable_high_active=PIN_UNDEFINED }, 
                        },
        .inputConfigs = {
                            {  },
                            {  },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC  },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                            {  .pullup=true, .gpio_number=GPIO_NUM_34, .register_address=GPIO_IN1_REG, .register_bit=BIT2 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                        },
        .outputConfigs = {  
                            { .gpio_number=GPIO_NUM_0 },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                        }
    },

    { .board_type=BOARD_TYPE_ESP32_EVB, .num_steppers=3,
        .stepperConfig = {  
                            { .step=17, .direction=17, .enable_low_active=PIN_UNDEFINED, .enable_high_active=32 }, 
                            { .step=14, .direction=4,  .enable_low_active=PIN_UNDEFINED, .enable_high_active=32 }, 
                            { .step=15, .direction=33, .enable_low_active=PIN_UNDEFINED, .enable_high_active=32 }, 
                        },
        .inputConfigs = {
                            {  },
                            {  },
                            {  .pullup=true, .gpio_number=GPIO_NUM_16, .register_bit=BIT16 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_39, .register_address=GPIO_IN1_REG, .register_bit=BIT7 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_34, .register_address=GPIO_IN1_REG, .register_bit=BIT2 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_35, .register_address=GPIO_IN1_REG, .register_bit=BIT3 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_36, .register_address=GPIO_IN1_REG, .register_bit=BIT4 },
                        },
        .outputConfigs = {  
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_33 },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                        }
    },

    { .board_type=BOARD_TYPE_ESP32_GATEWAY, .num_steppers=3,
        .stepperConfig = {  
                            { .step=17, .direction=17, .enable_low_active=PIN_UNDEFINED, .enable_high_active=32 }, 
                            { .step=14, .direction=4,  .enable_low_active=PIN_UNDEFINED, .enable_high_active=32 }, 
                            { .step=15, .direction=33, .enable_low_active=PIN_UNDEFINED, .enable_high_active=32 }, 
                        },
        .inputConfigs = {
                            {  },
                            {  },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                            {  .pullup=true, .gpio_number=GPIO_NUM_34, .register_address=GPIO_IN1_REG, .register_bit=BIT2 },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                            {  .pullup=true, .gpio_number=GPIO_NUM_NC },
                        },
        .outputConfigs = {  
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                        }
    },

    { .board_type=BOARD_TYPE_ESP32_WT32_ETH01, .num_steppers=4,
        .stepperConfig = {  
                            { .step=12, .direction=17, .enable_low_active=5, .enable_high_active=PIN_UNDEFINED }, 
                            { .step=14, .direction=4,  .enable_low_active=5, .enable_high_active=PIN_UNDEFINED }, 
                            { .step=15, .direction=33, .enable_low_active=5, .enable_high_active=PIN_UNDEFINED }, 
                            { .step=2,  .direction=32, .enable_low_active=5, .enable_high_active=PIN_UNDEFINED }
                        },
        .inputConfigs = {
                            {  },
                            {  },
                            { .pullup=true, .gpio_number=GPIO_NUM_NC },
                            { .pullup=true, .gpio_number=GPIO_NUM_NC },
                            { .pullup=true, .gpio_number=GPIO_NUM_NC },
                            { .pullup=true, .gpio_number=GPIO_NUM_NC },
                            { .pullup=true, .gpio_number=GPIO_NUM_NC },
                        },
        .outputConfigs = {  
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                        }
    },

    { .board_type=BOARD_TYPE_ESP32_MKSDLC32, .num_steppers=4,
        .stepperConfig = {  
                            { .step=25, .direction=I2SO(2), .enable_low_active=I2SO(0), .enable_high_active=PIN_UNDEFINED }, // STEP=LCD_CS_0, DIR=X_DIR
                            { .step=26, .direction=I2SO(6),  .enable_low_active=I2SO(0), .enable_high_active=PIN_UNDEFINED }, // STEP=LCD_TOUCH_CS, DIR=Y_DIR
                            { .step=27, .direction=I2SO(4), .enable_low_active=I2SO(0), .enable_high_active=PIN_UNDEFINED }, // STEP=LCD_RST_0, DIR=Z_DIR
                            { .step=5, .direction=33, .enable_low_active=I2SO(0), .enable_high_active=PIN_UNDEFINED }, // STEP=LCD_EN_0, DIR=LCD_RS
                        },
        .inputConfigs = {
                            {  },
                            {  },
                            {  },
                            { .name="ProbeIn", .pullup=true, .gpio_number=GPIO_NUM_22, .register_address=GPIO_IN_REG,  .register_bit=BIT22 },  // Probe in
                            { .name="Z-", .pullup=true, .gpio_number=GPIO_NUM_34, .register_address=GPIO_IN1_REG, .register_bit=BIT2 },   // Z- IN
                            { .name="Y-", .pullup=true, .gpio_number=GPIO_NUM_35, .register_address=GPIO_IN1_REG, .register_bit=BIT3 },   // Y- IN
                            { .name="X-", .pullup=true, .gpio_number=GPIO_NUM_36, .register_address=GPIO_IN1_REG, .register_bit=BIT4 },   // X- IN
                        },
        .outputConfigs = {  
                            { .name="BlueLed", .gpio_number=GPIO_NUM_2 }, // Machine on = Blue LED
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .gpio_number=GPIO_NUM_NC },
                            { .name="PWMSpindle", .gpio_number=GPIO_NUM_32 }, // PWM "Spindle" output
                            { .name="Beeper", .gpio_number=I2SO(7) }, // BEEPER
                        }
    }
        
};




#if ESP32_RMII_ETHERNET

    typedef struct  {
        board_type_t board_type = BOARD_TYPE_NONE;
        int8_t address = -1;
        int8_t power_enable_pin = -1;
        int8_t phy_mdc_pin = -1;
        int8_t phy_mdio_pin = -1;
        eth_clock_mode_t phy_clk_mode = ETH_CLOCK_GPIO0_IN;
        eth_phy_type_t phy_type = ETH_PHY_LAN8720;

    } ethernet_phy_pinconfig_t;

    inline ethernet_phy_pinconfig_t eth_phy_configs[6] = {
        {.board_type=BOARD_TYPE_NONE}, 
        {.board_type=BOARD_TYPE_ESP32_WOKWI_SIMUL},
        {.board_type=BOARD_TYPE_ESP32_POE, .address=0, .power_enable_pin=12, .phy_mdc_pin=23, .phy_mdio_pin=18, .phy_clk_mode=ETH_CLOCK_GPIO17_OUT, .phy_type=ETH_PHY_LAN8720},
        {.board_type=BOARD_TYPE_ESP32_EVB, .address=0, .power_enable_pin=-1, .phy_mdc_pin=23, .phy_mdio_pin=18, .phy_clk_mode=ETH_CLOCK_GPIO0_IN, .phy_type=ETH_PHY_LAN8720},
        {.board_type=BOARD_TYPE_ESP32_GATEWAY, .address=0, .power_enable_pin=-1, .phy_mdc_pin=23, .phy_mdio_pin=18, .phy_clk_mode=ETH_CLOCK_GPIO0_IN, .phy_type=ETH_PHY_LAN8720},
        {.board_type=BOARD_TYPE_ESP32_WT32_ETH01, .address=1, .power_enable_pin=16, .phy_mdc_pin=23, .phy_mdio_pin=18, .phy_clk_mode=ETH_CLOCK_GPIO0_IN, .phy_type=ETH_PHY_LAN8720 }
    };

#endif


#endif