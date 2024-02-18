## RMII Ethernet
All native ethernet boards with RMII Phy onboard<br>

### ESP32-POE
<https://www.olimex.com/Products/IoT/ESP32/ESP32-POE/open-source-hardware>
To configure the board with 3 stepper motors use `boardconfig -t 2 -n 3`<br>
Default, but configurable at runtime pin maps are in Config.h<br>
Reserved Ethernet PHY GPIOs: GPIO12, GPIO23, GPIO18, GPIO17<br>

### ESP32-EVB
<https://www.olimex.com/Products/IoT/ESP32/ESP32-EVB/open-source-hardware>
To configure the board with 3 stepper motors use `boardconfig -t 3 -n 3`<br>
Default, but configurable at runtime pin maps are in Config.h<br>
Reserved Ethernet PHY GPIOs: GPIO23, GPIO18, GPIO0<br>


### ESP32-GATEWAY
<https://www.olimex.com/Products/IoT/ESP32/ESP32-GATEWAY/open-source-hardware>
To configure the board with 3 stepper motors use `boardconfig -t 4 -n 3`<br>
Default, but configurable at runtime pin maps are in Config.h<br>
Reserved Ethernet PHY GPIOs: GPIO23, GPIO18, GPIO0<br>

### ESP32-WT32-ETH01
<https://en.wireless-tag.com/product-item-2.html>
<https://github.com/egnor/wt32-eth01?tab=readme-ov-file>
To configure the board with 3 stepper motors use `boardconfig -t 5 -n 3`<br>
Default, but configurable at runtime pin maps are in Config.h<br>
Reserved Ethernet PHY GPIOs: GPIO16, GPIO23, GPIO18, GPIO0<br>

### Others
Any ESP32 board that has an Ethernet phy can be supported<br>

## SPI Ethernet
All SPI Ethernet boards<br>

### MKS-DLC32
<https://github.com/makerbase-mks/MKS-DLC32>
Board configuration template is available in Config.h<br>
To configure the board with 4 stepper motors use `boardconfig -t 6 -n 4`<br>

4 Axis stepper motors pins defaults are:<br>
# X Axis:
* Step =      **LCD_CS_0 (GPIO25)**      EXP1 header pin #7
* Direction = **X_DIR (I2S)**            Existing pin labelled as "D" on the 4x1 pin header below X axis
* Enable =    **Shared (I2S)**           Any existing stepper pins labelled as "E" on the 4x1 pin headers below each driver connection
# Y Axis:
* Step =      **LCD_TOUCH_CS (GPIO26)**  EXP1 header pin #5
* Direction = **Y_DIR (I2S)**            Existing pin labelled as "D" on the 4x1 pin header below X axis
* Enable =    **Shared (I2S)**           Any existing stepper pins labelled as "E" on the 4x1 pin headers below each driver connection
# Z Axis:
* Step =      **LCD_RST_0 (GPIO27)**     EXP1 header pin #4
* Direction = **Z_DIR (I2S)**            Existing pin labelled as "D" on the 4x1 pin header below X axis
* Enable =    **Shared (I2S)**           Any existing stepper pins labelled as "E" on the 4x1 pin headers below each driver connection
# A Axis:
* Step =      **LCD_EN_0 (GPIO5)**       EXP1 header pin #3
* Direction = **LCD_RS (GPIO33)**        EXP1 header pin #8
* Enable =    **Shared (I2S)**           Any existing stepper pins labelled as "E" on the 4x1 pin headers below each driver connection


Wire the SPI Ethernet W5500 module to the pins below:<br>

![alt text](https://github.com/wezhunter/ESP32_LinuxCNC_MotionController_RealTime/docs/MKS-DLC32-W5500-Wiring.png)