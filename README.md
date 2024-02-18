# ESP32UDP - ESP32 & RMII/SPI Ethernet LinuxCNC Real time High Speed Smooth Motion Controller - 200khz Step Pulse Frequency Multi Axis
Hardware based external step generator and IO interface for LinuxCNC over native RMII Ethernet OR W5500 SPI Ethernet on a dual core ESP32 for almost any board.<br>
The hardware is connected to LinuxCNC via Ethernet.<br>
The controller operates in position mode at low speed and at higher speeds in velocity mode.<br>


### Credits
* Credit goes to Juhász Zoltán for his original great work and concept for the HAL2UDP components using W5500 SPI ethernet and software-based Step generation on ESP32.
This project was originally based on it but the firmware source hardly resembles any of it now. GPL licensing is given in the source header.
* gin66 et al. for creating the truly marvelous FastAccelStepper library: https://github.com/gin66/FastAccelStepper


### Features
* Based heavily on Arduino libraries to allow easy customizations or integrations
* Uses native ESP32 RMII Ethernet for high performance and low latency
* **ENABLE** **STEP** and **DIR** signals for up-to 6 axes (with careful planning with GPIO pins)
* 7 input pins
* 7 output pins, any can be pwm signal
* Step frequency up to 200 kHz total using RMT peripheral. Can drive 6 motors reliably, smoothly and accurately across all acceleration profiles
* Extensible: Spare core #1 for Arduino tasks or extra libraries
* Core 0 handles UDP Server & Client, Commands, Motor step generation, position and feedback control
* WiFI working with OTA updates and Telnet server for remote monitoring. OTA firmware updates can be performed via Ethernet too to speed up development iterations if using a Virtual LinuxCNC instance
* Serial and Telnet interface with stepper and input status stats
* Detailed logging output on serial interface (to be improved and easier to visualize any board setup or config errors later)
* Handles most setup problems and reports on the Serial console if there are any problems
* Event driven real-time UDP networking protocol with sleeping when idle and error handling freeing resources on ESP32
* Inputs pin status and triggers are handled by high speed interrupt handler and reading of GPIO input registers
* ESP32 Pulse Counter peripheral is unused allowing for Servo Encoder feedback connection in future. Motor position reporting back to LinuxCNC PID controller should be straight forward
* Multiple board support with native RMII ethernet modules or use W5500 SPI Ethernet module. Example boards with RMII PHYs (Native Ethernet): WT32-ETH01, Olimex ESP32-EVB, ESP32-Gateway, ESP32-POE
* Future scope - ESP32-S2/S3 4 Axis USB based networking (USB RNDIS/ECM Ethernet) so no more Ethernet adapters or PHYs. A single $5 ESP32-S2 or S3 module with USB-C will handle all LinuxCNC real-time comms and high-speed and accurate motor driving (yes, it is possible!)
* I2S Output on MKS-DLC32 3 (or 4) axis CNC board. Direction and Enable pins are provided by I2S. Step Pins are connected to the EXP1 header which can be wired to each axis 4 pin header if theres a need to use the PCB stepper driver modules. W5500 SPI Ethernet is connected to EXP2 and two I2C pins on header. Documentation updates coming soon

### Major Refactor - Feb 2024
* Breaking change - modified UDP server and client port on Controller to 58000. Requires recompile of HAL component detailed below
* Implement fully featured console command line on both Serial and Telnet interfaces for board config and status
* Unify code to support single firmware images for all ESP32 boards
* Two firmware types - SPI Ethernet, RMII Native Ethernet. In future 3rd for USB S2/S3
* Combined controller "Server" and ESP-NOW "Client" into single project - configurable at runtime via CLI
* Split codebase into separate files for respective features
* Boot loop detection & safe mode CLI with Ethernet support for OTA flashing
* Configuration manager implementation using NVS flash storage. Config retention between firmware updates and restarts
* Default generic pinmaps for board types based on 'reasonable' available pins
* Handle default RMII Phy pin config per board type. 
* Configure SPI pins for SPI Ethernet module via CLI. Supports any board
* Input PIN configuration per board type via CLI
* Output PIN configuration per board type via CLI
* Configure stepper motor pin maps and motor settings, per board type at runtime via CLI
* Firmware updates via either Arduino standard OTA or pull latest from compiled github releases including in safe mode. See 'firmware' command
* Add Wokwi ESP32 simulator to support rapid development. Install via VSCode Extensions. Requires club membership to use Private gateway
* Wokwi sim uses "private gateway" to connect LinuxCNC to virtual ESP32 over UDP. Latency is high but sufficient to perform basic comms tests. <wokwigw --forward udp:58000:10.13.37.2:58000>
* Implement speedy GDB debugging setup via Wokwi simulator
* See console 'help' command or docs on how to configure each board

### NOTES
* If you're experiencing problems with serial console ensure you 'restart' the board before issuing commands. Initial ASCII escape codes are sent to console during setup. TODO implement handling Serial RTS/DTS and send ASCII codes on connect.
* See SerialCommands.md for basic serial console usage and board setup

### TODO
* ~~Refactor code splitting into several files~~
* ~~Optimize IRAM cache hit ratios~~
* ~~Improve logging and debugging~~
* ~~Reduce RAM usage and manage infrequent strings better (PROGMEM)~~
* Implement hardware based Encoder Quadrature incremental for counting for up-to 4 axes
* Support I2C GPIO mux and interrupt feature to free up built-in GPIO pins to support 6 axes across many boards
* Add Servo closed-loop PID control loops with LinuxCNC control
* ~~Add SPI Ethernet support (WizNet W5500) and optimize for low latency in the event someone wants to use it in their design~~ DONE
* Add USB-C Device RNDIS/ECM networking support on the ESP32-S2/S3 MCUs
* Document all serial console commands and how they are used

### Precompiled Firmware
Two precompiled builds exist - One for SPI Ethernet and one for Native RMII Ethernet<br>
Download required firmware from github release section in this repo and flash using standard ESP32 tools<br>
Use the serial command line interface to configure your board type 'boardconf' or use one as a template and configure your own motors, inputs and outputs using the CLI<br>

### Install & Build
Clone this repository and open it with platformIO in Visual Studio Code.<br>
See Config.h for example board types choose a Environment for your ethernet type and flash<br>

### Settings
On LinuxCNC Host<br>
Set your ethernet NIC that ESP32 is connected to 192.168.111.2/24 or any address other than 192.168.111.1 within the /24 subnet<br>
ESP32 has a static ip of 192.168.111.1. This can be changed in Config.h<br>

### LinuxCNC driver
```bash
sudo apt-get install linuxcnc-uspace-dev build-essential
```
```bash
sudo halcompile --install esp32udp.comp
```
### Hardware
Any ESP-WROOM/WROVER-32 board with RMII Ethernet Phy (e.g LAN8720) 

Ideal pin mappings and available IO per board will be documented soon

### LinuxCNC HAL pins
esp32udp.0.position_cmd (in - float) commanded position in position units
esp32udp.1.position_cmd
esp32udp.2.position_cmd

esp32udp.0.velocity_cmd (in - float) commanded velocity in position units/s
esp32udp.1.velocity_cmd
esp32udp.2.velocity_cmd

esp32udp.0.position_fb (out - float) feedback position in position units
esp32udp.1.position_fb
esp32udp.2.position_fb

esp32udp.0.velocity_fb (out - float) feedback velocity in position units/s
esp32udp.1.velocity_fb
esp32udp.2.velocity_fb

esp32udp.out.00 (in - bit) digital output
esp32udp.out.01
esp32udp.out.02
esp32udp.out.03
esp32udp.out.04
esp32udp.out.05

esp32udp.pwm.00 (in - float) PWM output 0...1
esp32udp.pwm.01
esp32udp.pwm.02
esp32udp.pwm.03
esp32udp.pwm.04
esp32udp.pwm.05

esp32udp.in.00 (out - bit) digital input
esp32udp.in.01
esp32udp.in.02
esp32udp.in.03
esp32udp.in.04
esp32udp.in.05
esp32udp.in.06

esp32udp.ready 			 		(out - bit) module state
esp32udp.enable 		 		(in - bit) module enable
esp32udp.packets_lost_tx 		(out - s32) lost TX packets
esp32udp.last_pkt_rx_time_ms	(out - s32) Last Packet RX Time in MS (up-to 5s when idle) 
esp32udp.send_pkt				(out - s32) TX Packet queue counter 
esp32udp.udp_seq_number		    (out - s32) UDP sequence number from ESP32, useful for monitoring 

### LinuxCNC HAL parameters

esp32udp.0.scale (rw - float) steps per position unit
esp32udp.1.scale
esp32udp.2.scale

esp32udp.0.accel (rw - float) acceleration in position units/s<sup>2</sup>
esp32udp.1.accel
esp32udp.2.accel

esp32udp.pwm.00.freq (rw - u32) PWM frequency in Hz 0..65000
esp32udp.pwm.01.freq
esp32udp.pwm.02.freq
esp32udp.pwm.03.freq
esp32udp.pwm.04.freq
esp32udp.pwm.05.freq

### PWM usage
If the esp32udp.pwm.xx.freq parameter is set to 0, the esp32udp.out.xx pin works and the esp32udp.pwm.xx pin does not.<br>
If the value of the esp32udp.pwm.xx.freq parameter is not 0, the esp32udp.out.xx pin does not work and the esp32udp.pwm.xx pin does.<br>