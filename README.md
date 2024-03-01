## ESP32UDP - ESP32 + Native RMII or SPI Ethernet LinuxCNC Real time High Speed Smooth Motion Controller - 200khz Step Pulse Frequency for Multi Axis solutions
Hardware based external step generator and IO interface for LinuxCNC over native RMII Ethernet or W5500 SPI Ethernet on a dual core ESP32 for almost any board.<br>
The hardware is connected to LinuxCNC via Ethernet.<br>
The controller operates in position mode at low speed and at higher speeds in velocity mode.<br>

Reliable high speed, high precision motion multi-axis motion in hardware - 80khz on a single stepper!
It screams...

[![Ultra Speed Motion](https://img.youtube.com/vi/KoUIp38FQs0/0.jpg)](https://youtu.be/KoUIp38FQs0)


### NOTE - 28th Feb 2024
There is a work in progress release which is currently being worked on offline due to the nature of rapid development and size of changes. 
The new release adds smooth motion on ramp up, ESP32-S2 & S3 USB support and many bug fixes.
It's advised to wait until that is released before experimenting with this

Board, motor and IO pin config via the serial console is deprecated in the next release in favour of a responsive WebUI. Basic serial console commands will remain for troubleshooting purposes.
Configuration Export & Import to JSON will exist

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
* Future scope - ESP32-S2/S3 (4 or 8) Axis USB based networking (USB RNDIS/ECM Ethernet) so no more Ethernet adapters or PHYs. A single $5 ESP32-S2 or S3 module with USB-C will handle all LinuxCNC real-time comms and high-speed and accurate motor driving (yes, it is possible!)
* I2S Output on MKS-DLC32 3 (or 4) axis CNC board. Direction and Enable pins are provided by I2S. Step Pins are connected to the EXP1 header which can be wired to each axis 4 pin header if theres a need to use the PCB stepper driver modules. W5500 SPI Ethernet is connected to EXP2 and two I2C pins on header. See Config.h for the default 4 axis pin configuration or here

### Major Refactor - Feb 2024
* Breaking change - modified UDP server and client port on Controller to 58000. Requires recompile of HAL component detailed below
* Implement fully featured console command line on both Serial and Telnet interfaces for board config and status
* Unify code to support single firmware images for all ESP32 boards
* Two firmware types - SPI Ethernet, RMII Native Ethernet. In future 3rd for USB S2/S3
* Combined controller "Server" and ESP-NOW "Client" into single project - configurable at runtime via CLI
* Split codebase into separate files for respective features
* Boot loop detection & safe mode CLI with Ethernet support for OTA flashing
* Configuration manager implementation using NVS flash storage. Config retention between firmware updates and restarts
* Default generic pin maps for board types based on 'reasonable' available pins
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

### DISCLAIMER
This is currently an experimental firmware and should NOT be used on any production machinery. If using on a machine then you should be prepared for it to break or obliterate your machine, unless you've benchtested it sufficiently. I accept no responsibility for any damages or injuries caused as a result of using this project. Use at your own risk.

### NOTES
* If you're experiencing problems with serial console ensure you 'restart' the board before issuing commands. Initial ANSI escape codes are sent to console during setup. TODO implement handling Serial RTS/DTS and send ANSI escape codes on connect. This will be resolved soon
* See `SerialCommands.md` for basic serial console usage and board setup

### Wiring Guides & Board Type Config
Please see `/docs/Wiring.md` for wiring<br>
Serial Console doc is `SerialCommands.md`<br>

### TODO
Please see Github Issues for an up-to date list<br>

### Reporting Problems or Providing Feedback
Please report any issues using github Issues and they should typically be responded to within 24 hours<br>

### Feature Requests or Pull Requests
For feature requests or ideas please create an Issue<br>
If contributing then please fork and create a appropriately named branch in your own repo<br>
Submit a Pull Request and will be merged if it tests out fine<br>

### Precompiled Firmware
Two precompiled builds exist - One for SPI Ethernet and one for Native RMII Ethernet<br>
Download required firmware from github release section in this repo and flash using standard ESP32 tools<br>
Use the serial command line interface to configure your board type 'boardconf' or use one as a template and configure your own motors, inputs and outputs using the CLI<br>

### Install & Build
Clone this repository and open it with platformIO in Visual Studio Code.<br>
See Config.h for example board types choose a Environment for your ethernet type and flash<br>

### LinxCNC Settings
On LinuxCNC Host<br>
Set your ethernet NIC that ESP32 is connected to 192.168.111.2/24 or any address other than 192.168.111.1 within the /24 subnet<br>
ESP32 has a static ip of 192.168.111.1. This can be changed in Config.h<br>


### Hardware
Any ESP-WROOM/WROVER-32 board with RMII Ethernet Phy (e.g LAN8720) or a W5500 SPI Ethernet module attached to SPI interface with including Interrupt pin


### LinuxCNC driver
```bash
sudo apt-get install linuxcnc-uspace-dev build-essential
```
```bash
sudo halcompile --install esp32udp.comp
```


### LinuxCNC HAL pins
esp32udp.0.position_cmd (in - float) commanded position in position units<br>
esp32udp.1.position_cmd<br>
esp32udp.2.position_cmd<br>

esp32udp.0.velocity_cmd (in - float) commanded velocity in position units/s<br>
esp32udp.1.velocity_cmd<br>
esp32udp.2.velocity_cmd<br>

esp32udp.0.position_fb (out - float) feedback position in position units<br>
esp32udp.1.position_fb<br>
esp32udp.2.position_fb<br>

esp32udp.0.velocity_fb (out - float) feedback velocity in position units/s<br>
esp32udp.1.velocity_fb<br>
esp32udp.2.velocity_fb<br>

esp32udp.out.00 (in - bit) digital output<br>
esp32udp.out.01<br>
esp32udp.out.02<br>
esp32udp.out.03<br>
esp32udp.out.04<br>
esp32udp.out.05<br>

esp32udp.pwm.00 (in - float) PWM output 0...1<br>
esp32udp.pwm.01<br>
esp32udp.pwm.02<br>
esp32udp.pwm.03<br>
esp32udp.pwm.04<br>
esp32udp.pwm.05<br>

esp32udp.in.00 (out - bit) digital input<br>
esp32udp.in.01<br>
esp32udp.in.02<br>
esp32udp.in.03<br>
esp32udp.in.04<br>
esp32udp.in.05<br>
esp32udp.in.06<br>

esp32udp.ready 			 		(out - bit) module state<br>
esp32udp.enable 		 		(in - bit) module enable<br>
esp32udp.packets_lost_tx 		(out - s32) lost TX packets<br>
esp32udp.last_pkt_rx_time_ms	(out - s32) Last Packet RX Time in MS (up-to 5s when idle)<br>
esp32udp.send_pkt				(out - s32) TX Packet queue counter<br>
esp32udp.udp_seq_number		    (out - s32) UDP sequence number from ESP32, useful for monitoring<br>

### LinuxCNC HAL parameters

esp32udp.0.scale (rw - float) steps per position unit<br>
esp32udp.1.scale<br>
esp32udp.2.scale<br>

esp32udp.0.accel (rw - float) acceleration in position units/s<sup>2</sup><br>
esp32udp.1.accel<br>
esp32udp.2.accel<br>

esp32udp.pwm.00.freq (rw - u32) PWM frequency in Hz 0..65000<br>
esp32udp.pwm.01.freq<br>
esp32udp.pwm.02.freq<br>
esp32udp.pwm.03.freq<br>
esp32udp.pwm.04.freq<br>
esp32udp.pwm.05.freq<br>

### PWM usage
If the esp32udp.pwm.xx.freq parameter is set to 0, the esp32udp.out.xx pin works and the esp32udp.pwm.xx pin does not.<br>
If the value of the esp32udp.pwm.xx.freq parameter is not 0, the esp32udp.out.xx pin does not work and the esp32udp.pwm.xx pin does.<br>
