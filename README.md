# ESP32UDP
Hardware based external step generator and IO interface for LinuxCNC over native RMII Ethernet on a dual core ESP32 for numerous boards.<br>

The hardware is connected to LinuxCNC over Ethernet. The controller operates in position mode at low speed and at higher speeds in velocity mode.<br>

### Features
* Based heavily on Arduino libraries to allow easy customisation or custom integrations
* Uses native ESP32 RMII Ethernet for high performance and low latency
* **ENABLE** **STEP** and **DIR** signals for up-to 6 axes (if careful planning with GPIO pins)
* 7 input pins
* 6 output pins, any can be pwm signal
* Step frequency up to 200 kHz total using RMT peripheral. Can drive 6 motors reliably, smoothly and accurately across all accreleration profiles
* Extensibe: Spare core #1 for Arduino tasks or extra libraries
* Core 0 handles UDP Server & Client, Commands, Motor step generation, position and feedback control
* WiFI working with OTA updates and Telnet server for remote monitoring. OTA firmware updates can be performed via Ethernet too to speed up development iterations if using a Virual LinuxCNC instance
* Serial and Telnet interface with stepper and input status stats
* Detailed logging output on serial interface (to be improved and easier to visualise any board setup or config errors later)
* Handles most setup problems and reports on the Serial console if there are any problems
* Event driven real-time UDP networking protocol with sleeping when idle and error handling freeing resources on ESP32
* Inputs pin status and triggers are handled by high speed interrupt handler and reading of GPIO input registers
* ESP32 Pulse Counter perihperal is unused allowing for Servo Encoder feedback connection in future. Motor position reporting back to LinuxCNC PID controller should be straight forward
* Multiple board support with native RMII ethernet modules. WT32-ETH01, Olimex ESP32-EVB, ESP32-Gateway, ESP32-POE - or any other ESP32 with any of the supports LAN Phys will work
* Something more somewhere but will continue to evolve over time
* Future scope - ESP32-S2/S3 4 Axis USB based networking (USB RNDIS/ECM Ethernet) so no more Ethernet adapters or Phys. A single $5 ESP32-S2 or S3 module with USB-C will handle all LinuxCNC real-time comms and high-speed and accurate motor driving (yes, it is possible!)

### Install
Clone this repository and open it with platformIO in Visual Studio Code

### Settings
LinuxCNC Host
Set your ethernet NIC that ESP32 is connected to 192.168.111.2/24<br>
ESP32 has a static ip of 192.168.111.1. This can be changed in Config.h

### LinuxCNC driver
```bash
sudo apt-get install linuxcnc-uspace-dev build-essential
```
```bash
sudo halcompile --install esp32udp.comp
```
### Hardware
Any ESP-WROOM/WROVER-32 board with RMII Ethernet Phy (e.g LAN8720) <br>

Ideal pin mappings and available IO per board will be documented soon

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
esp32udp.last_pkt_rx_time_ms	(out - s32) Last Packet RX Time in MS (up-to 5s when idle) <br>
esp32udp.send_pkt				(out - s32) TX Packet queue counter <br>
esp32udp.udp_seq_number		    (out - s32) UDP sequence number from ESP32, useful for monitoring <br>

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

### Credits
Credit goes to Juhász Zoltán for his original great work and concept for the HAL2UDP components using W5500 SPI ethernet and software based Step generation on ESP32.
This project was originally based on it but the firmware source hardly resembles any of it now. GPL licensing is given in the source header.