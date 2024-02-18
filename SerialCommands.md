### Serial Console Usage Example

1) Configure a board via `boardconfig`. Restart the board once set `restart`. Ethernet should initialise if pin maps are correct. 
    For SPI Ethernet; you can use `spiethconfig` command to change pins as needed.
    ```
    > boardconfig -t 1 -n 3
    Configure stepper motors pins and parameters
    Usage:
    boardconfig [OPTION...]

        --help             Show help
    -t, --type arg         Board type number = 
                            0 = 'None'
                            1 = 'Wowki Simulator ESP32'
                            2 = 'Olimex ESP32-POE'
                            3 = 'Olimex ESP32-EVB'
                            4 = 'Olimex ESP32-GATEWAY'
                            5 = 'SeedStudio WT32-ETH01'
                            6 = 'MKS-DLC32'
                            7 = 'Default'

    -n, --numsteppers arg  Number of enabled steppers (0-6)
    ```
2) Configure WiFi, if needed. NOTE: AP+STA is needed for ESP-NOW functionality on both Controller and Client
    ```
    > wificonfig --help
    Configure WiFi mode and AP/STA settings
    Usage:
    wificonfig [OPTION...]

        --help          Show help
    -m, --mode arg      WiFi Mode =
                        0 = OFF,
                        1 = STATION,
                        2 = AP,
                        3 = STATION+AP
    -s, --ssid arg      SSID to connect to (STA) or host (AP)
    -p, --password arg  Password
    -h, --hidessid      SSID is hidden when in AP mode
    ```

3) Get current configuration `getconfig` to check pin maps
    ```
    Reading NVS Config...
    NVS Config Version: 2
    BOARD TYPE: 6
    BOARD NAME: MKS-DLC32
    [Mode]: 'Controller'
    [Board Config] Type: 6, Name: 'MKS-DLC32'
    [WiFi Config] Mode: (0) 'OFF', SSID: '', PWD: '', Hidden: 0
    [Ethernet] Type: 'SPI', GPIO: { miso: 19, mosi: 23, sck: 18, cs: 0, int: 4 }
    [ESP-NOW] State: 0, PeerMAC: ''
    [UDP-SERVER] State: 1, Port: 58000
    [UDP-CLIENT] State: 1, Port: 58001


    ==========Steppers==========
    Number of Steppers: 4
    Motor[0]: { 'StepPin': 25, 'DirPin': 130, 'EnHighPin': -1, 'EnLowPin': 128, 'AutoEn': 0, 'DirDelay': 1000, 'OnDelayUs': 2000, 'OffDelayMs': 5 }
    Motor[1]: { 'StepPin': 26, 'DirPin': 134, 'EnHighPin': -1, 'EnLowPin': 128, 'AutoEn': 0, 'DirDelay': 1000, 'OnDelayUs': 2000, 'OffDelayMs': 5 }
    Motor[2]: { 'StepPin': 27, 'DirPin': 132, 'EnHighPin': -1, 'EnLowPin': 128, 'AutoEn': 0, 'DirDelay': 1000, 'OnDelayUs': 2000, 'OffDelayMs': 5 }
    Motor[3]: { 'StepPin': 5, 'DirPin': 33, 'EnHighPin': -1, 'EnLowPin': 128, 'AutoEn': 0, 'DirDelay': 1000, 'OnDelayUs': 2000, 'OffDelayMs': 5 }
    * Uses I2S output for pins. Values > 128 are I2S!


    ==========Inputs==========
    Input[0]: { 'GPIO': -1, 'UDPInNum': 0, 'Name': 'Unused', 'PullUp': 1, 'PullDown': 0, 'RegAddr': 'GPIO_IN_REG', 'RegBit': 0x00 }
    Input[1]: { 'GPIO': -1, 'UDPInNum': 1, 'Name': 'Unused', 'PullUp': 1, 'PullDown': 0, 'RegAddr': 'GPIO_IN_REG', 'RegBit': 0x00 }
    Input[2]: { 'GPIO': -1, 'UDPInNum': 2, 'Name': 'Unused', 'PullUp': 1, 'PullDown': 0, 'RegAddr': 'GPIO_IN_REG', 'RegBit': 0x00 }
    Input[3]: { 'GPIO': 22, 'UDPInNum': 3, 'Name': 'ProbeIn', 'PullUp': 1, 'PullDown': 0, 'RegAddr': 'GPIO_IN_REG', 'RegBit': 0x400000 }
    Input[4]: { 'GPIO': 34, 'UDPInNum': 4, 'Name': 'Z-', 'PullUp': 1, 'PullDown': 0, 'RegAddr': 'GPIO_IN1_REG', 'RegBit': 0x04 }
    Input[5]: { 'GPIO': 35, 'UDPInNum': 5, 'Name': 'Y-', 'PullUp': 1, 'PullDown': 0, 'RegAddr': 'GPIO_IN1_REG', 'RegBit': 0x08 }
    Input[6]: { 'GPIO': 36, 'UDPInNum': 6, 'Name': 'X-', 'PullUp': 1, 'PullDown': 0, 'RegAddr': 'GPIO_IN1_REG', 'RegBit': 0x10 }
    Output[0]: { 'GPIO': 2, 'UDPOutNum': 0, 'Name': 'BlueLed' }
    Output[1]: { 'GPIO': -1, 'UDPOutNum': 1, 'Name': 'Unused' }
    Output[2]: { 'GPIO': -1, 'UDPOutNum': 2, 'Name': 'Unused' }
    Output[3]: { 'GPIO': -1, 'UDPOutNum': 3, 'Name': 'Unused' }
    Output[4]: { 'GPIO': -1, 'UDPOutNum': 4, 'Name': 'Unused' }
    Output[5]: { 'GPIO': 32, 'UDPOutNum': 5, 'Name': 'test' }
    Output[6]: { 'GPIO': 135, 'UDPOutNum': 6, 'Name': 'Beeper' }
    ```

4) To configure a motor use `setmotor` command:
    ```
    > setmotor --help
    Configure stepper motors pins and parameters
    Usage:
    setmotor [OPTION...]

        --help                  Show help
    -m, --motor arg             Motor index number (0-6)
    -s, --steppin arg           Step GPIO (int)
    -d, --dirpin arg            Direction GPIO (int)
    -h, --enablehighpin arg     Enable active high GPIO (int)
    -l, --enablelowpin arg      Enable active low GPIO (int)
    -a, --autoenable            Auto enable (true|false)
    -1, --ondelayus arg         On delay time in us (0>120000)
    -2, --offdelayms arg        Off delay time in ms (0>120)
    -c, --dirchangedelayus arg  Direction change delay in us (0>4095)
    ```

5) To configure an Input pin use `inputconfig` command:
    ```
    > inputconfig --help
    Configure Input pins and parameters
    Usage:
    inputconfig [OPTION...]

        --help              Show help
    -i, --index arg         Input index number (0-6)
    -n, --name arg          Input name (string)
    -g, --gpiopin arg       GPIO Pin (-1=Disable, 0->128)
    -u, --pullup            Enable Pullup Resistors (0|1)
    -d, --pulldown          Enable Down Resistors (0|1)
    -r, --registerbank arg  Input Register Bank (0=IN_REG, 1=IN1_REG)
    -b, --registerbit arg   Input Register Bit Number (int)
    ```

6) To configure an Output pin use `outputconfig` command:
    ```
    outputconfig --help
    Configure Output pins and parameters
    Usage:
    outputconfig [OPTION...]

        --help         Show help
    -i, --index arg    Output index number (0-6)
    -n, --name arg     Output name (string)
    -g, --gpiopin arg  GPIO Pin (-1=Disable, 0->128)
    ```

7) Save configuration using `saveconfig` 
8) Enable startup stepper motor configuration using `enablemotorconfig on` or disable using `enablemotorconfig off`
9) Control console logging using `log on` and `log off` 
10) Change device mode between Controller and Client using `mode controller` or `mode client`
11) See stats using `stats`
12) Get a single motor configuration using `getmotor -m X`
13) See uptime in seconds using `uptime`
14) Toggle ESP-NOW server/client feature using `espnow on` or `espnow off`
15) Get system information using `sysinfo`
16) Check heap memory stats using `meminfo`
17) Ping a host `ping x.x.x.x`
18) View IP configuration of both WiFi and Ethernet `ipconfig`
19) Stop loop processing via `stop` command


### SPI Ethernet Firmware Only

Configure SPI Ethernet module pins using `spiethconfig` 

```
    > spiethconfig --help
    SPI Ethernet Pin Configuration.
    Usage:
    spiethconfig [OPTION...]

        --help           Show help
    -s, --miso arg       MISO pin
    -i, --mosi arg       MOSI pin
    -c, --sck arg        SCK pin
    -e, --csenable arg   Chip Select Enable pin
    -t, --interrupt arg  Interrupt pin
```

### Client Mode Configuration

Switch to client mode via `mode client` and `restart`
When in client mode use the command `setcontrollermac 00:00:00:00:00:00` using the WiFi mac address of the controller that is printed on its console at boot.<br>
Use `espnow on` on both Controller and Client in order for them to establish a connection.<br>
Settings are saved into NVS and will persist during restarts or firmware upgrades.<br>


### Full Help Output
```
help 
    Print the list of registered commands

clear  
    Clears the screen using ANSI codes

history  
    Shows and clear command history (using -c parameter)

echo  
    Echos the text supplied as argument

multiline_mode
    Sets the multiline mode of the console

env
    List all environment variables.

declare
    Change enviroment variables

sysinfo
    Shows informations about the system like chip model and ESP-IDF version

restart
    Restart / Reboot the system

meminfo
    Shows information about heap usage

date
    Shows and modify the system time

ping
    Ping host

ipconfig
    Show IP and connection informations

pinMode
    Changes the pinmode of an GPIO pin (similar to Arduino function)

digitalRead
    Reads the state of an input pin (similar to Arduino function)

digitalWrite
    Writes the state of an ouput pin (similar to Arduino function)

analogRead
    Show the voltage at an analog pin in millivollts.

setmotor  Use --help option of command for more info
    Configure stepper motors pins and parameters

enablemotorconfig
    Enable/Disable motor config at startup

getmotor  Use --help option of command for more info
    Get a stepper motors current config

stats
    Loop task communications statistics

mode
    Set device mode to Client or Controller. mode = 'controller' | 'client'.
    Restart required

espnow
    Enable/Disable ESP-NOW wireless P2P link between controller and client ESP32 devices

log
    Enable / Disable informational logging. 

debug
    Enable / Disable debug logging.

getconfig
    Read environment configuration from NVS storage

saveconfig
    Save environment configuration to NVS storage

resetconfig
    Clear/reset NVS configuration to defaults

boardconfig  Use --help option of command for more info
    Configure stepper motors pins and parameters

wificonfig  Use --help option of command for more info
    Configure WiFi mode and AP/STA settings

inputconfig  Use --help option of command for more info
    Configure Input pins and parameters

outputconfig  Use --help option of command for more info
    Configure Output pins and parameters

firmwareupdate
    Perform firmware update via WiFi from online repository

uptime
    System uptime in seconds

stop
    Stop processing task loops

spiethconfig  Use --help option of command for more info
    SPI Ethernet Pin Configuration.

motormovetest
    Send debug motor movement for axis for either 'forwards' or 'backwards' moves
```