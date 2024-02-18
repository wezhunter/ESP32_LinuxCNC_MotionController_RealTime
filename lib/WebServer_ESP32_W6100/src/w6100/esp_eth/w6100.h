/****************************************************************************************************************************
  w6100.h

  For Ethernet shields using ESP32_W6100 (ESP32 + W6100)

  WebServer_ESP32_W6100 is a library for the ESP32 with Ethernet W6100 to run WebServer

  Based on and modified from ESP32-IDF https://github.com/espressif/esp-idf
  Built by Khoi Hoang https://github.com/khoih-prog/WebServer_ESP32_W6100
  Licensed under GPLv3 license

  Version: 1.5.3

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.5.2   K Hoang      06/01/2022 Initial coding for ESP32_W6100 (ESP32 + W6100). Sync with WebServer_ESP32_W5500 v1.5.2
  1.5.3   K Hoang      11/01/2023 Using `SPI_DMA_CH_AUTO`
 *****************************************************************************************************************************/

// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifndef _ESP32_ETH_W6100_H_
#define _ESP32_ETH_W6100_H_

////////////////////////////////////////

#define W6100_ADDR_OFFSET (16) // Address length
#define W6100_BSB_OFFSET  (3)  // Block Select Bits offset
#define W6100_RWB_OFFSET  (2)  // Read Write Bits offset

////////////////////////////////////////

#define W6100_BSB_COM_REG        (0x00)    // Common Register
#define W6100_BSB_SOCK_REG(s)    ((s)*4+1) // Socket Register
#define W6100_BSB_SOCK_TX_BUF(s) ((s)*4+2) // Socket TX Buffer
#define W6100_BSB_SOCK_RX_BUF(s) ((s)*4+3) // Socket RX Buffer

////////////////////////////////////////

#define W6100_ACCESS_MODE_READ  (0) // Read Mode
#define W6100_ACCESS_MODE_WRITE (1) // Write Mode

////////////////////////////////////////

#define W6100_SPI_OP_MODE_VDM   (0x00) // Variable Data Length Mode (SPI frame is controlled by CS line)
#define W6100_SPI_OP_MODE_FDM_1 (0x01) // Fixed Data Length Mode, 1 Byte Length
#define W6100_SPI_OP_MODE_FDM_2 (0x02) // Fixed Data Length Mode, 2 Bytes Length
#define W6100_SPI_OP_MODE_FDM_4 (0x03) // Fixed Data Length Mode, 4 Bytes Length

////////////////////////////////////////

#define W6100_MAKE_MAP(offset, bsb) ((offset) << W6100_ADDR_OFFSET | (bsb) << W6100_BSB_OFFSET)

////////////////////////////////////////

#define W6100_REG_MR        W6100_MAKE_MAP(0x4000, W6100_BSB_COM_REG) // Mode
#define W6100_REG_MAC       W6100_MAKE_MAP(0x4120, W6100_BSB_COM_REG) // MAC Address

#define W6100_REG_IR        W6100_MAKE_MAP(0x2100, W6100_BSB_COM_REG) // Interrupt
#define W6100_REG_IMR       W6100_MAKE_MAP(0x2104, W6100_BSB_COM_REG) // Interrupt Mask
#define W6100_REG_SIR       W6100_MAKE_MAP(0x2101, W6100_BSB_COM_REG) // Socket Interrupt
#define W6100_REG_SIMR      W6100_MAKE_MAP(0x2114, W6100_BSB_COM_REG) // Socket Interrupt Mask
#define W6100_REG_RTR       W6100_MAKE_MAP(0x4200, W6100_BSB_COM_REG) // Retry Time

#define W6100_REG_PHYCFGR   W6100_MAKE_MAP(0x3000, W6100_BSB_COM_REG) // PHY Configuration
#define W6100_REG_VERSIONR  W6100_MAKE_MAP(0x0000, W6100_BSB_COM_REG) // Chip version

////////////////////////////////////////

#define W6100_REG_SOCK_MR(s)         W6100_MAKE_MAP(0x0000, W6100_BSB_SOCK_REG(s)) // Socket Mode
#define W6100_REG_SOCK_CR(s)         W6100_MAKE_MAP(0x0010, W6100_BSB_SOCK_REG(s)) // Socket Command
#define W6100_REG_SOCK_IR(s)         W6100_MAKE_MAP(0x0020, W6100_BSB_SOCK_REG(s)) // Socket Interrupt
#define W6100_REG_SOCK_SR(s)         W6100_MAKE_MAP(0x0030, W6100_BSB_SOCK_REG(s)) // Socket Status

#define W6100_REG_SOCK_RXBUF_SIZE(s) W6100_MAKE_MAP(0x0220, W6100_BSB_SOCK_REG(s)) // Socket Receive Buffer Size
#define W6100_REG_SOCK_TXBUF_SIZE(s) W6100_MAKE_MAP(0x0200, W6100_BSB_SOCK_REG(s)) // Socket Transmit Buffer Size
#define W6100_REG_SOCK_TX_FSR(s)     W6100_MAKE_MAP(0x0204, W6100_BSB_SOCK_REG(s)) // Socket TX Free Size

#define W6100_REG_SOCK_TX_RD(s)      W6100_MAKE_MAP(0x0208, W6100_BSB_SOCK_REG(s)) // Socket TX Read Pointer
#define W6100_REG_SOCK_TX_WR(s)      W6100_MAKE_MAP(0x020C, W6100_BSB_SOCK_REG(s)) // Socket TX Write Pointer
#define W6100_REG_SOCK_RX_RSR(s)     W6100_MAKE_MAP(0x0224, W6100_BSB_SOCK_REG(s)) // Socket RX Received Size
#define W6100_REG_SOCK_RX_RD(s)      W6100_MAKE_MAP(0x0228, W6100_BSB_SOCK_REG(s)) // Socket RX Read Pointer
#define W6100_REG_SOCK_RX_WR(s)      W6100_MAKE_MAP(0x022C, W6100_BSB_SOCK_REG(s)) // Socket RX Write Pointer

#define W6100_REG_SOCK_IMR(s)        W6100_MAKE_MAP(0x0024, W6100_BSB_SOCK_REG(s)) // Socket Interrupt Mask

////////////////////////////////////////

#define W6100_MEM_SOCK_TX(s,addr) W6100_MAKE_MAP(addr, W6100_BSB_SOCK_TX_BUF(s))    // Socket TX buffer address
#define W6100_MEM_SOCK_RX(s,addr) W6100_MAKE_MAP(addr, W6100_BSB_SOCK_RX_BUF(s))    // Socket RX buffer address

////////////////////////////////////////

#define W6100_MR_RST (1<<7) // Software reset
#define W6100_MR_PB  (1<<4) // Ping block (block the response to a ping request)

////////////////////////////////////////

#define W6100_SIMR_SOCK0 (1<<0) // Socket 0 interrupt

////////////////////////////////////////

#define W6100_SMR_MAC_RAW    (0x07)   // MAC RAW mode
#define W6100_SMR_MAC_FILTER (1<<7)   // MAC filter

////////////////////////////////////////

#define W6100_SCR_OPEN  (0x01) // Open command
#define W6100_SCR_CLOSE (0x10) // Close command
#define W6100_SCR_SEND  (0x20) // Send command
#define W6100_SCR_RECV  (0x40) // Recv command

////////////////////////////////////////

#define W6100_SIR_RECV (1<<2)  // Receive done
#define W6100_SIR_SEND (1<<4)  // Send done

////////////////////////////////////////

#define W6100_CHPLCKR_UNLOCK              0xCE
#define W6100_NETLCKR_UNLOCK              0x3A
#define W6100_PHYLCKR_UNLOCK              0x53

////////////////////////////////////////

#define SYCR0                             0x2004    // System Config Register 0
#define SYCR1                             0x2005    // System Config Register 1
#define SYSR_W6100                        0x2000    // System Status Register

#define CHPLCKR_W6100                     0x41F4    // Chip Lock Register
#define NETLCKR_W6100                     0x41F5    // Network Lock Register
#define PHYLCKR_W6100                     0x41F6    // PHY Lock Register

#define VERSIONR_W6100                    0x0       // Chip Version Register [RO]=0x6100
#define CVERSIONR_W6100                   0x0002    // Chip Version Register [RO]=0x4661

//////////////////////////////////////////////////

#define W6100_REG_SYCR0                   W6100_MAKE_MAP(SYCR0, W6100_BSB_COM_REG)          // System Command Register
#define W6100_REG_SYSR_W6100              W6100_MAKE_MAP(SYSR_W6100, W6100_BSB_COM_REG)     // System Status Register

#define W6100_REG_CHPLCKR_W6100           W6100_MAKE_MAP(CHPLCKR_W6100, W6100_BSB_COM_REG)  // Chip Lock Register
#define W6100_REG_NETLCKR_W6100           W6100_MAKE_MAP(NETLCKR_W6100, W6100_BSB_COM_REG)  // Network Lock Register
#define W6100_REG_PHYLCKR_W6100           W6100_MAKE_MAP(PHYLCKR_W6100, W6100_BSB_COM_REG)  // PHY Lock Register

// Chip Version Register
#define W6100_REG_VERSIONR_W6100          W6100_MAKE_MAP(VERSIONR_W6100, W6100_BSB_COM_REG)
#define W6100_REG_CVERSIONR_W6100         W6100_MAKE_MAP(CVERSIONR_W6100, W6100_BSB_COM_REG)

////////////////////////////////////////

#define W6100_SYSR_CHPL_LOCK              (1<<7)
#define W6100_SYSR_CHPL_ULOCK             (0<<7)

////////////////////////////////////////

#define W6100_UDP_HEADER_IPV              (1<<7)
#define W6100_UDP_HEADER_IPV4             (0<<7)
#define W6100_UDP_HEADER_IPV6             (1<<7)
#define W6100_UDP_HEADER_ALL              (1<<6)
#define W6100_UDP_HEADER_MUL              (1<<5)
#define W6100_UDP_HEADER_GUA              (0<<3)
#define W6100_UDP_HEADER_LLA              (1<<3)

////////////////////////////////////////

#define W6100_SLCR_NS                     (1<<2)
#define W6100_SLCR_RS                     (1<<1)
#define W6100_SLIR_TIOUT                  (1<<7)
#define W6100_ICMP6BLK_RA                 (1<<2)

////////////////////////////////////////

#define W6100_SnESR_TCP4                  (0<<2)
#define W6100_SnESR_TCP6                  (1<<2)

#define W6100_SnMR_TCP4                   (1<<0)
#define W6100_SnMR_TCPD                   (13<<0)

#define W6100_SnPSR_AUTO                  (0<<0)
#define W6100_SnPSR_LLA                   (2<<0)
#define W6100_SnPSR_GUA                   (3<<0)

////////////////////////////////////////

#endif    // _ESP32_ETH_W6100_H_
