#ifndef __W5500_H
#define __W5500_H

//setup the address register map
#define MODE_REG 					0x0000
#define GAR0_REG					0x0001
#define GAR1_REG					0x0002
#define GAR2_REG						0x0003
#define GAR3_REG					0x0004
#define SUBR0_REG					0x0005
#define SUBR1_REG					0x0006
#define SUBR2_REG					0x0007
#define SUBR3_REG					0x0008
#define SHAR0_REG					0x0009
#define SHAR1_REG					0x000A
#define SHAR2_REG					0x000B
#define SHAR3_REG					0x000C
#define SHAR4_REG					0x000D
#define SHAR5_REG					0x000E
#define SIPR0_REG					0x000F
#define SIPR1_REG					0x0010
#define SIPR2_REG					0x0011
#define SIPR3_REG					0x0012
#define INTLEVEL0_REG			0x0013
#define INTLEVEL1_REG			0x0014
#define IR_REG						0x0015
#define IMR_REG						0x0016
#define SIR_REG						0x0017
#define SIMR_REG					0x0018
#define RTR0_REG					0x0019
#define RTR1_REG					0x001A
#define RCR_REG						0x001B
#define PTIMER_REG				0x001C
#define PMAGIC_REG				0x001D
#define PHAR0_REG					0x001E
#define PHAR1_REG					0x001F
#define PHAR2_REG					0x0020
#define PHAR3_REG					0x0021
#define PHAR4_REG					0x0022
#define PHAR5_REG					0x0023
#define PSID0_REG					0x0024
#define PSID1_REG					0x0025
#define PMRU0_REG					0x0026
#define PMRU1_REG					0x0027
#define UIPR0_REG					0x0028
#define UIPR1_REG					0x0029
#define UIPR2_REG					0x002A
#define UIPR3_REG					0x002B
#define UPORTR0_REG				0x002C
#define UPORTR1_REG				0x002D
#define PHYCFGR						0x002E
#define VERSIONR					0x0039

//setup the control phase map - blocks
#define COMMON_REG 				0x00
#define SOCKET_0_REG 			0x08
#define SOCKET_0_TX_BUF		0x10
#define SOCKET_0_RX_BUF		0x18
#define SOCKET_1_REG			0x28
#define SOCKET_1_TX_BUF		0x30
#define SOCKET_1_RX_BUF		0x38
#define SOCKET_2_REG			0x48
#define SOCKET_2_TX_BUF		0x50
#define SOCKET_2_RX_BUF		0x58
#define SOCKET_3_REG			0x68
#define SOCKET_3_TX_BUF		0x70
#define SOCKET_3_RX_BUF		0x78
#define SOCKET_4_REG			0x88
#define SOCKET_4_TX_BUF		0x90
#define SOCKET_4_RX_BUF		0x98
#define SOCKET_5_REG			0xA8
#define SOCKET_5_TX_BUF		0xB0
#define SOCKET_5_RX_BUF		0xB8
#define SOCKET_6_REG			0xC8
#define SOCKET_6_TX_BUF		0xD0
#define SOCKET_6_RX_BUF		0xD8
#define SOCKET_7_REG			0xE8
#define SOCKET_7_TX_BUF		0xF0
#define SOCKET_7_RX_BUF		0xF8
//setup the control phase map - read/write
#define SOCKET_READ				0x00
#define SOCKET_WRITE			0x04
//setup the control phase map - spi operation mode
#define VAR_DATA_MODE			0x00
#define FIXED_LEN_1				0x01
#define FIXED_LEN_2				0x02
#define FIXED_LEN_4				0x03

#endif
