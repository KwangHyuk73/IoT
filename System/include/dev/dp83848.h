/*
 ******************************************************************************
 * $File    dp83848.h
 * $Brief   Header file of National Semiconductor's DP83848 Ethernet PHY Chip.
 * $Created on: Jan 18, 2023
 ******************************************************************************
 *
 * Copyright (c) 2023 Digital Museum. All rights reserved.
 *
 * This software component is licensed by Digital Museum under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 *	https://www.youtube.com/@digitalmuseum6400
 *	https://github.com/KwangHyuk73
 *	e-mail:ponytail2k@gmail.com
 ******************************************************************************
 */


#ifndef INCLUDE_DEV_DP83848_H_
#define INCLUDE_DEV_DP83848_H_

/*
 *	National Semiconductor PHYSICAL LAYER TRANSCEIVER DP83848
 */
#define DP83848_BMCR				0x0		/* Basic Mode Control Register */
#define DP83848_BMSR				0x1		/* Basic Mode Status Register */
#define DP83848_PHYID1				0x2		/* PHY Identifier Register 1 */
#define DP83848_PHYID2				0x3		/* PHY Identifier Register 2 */
#define DP83848_ANAR				0x4		/* Auto_Nego Advt Register  */
#define DP83848_ANLPAR				0x5		/* Auto_Nego Link Partner Ability Register */
#define DP83848_ANER				0x6		/* Auto-Nego Expansion Register  */
#define DP83848_ANNPTR				0x7		/* Auto-Nego Next Page Transmit Register  */
#define DP83848_PHYSTS				0x10	/* PHY Status Register  */
#define DP83848_MICR				0x11	/* PHY Interrupt Control Register */
#define DP83848_MISR				0x12	/* PHY Interrupt Status Register */
#define DP83848_FCSCR				0x14	/* False Carrier Sense Counter Register */
#define DP83848_RECR				0x15	/* Receiver Error Counter Register */
#define DP83848_PCSR				0x16	/* PCS Configuration and Status Register */
#define DP83848_RBR					0x17	/* RMII and Bypass Register */
#define DP83848_LEDCR				0x18	/* LED Direct Control Register */
#define DP83848_PHYCR				0x19	/* PHY Status Register  */
#define DP83848_10BTSCR				0x1A	/* 10Base-T Status/Control Register */
#define DP83848_CDCTRL1				0x1B	/* CD Test and BIST Extensions Register */
#define DP83848_EDCR				0x1D	/* Energy Detect Control Register  */

//	Bit definitions of BMCR
#define BMCR_RESET					(1 << 15)  /* 1= S/W Reset */
#define BMCR_LOOPBACK				(1 << 14)  /* 1=loopback Enabled */
#define BMCR_SPEED_SELECT			(1 << 13)
#define BMCR_AUTONEG				(1 << 12)
#define BMCR_POWER_DOWN				(1 << 11)
#define BMCR_ISOLATE				(1 << 10)
#define BMCR_RESTART_AUTONEG		(1 << 9)
#define BMCR_DUPLEX_MODE			(1 << 8)
#define BMCR_COLLISION_TEST			(1 << 7)

//	Bit definitions of BMSR
#define BMSR_100BASE_T4				(1 << 15)
#define BMSR_100BASE_TX_FD			(1 << 14)
#define BMSR_100BASE_TX_HD			(1 << 13)
#define BMSR_10BASE_T_FD			(1 << 12)
#define BMSR_10BASE_T_HD			(1 << 11)
#define BMSR_MF_PREAMB_SUPPR		(1 << 6)
#define BMSR_AUTONEG_COMP			(1 << 5)
#define BMSR_RMT_FAULT				(1 << 4)
#define BMSR_AUTONEG_ABILITY		(1 << 3)
#define BMSR_LINK_STATUS			(1 << 2)
#define BMSR_JABBER_DETECT			(1 << 1)
#define BMSR_EXTEND_CAPAB			(1 << 0)

#define PHYID1_OUI					0x2000
#define PHYID2_OUI					0x5c90

#endif /* INCLUDE_DEV_DP83848_H_ */
