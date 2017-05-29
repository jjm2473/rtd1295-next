/*
 *
 * Copyright (C) FUJITSU LIMITED 2007-2008
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011. All rights reserved.
 * Copyright (C) 2011-13 FUJITSU SEMICONDUCTOR LIMITED
 *
 * Derived from f_mac3h driver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */


#ifndef __NET_FGMAC4_H
#define __NET_FGMAC4_H

#include <linux/phy.h>
#include <linux/clk.h>

/* F_GMAC4 Register Address Map
 */
#define FGMAC4_REG_MCR		0x0000	/* MAC Configuration Register         */
#define FGMAC4_REG_MFFR		0x0004	/* MAC Frame Filter Register          */
#define FGMAC4_REG_MHTRH	0x0008	/* MAC Hash Table Register(High)      */
#define FGMAC4_REG_MHTRL	0x000C	/* MAC Hash Table Register(Low)       */
#define FGMAC4_REG_GAR		0x0010	/* GMII Address Register              */
#define FGMAC4_REG_GDR		0x0014	/* GMII Data Register                 */
#define FGMAC4_REG_FCR		0x0018	/* Flow Control Register              */
#define FGMAC4_REG_VTR		0x001C	/* VLAN Tag Register                  */
#define FGMAC4_REG_RWFFR	0x0028	/* Remote Wake-up Frame FilterRegister*/
#define FGMAC4_REG_PMTR		0x002C	/* PMT Register                       */
#define FGMAC4_REG_LPICSR	0x0030	/* LPI Control and Status Register    */
#define FGMAC4_REG_LPITCR	0x0034	/* LPI Timers Control Register        */
#define FGMAC4_REG_ISR		0x0038	/* Interrupt Status Register          */
#define FGMAC4_REG_IMR		0x003C	/* Interrupt Mask Register            */
#define FGMAC4_REG_MAR0H	0x0040	/* MAC Address0 Register(High)        */
#define FGMAC4_REG_MAR0L	0x0044	/* MAC Address0 Register(Low)         */
#define FGMAC4_REG_MAR1H	0x0048	/* MAC Address1 Register(High)        */
#define FGMAC4_REG_MAR1L	0x004C	/* MAC Address1 Register(Low)         */
#define FGMAC4_REG_MAR2H	0x0050	/* MAC Address2 Register(High)        */
#define FGMAC4_REG_MAR2L	0x0054	/* MAC Address2 Register(Low)         */
#define FGMAC4_REG_MAR3H	0x0058	/* MAC Address3 Register(High)        */
#define FGMAC4_REG_MAR3L	0x005C	/* MAC Address3 Register(Low)         */
#define FGMAC4_REG_MAR4H	0x0060	/* MAC Address4 Register(High)        */
#define FGMAC4_REG_MAR4L	0x0064	/* MAC Address4 Register(Low)         */
#define FGMAC4_REG_MAR5H	0x0068	/* MAC Address5 Register(High)        */
#define FGMAC4_REG_MAR5L	0x006C	/* MAC Address5 Register(Low)         */
#define FGMAC4_REG_MAR6H	0x0070	/* MAC Address6 Register(High)        */
#define FGMAC4_REG_MAR6L	0x0074	/* MAC Address6 Register(Low)         */
#define FGMAC4_REG_MAR7H	0x0078	/* MAC Address7 Register(High)        */
#define FGMAC4_REG_MAR7L	0x007C	/* MAC Address7 Register(Low)         */
#define FGMAC4_REG_MAR8H	0x0080	/* MAC Address8 Register(High)        */
#define FGMAC4_REG_MAR8L	0x0084	/* MAC Address8 Register(Low)         */
#define FGMAC4_REG_MAR9H	0x0088	/* MAC Address9 Register(High)        */
#define FGMAC4_REG_MAR9L	0x008C	/* MAC Address9 Register(Low)         */
#define FGMAC4_REG_MAR10H	0x0090	/* MAC Address10 Register(High)       */
#define FGMAC4_REG_MAR10L	0x0094	/* MAC Address10 Register(Low)        */
#define FGMAC4_REG_MAR11H	0x0098	/* MAC Address11 Register(High)       */
#define FGMAC4_REG_MAR11L	0x009C	/* MAC Address11 Register(Low)        */
#define FGMAC4_REG_MAR12H	0x00A0	/* MAC Address12 Register(High)       */
#define FGMAC4_REG_MAR12L	0x00A4	/* MAC Address12 Register(Low)        */
#define FGMAC4_REG_MAR13H	0x00A8	/* MAC Address13 Register(High)       */
#define FGMAC4_REG_MAR13L	0x00AC	/* MAC Address13 Register(Low)        */
#define FGMAC4_REG_MAR14H	0x00B0	/* MAC Address14 Register(High)       */
#define FGMAC4_REG_MAR14L	0x00B4	/* MAC Address14 Register(Low)        */
#define FGMAC4_REG_MAR15H	0x00B8	/* MAC Address15 Register(High)       */
#define FGMAC4_REG_MAR15L	0x00BC	/* MAC Address15 Register(Low)        */
#define FGMAC4_REG_RGSR		0x00D8	/* RGMII status Reigster              */

/* MDC */
#define FGMAC4_REG_BMR		0x1000	/* MDC BUS Mode Register              */
#define FGMAC4_REG_TPDR		0x1004	/* MDC Transmit Poll Demand Register  */
#define FGMAC4_REG_RPDR		0x1008	/* MDC Receive Poll Demand Register   */
#define FGMAC4_REG_RDLAR	0x100C	/* MDC Rx Desc List Address Register  */
#define FGMAC4_REG_TDLAR	0x1010	/* MDC Tx Desc List Address Register  */
#define FGMAC4_REG_SR		0x1014	/* MDC Status Register                */
#define FGMAC4_REG_OMR		0x1018	/* MDC Operation Mode Register        */
#define FGMAC4_REG_IER		0x101C	/* MDC Interrupt Enable Register      */
#define FGMAC4_REG_MFC		0x1020	/* MDC Missed Frame Register          */
#define FGMAC4_REG_RIWTR	0x1024	/* Rx Intr Watchdog Timer Register    */
#define FGMAC4_REG_AHBSR	0x102C	/* AHB Status Register                */
#define FGMAC4_REG_CHTDR	0x1048	/* MDC Cur Host Transmit Desc Register*/
#define FGMAC4_REG_CHRDR	0x104C	/* MDC Cur Host Receive Desc Register */
#define FGMAC4_REG_CHTBAR	0x1050	/* MDC Cur Host TX Buf Addr Register  */
#define FGMAC4_REG_CHRBAR	0x1054	/* MDC Cur Host RX Buf Addr Register  */

/* MMC(MAC Management Counters) Register Address Map */
#define FGMAC4_REG_MMC_CNTL			0x0100
#define FGMAC4_REG_MMC_INTR_RX			0x0104
#define FGMAC4_REG_MMC_INTR_TX			0x0108
#define FGMAC4_REG_MMC_INTR_MASK_RX		0x010C
#define FGMAC4_REG_MMC_INTR_MASK_TX		0x0110

#define FGMAC4_REG_MMC_TXOTCETCOUNT_GB		0x0114
#define FGMAC4_REG_MMC_TXFRAMECOUNT_GB		0x0118
#define FGMAC4_REG_MMC_TXBROADCASTFRAMES_G	0x011C
#define FGMAC4_REG_MMC_TXMULTICASTFRAMES_G	0x0120
#define FGMAC4_REG_MMC_TX64OCTECS_GB		0x0124
#define FGMAC4_REG_MMC_TX65TO127OCTETS_GB	0x0128
#define FGMAC4_REG_MMC_TX128TO255OCTETS_GB	0x012C
#define FGMAC4_REG_MMC_TX256TO511OCTETS_GB	0x0130
#define FGMAC4_REG_MMC_TX512TO1023OCTETS_GB	0x0134
#define FGMAC4_REG_MMC_TX1024TOMAXOCTETS_GB	0x0138
#define FGMAC4_REG_MMC_TXUNICASTFRAMES_GB	0x013C
#define FGMAC4_REG_MMC_TXMULTICASTFRAMES_GB	0x0140
#define FGMAC4_REG_MMC_TXBROADCASTFRAMES_GB	0x0144
#define FGMAC4_REG_MMC_TXUNDERFLOWERROR		0x0148
#define FGMAC4_REG_MMC_TXSINGLECOL_G		0x014C
#define FGMAC4_REG_MMC_TXMULTICOL_G		0x0150
#define FGMAC4_REG_MMC_TXDEFERRED		0x0154
#define FGMAC4_REG_MMC_TXLATECOL		0x0158
#define FGMAC4_REG_MMC_TXEXESSCOL		0x015C
#define FGMAC4_REG_MMC_TXCARRIERERROR		0x0160
#define FGMAC4_REG_MMC_TXOCTETCOUNT_G		0x0164
#define FGMAC4_REG_MMC_TXFRAMECOUNT_G		0x0168
#define FGMAC4_REG_MMC_TXEXECESSDEF		0x016C
#define FGMAC4_REG_MMC_TXPAUSEFRAMES		0x0170
#define FGMAC4_REG_MMC_TXVLANFRAMES_G		0x0174
#define FGMAC4_REG_MMC_RXFRAMECOUNT_GB		0x0180
#define FGMAC4_REG_MMC_RXOCTETCOUNT_GB		0x0184
#define FGMAC4_REG_MMC_RXOCTETCOUNT_G		0x0188
#define FGMAC4_REG_MMC_RXBROADCASTFRAMES_G	0x018C
#define FGMAC4_REG_MMC_RXMULTICASTFRAMES_G	0x0190
#define FGMAC4_REG_MMC_RXCRCERROR		0x0194
#define FGMAC4_REG_MMC_RXALIGNMENTERROR		0x0198
#define FGMAC4_REG_MMC_RXRUNTERROR		0x019C
#define FGMAC4_REG_MMC_RXJABBERERROR		0x01A0
#define FGMAC4_REG_MMC_RXUNDERSIZE_G		0x01A4
#define FGMAC4_REG_MMC_RXOVERSIZE_G		0x01A8
#define FGMAC4_REG_MMC_RX64OCTETS_GB		0x01AC
#define FGMAC4_REG_MMC_RX65TO127OCTETS_GB	0x01B0
#define FGMAC4_REG_MMC_RX128TO255OCTETS_GB	0x01B4
#define FGMAC4_REG_MMC_RX256TO511OCTETS_GB	0x01B8
#define FGMAC4_REG_MMC_RX512TO1023OCTETS_GB	0x01BC
#define FGMAC4_REG_MMC_RX1024TOMAXOCTETS_GB	0x01C0
#define FGMAC4_REG_MMC_RXUNICASTFRAMES_G	0x01C4
#define FGMAC4_REG_MMC_RXLENGTHERROR		0x01C8
#define FGMAC4_REG_MMC_RXOUTOFRANGETYPE		0x01CC
#define FGMAC4_REG_MMC_RXPAUSEFRAMES		0x01D0
#define FGMAC4_REG_MMC_RXFIFOOVERFLOW		0x01D4
#define FGMAC4_REG_MMC_RXVLANFRAMES_GB		0x01D8
#define FGMAC4_REG_MMC_RXWATCHDOGERROR		0x01DC
#define FGMAC4_REG_MMC_IPC_INTR_MASK_RX		0x0200
#define FGMAC4_REG_MMC_IPC_INTR_RX		0x0208
#define FGMAC4_REG_MMC_RXIPV4_GB_FRMS		0x0210
#define FGMAC4_REG_MMC_RXIPV4_HDRERR_FRMS	0x0214
#define FGMAC4_REG_MMC_RXIPV4_NOPAY_FRMS	0x0218
#define FGMAC4_REG_MMC_RXIPV4_FRAG_FRMS		0x021C
#define FGMAC4_REG_MMC_RXIPV4_UDSBL_FRMS	0x0220
#define FGMAC4_REG_MMC_RXIPV6_GB_FRMS		0x0224
#define FGMAC4_REG_MMC_RXIPV6_HDRERR_FRMS	0x0228
#define FGMAC4_REG_MMC_RXIPV6_NOPAY_FRMS	0x022C
#define FGMAC4_REG_MMC_RXUDP_GB_FRMS		0x0230
#define FGMAC4_REG_MMC_RXUDP_ERR_FRMS		0x0234
#define FGMAC4_REG_MMC_RXTCP_GB_FRMS		0x0238
#define FGMAC4_REG_MMC_RXTCP_ERR_FRMS		0x023C
#define FGMAC4_REG_MMC_RXICMP_GB_FRMS		0x0240
#define FGMAC4_REG_MMC_RXICMP_ERR_FRMS		0x0244
#define FGMAC4_REG_MMC_RXIPV4_GB_OCTETS		0x0250
#define FGMAC4_REG_MMC_RXIPV4_HDRERR_OCTETS	0x0254
#define FGMAC4_REG_MMC_RXIPV4_NOPAY_OCTETS	0x0258
#define FGMAC4_REG_MMC_RXIPV4_FRAG_OCTETS	0x025C
#define FGMAC4_REG_MMC_RXIPV4_UDSBL_OCTETS	0x0260
#define FGMAC4_REG_MMC_RXIPV6_GB_OCTETS		0x0264
#define FGMAC4_REG_MMC_RXIPV6_HDRERR_OCTETS	0x0268
#define FGMAC4_REG_MMC_RXIPV6_NOPAY_OCTETS	0x026C
#define FGMAC4_REG_MMC_RXUDP_GB_OCTETS		0x0270
#define FGMAC4_REG_MMC_RXUDP_ERR_OCTETS		0x0274
#define FGMAC4_REG_MMC_RXTCP_GB_OCTETS		0x0278
#define FGMAC4_REG_MMC_RXTCP_ERR_OCTETS		0x027C
#define FGMAC4_REG_MMC_RXICMP_GB_OCTETS		0x0280
#define FGMAC4_REG_MMC_RXICMP_ERR_OCTETS	0x0284



/* Values and Masks
 */
#define SET_0			0x00000000
#define SET_1			0xFFFFFFFF

/* IMR : Interrupt Mask Register */
#define FGMAC4_IMR_LPIIM	0x00000400  /* LPI Interrupt Mask */
#define FGMAC4_IMR_TSIM		0x00000200  /* Time Stamp Interrupt Mask */
#define FGMAC4_IMR_PIM		0x00000008  /* PMT Interrupt Mask */
#define FGMAC4_IMR_RGIM		0x00000001  /* RGMII Interrupt Mask */

/* MCR:MAC Configuration Register */
#define FGMAC4_MCR_CST		0x02000000  /* CRC stripping                  */
#define FGMAC4_MCR_TC		0x01000000  /* Tx Configuration (in RGMII)    */
#define FGMAC4_MCR_WD		0x00800000  /* Disable RX Watchdog timeout    */
#define FGMAC4_MCR_JD		0x00400000  /* Disable TX Jabber timer        */
#define FGMAC4_MCR_BE		0x00200000  /* Frame Burst Enable             */
#define FGMAC4_MCR_JE		0x00100000  /* Jumbo Frame Enable             */
#define FGMAC4_MCR_IFG		0x000E0000  /* Inter-Frame GAP                */
#define FGMAC4_MCR_DCRS		0x00010000  /* Disable Carrier During Trans   */
#define FGMAC4_MCR_PS		0x00008000  /* Port Select 0:GMII,1:MII       */
#define FGMAC4_MCR_FES		0x00004000  /* Speed                          */
#define FGMAC4_MCR_DO		0x00002000  /* Disable Receive Own            */
#define FGMAC4_MCR_LM		0x00001000  /* Loop-back Mode                 */
#define FGMAC4_MCR_DM		0x00000800  /* Duplex mode                    */
#define FGMAC4_MCR_IPC		0x00000400  /* Cehcksum Offload               */
#define FGMAC4_MCR_DR		0x00000200  /* Disable Retry                  */
#define FGMAC4_MCR_LUD		0x00000100  /* Link Up/Down                   */
#define FGMAC4_MCR_ACS		0x00000080  /* Automatic Pad/CRC Stripping    */
#define FGMAC4_MCR_BL_00	0x00000000  /* Back-off Limit is setted 0     */
#define FGMAC4_MCR_DC		0x00000010  /* Deferral Check                 */
#define FGMAC4_MCR_TX_ENABLE	0x00000008  /* Enable Transmitter             */
#define FGMAC4_MCR_RX_ENABLE	0x00000004  /* Enable Receiver                */

#define FGMAC4_MCR_IFG_MASK	0x000E0000
#define FGMAC4_MCR_IFG_96	0x00000000
#define FGMAC4_MCR_IFG_88	0x00020000
#define FGMAC4_MCR_IFG_80	0x00040000
#define FGMAC4_MCR_IFG_72	0x00060000
#define FGMAC4_MCR_IFG_64	0x00080000
#define FGMAC4_MCR_IFG_56	0x000A0000
#define FGMAC4_MCR_IFG_48	0x000C0000
#define FGMAC4_MCR_IFG_40	0x000E0000

/* bit15:PS(Port Select) */
#define FGMAC4_MCR_GMII_PORT	0x0
#define FGMAC4_MCR_MII_PORT	0x00008000
/* bit11:DM(Duplex mode) */
#define FGMAC4_MCR_HALF_DUPLEX	0x0
#define FGMAC4_MCR_FULL_DUPLEX	0x00000800

/* MFFR:MAC Frame Filter Register */

/* Alias definitions */
#define FGMAC4_MFFR_RA		0x80000000  /* Receive All                    */
#define FGMAC4_MFFR_HPF		0x00000400  /* Hash or Perfect Filter         */
#define FGMAC4_MFFR_SAF		0x00000200  /* Source Address Filter          */
#define FGMAC4_MFFR_SAIF	0x00000100  /* Source Address Inverse Filter  */
#define FGMAC4_MFFR_PCF		0x000000C0  /* Pass Control Frames            */
#define FGMAC4_MFFR_DB		0x00000020  /* Disable Broadcast Frames       */
#define FGMAC4_MFFR_PM		0x00000010  /* Pass All Multicast             */
#define FGMAC4_MFFR_DAIF	0x00000008  /* DA Inverse Filtering           */
#define FGMAC4_MFFR_HMC		0x00000004  /* Hash Multicast                 */
#define FGMAC4_MFFR_HUC		0x00000002  /* Hash Unicast                   */
#define FGMAC4_MFFR_PR		0x00000001  /* Premiscous Mode                */

/* GAR:GMII Address Register */
#define FGMAC4_GAR_GW_R		0x00000000  /* GMII/MII Read                  */
#define FGMAC4_GAR_GW_W		0x00000002  /* GMII/MII Write                 */
#define FGMAC4_GAR_GB		0x00000001  /* GMII/MII Busy                  */
#define FGMAC4_GAR_PA_SHIFT	11          /* PHY address bit field shift    */
#define FGMAC4_GAR_GR_MASK	0x0000001F  /* GMII register field mask       */
#define FGMAC4_GAR_GR_SHIFT	6           /* GMII register field shift      */
#define FGMAC4_GAR_CR_MASK	0x0000000F  /* Clock range field mask         */
#define FGMAC4_GAR_CR_SHIFT	2           /* Clock range field shift        */

/* RGSR:RGMII Status Register */
#define FGMAC4_RGSR_LS		0x00000008  /* Link Status                    */
#define FGMAC4_RGSR_LSP		0x00000006  /* Link Speed                     */
#define FGMAC4_RGSR_LSP_10M	0x00000000  /* 2.5MHz for 10Mbps              */
#define FGMAC4_RGSR_LSP_100M	0x00000002  /* 25MHz for 100Mbps              */
#define FGMAC4_RGSR_LSP_1G	0x00000004  /* 125MHz for Gbps                */
#define FGMAC4_RGSR_LM		0x00000001  /* Link Mode                      */
#define FGMAC4_RGSR_LM_FULL	0x00000001  /* Full Duplex Mode               */

/* BMR:MDC Bus Mode Register */
#define FGMAC4_BMR_XPBL		0x01000000  /* MAX burst is N-times of PBL    */

#define FGMAC4_BMR_UPS		0x00800000  /* RXburst is RPBL,TXburst is PBL */
#define FGMAC4_BMR_RPBL_32	0x00400000  /* RX Burst Length is 32 Bytest   */
#define FGMAC4_BMR_RPBL_16	0x00200000  /* RX Burst Length is 16 Bytest   */
#define FGMAC4_BMR_RPBL_8	0x00100000  /* RX Burst Length is 8 Bytest    */
#define FGMAC4_BMR_RPBL_4	0x00080000  /* RX Burst Length is 4 Bytest    */
#define FGMAC4_BMR_RPBL_2	0x00040000  /* RX Burst Length is 2 Bytest    */
#define FGMAC4_BMR_RPBL_1	0x00020000  /* RX Burst Length is 1 Bytest    */
#define FGMAC4_BMR_FB		0x00010000  /* AHB Burst Mode                 */
#define FGMAC4_BMR_PR_00	0x00000000  /* RX TX Priority ratio is 1:1    */
#define FGMAC4_BMR_PR_01	0x00004000  /* RX TX Priority ratio is 2:1    */
#define FGMAC4_BMR_PR_10	0x00008000  /* RX TX Priority ratio is 3:1    */
#define FGMAC4_BMR_PR_11	0x0000C000  /* RX TX Priority ratio is 4:1    */
#define FGMAC4_BMR_PBL_32	0x00002000  /* Burst Length is 32 Bytes       */
#define FGMAC4_BMR_PBL_16	0x00001000  /* Burst Length is 16 Bytes       */
#define FGMAC4_BMR_PBL_8	0x00000800  /* Burst Length is 8 bytes        */
#define FGMAC4_BMR_PBL_4	0x00000400  /* Burst Length is 4 Bytes        */
#define FGMAC4_BMR_PBL_2	0x00000200  /* Burst Length is 2 Bytes        */
#define FGMAC4_BMR_PBL_1	0x00000100  /* Burst Length is 1 Bytes        */
#define FGMAC4_BMR_ATDS		0x00000080  /* Alternate Descriptor Size      */
#define FGMAC4_BMR_DSL		0x00000000  /* Descripter Skip Length         */
#define FGMAC4_BMR_DA		0x00000002  /* RX have priority               */
#define FGMAC4_BMR_SOFTWARE_RESET 0x00000001  /* software reset, init all regs*/

/* OMR:MDC Operation Mode Register */
#define FGMAC4_OMR_RSF		0x02000000  /* RX after whole frame in FIFO   */
#define FGMAC4_OMR_TSF		0x00200000  /* TX after whole frame in FIFO   */
#define FGMAC4_OMR_START_TX	0x00002000  /* Start Transmission             */
#define FGMAC4_OMR_START_RX	0x00000002  /* Start Receive                  */
#define FGMAC4_OMR_TTC_64B	0x00000000  /* TX after 64B written in FIFO   */
#define FGMAC4_OMR_TTC_128B	0x00004000  /* TX after 128B written in FIFO  */
#define FGMAC4_OMR_TTC_192B	0x00008000  /* TX after 192B written in FIFO  */
#define FGMAC4_OMR_TTC_256B	0x0000C000  /* TX after 256B written in FIFO  */
#define FGMAC4_OMR_TTC_40B	0x00010000  /* TX after 40B written in FIFO   */
#define FGMAC4_OMR_TTC_32B	0x00014000  /* TX after 32B written in FIFO   */
#define FGMAC4_OMR_TTC_24B	0x00018000  /* TX after 24B written in FIFO   */
#define FGMAC4_OMR_TTC_18B	0x0001C000  /* TX after 18B written in FIFO   */

/* IER:MDC Interrupt Enable Register */
#define FGMAC4_IER_INT_MASK	0x00000000
#define FGMAC4_IER_INT_ENABLE	0xffffffff
#define FGMAC4_IER_INT_NORMAL	0x00010000  /* Normal Interrupt Summary       */
#define FGMAC4_IER_INT_ABNORMAL	0x00008000  /* Abnormal Interrupt Summary     */
#define FGMAC4_IER_INT_EARY_RX	0x00004000  /* Eary Receive Int               */
#define FGMAC4_IER_INT_BUS_ERR	0x00002000  /* Fatal Bus Error Int            */
#define FGMAC4_IER_INT_EARY_TX	0x00000400  /* Eary Transmit Int              */
#define FGMAC4_IER_INT_RX_WT	0x00000200  /* Receive Watchdog Timeout Int   */
#define FGMAC4_IER_INT_RX_STOP	0x00000100  /* Receive Process Stop Int       */
#define FGMAC4_IER_INT_RXB_UNAV	0x00000080  /* RX Buffer Unavailable Int      */
#define FGMAC4_IER_INT_RX_INT	0x00000040  /* Receive Int                    */
#define FGMAC4_IER_INT_TX_UFLOW	0x00000020  /* Transmit Underflow Int         */
#define FGMAC4_IER_INT_RX_OVERF	0x00000010  /* Receive Overlow Int            */
#define FGMAC4_IER_INT_TX_JT	0x00000008  /* TX Jabber Timeout Int          */
#define FGMAC4_IER_INT_TXB_UNAV	0x00000004  /* TX Buffer Unavailable Int      */
#define FGMAC4_IER_INT_TX_STOP	0x00000002  /* Transmit Process Stop Int      */
#define FGMAC4_IER_INT_TX_INT	0x00000001  /* Transmit Int                   */
#ifdef CONFIG_FGMAC4_MMC
#define FGMAC4_CHECK_INT	0x080067FF  /* All interrupts                 */
#else
#define FGMAC4_CHECK_INT	0x000067FF  /* All interrupts                 */
#endif /* CONFIG_FGMAC4_MMC */

#define FGMAC4_SR_GMI	0x08000000  /* MMC interrupt status                   */
#define FGMAC4_SR_FBI	0x00002000  /* Fatal Bus Error Int status             */
#define FGMAC4_SR_RPS	0x00000100  /* Receive Process Stop Int status        */
#define FGMAC4_SR_RU	0x00000080  /* RX Buffer Unavailable Int status       */
#define FGMAC4_SR_RI	0x00000040  /* Receive Int status                     */
#define FGMAC4_SR_UNF	0x00000020  /* Transmit Underflow Int status          */
#define FGMAC4_SR_OVF	0x00000010  /* Receive Overlow Int status             */
#define FGMAC4_SR_TU	0x00000004  /* TX Buffer Unavailable Int status       */
#define FGMAC4_SR_TPS	0x00000002  /* Transmit Process Stop Int status       */
#define FGMAC4_SR_TI	0x00000001  /* Transmit Int status                    */

/* PMTR:PMT Register */
#define FGMAC4_PMTR_RWFFRPR	0x80000000  /* Remote Wakeup Frame Filter
					     * Register Pointer Reset
					     */
#define FGMAC4_PMTR_GU		0x00000200  /* Global Unicast                 */
#define FGMAC4_PMTR_WPR		0x00000040  /* Wake Up Frame Receive          */
#define FGMAC4_PMTR_MPR		0x00000020  /* Magic Packet Received          */
#define FGMAC4_PMTR_WFE		0x00000004  /* Wake-Up Frame Enable           */
#define FGMAC4_PMTR_MPE		0x00000002  /* Magic Packet Enable            */
#define FGMAC4_PMTR_PD		0x00000001  /* Power Down                     */

/* LPICSR:LPI Control and Status Register */
#define FGMAC4_LPICSR_LPITXA	0x00080000 /* LPI TX Automate */
#define FGMAC4_LPICSR_PLSEN	0x00040000 /* PHY Link Status Enable */
#define FGMAC4_LPICSR_PLS	0x00020000 /* PHY Link Status */
#define FGMAC4_LPICSR_LPIEN	0x00010000 /* LPI Enable */
#define FGMAC4_LPICSR_RLPIST	0x00000200 /* Receive LPI State */
#define FGMAC4_LPICSR_TLPIST	0x00000100 /* Transmit LPI State */
#define FGMAC4_LPICSR_RLPIEX	0x00000008 /* Receive LPI Exit */
#define FGMAC4_LPICSR_RLPIEN	0x00000004 /* Receive LPI Entry */
#define FGMAC4_LPICSR_TLPIEX	0x00000002 /* Transmit LPI Exit */
#define FGMAC4_LPICSR_TLPIEN	0x00000001

/* MMC_CNTL:MMC Control Register */
/* Auto clear after is readed */
#define FGMAC4_MMC_CNTL_RESET_ON_READ	0x00000004
/* Stop when arrive MAX       */
#define FGMAC4_MMC_CNTL_STOP_ROLL_OVER	0x00000002
/* Clear all MMC register     */
#define FGMAC4_MMC_CNTL_CLEAR_ALL	0x00000001
/* Mask MMC interrupt         */
#define FGMAC4_MMC_CNTL_MMC_INTR_MASK	0xFFFFFFFF

/* Descriptor Status */
#define FGMAC4_DESC_OWN_BIT		0x80000000
#define FGMAC4_TX_DESC_OWN_BIT		0x80000000
#define FGMAC4_RX_DESC_OWN_BIT		0x80000000

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_TX_DESC_INT_COMPLET	0x40000000
#else
#define FGMAC4_DESC_INT_COMPLET		0x80000000
#define FGMAC4_TX_DESC_INT_COMPLET	0x80000000
#endif

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_TX_DESC_LAST_SEG		0x20000000
#else
#define FGMAC4_DESC_LAST_SEG		0x40000000
#define FGMAC4_TX_DESC_LAST_SEG		0x40000000
#endif

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_TX_DESC_FIRST_SEG	0x10000000
#else
#define FGMAC4_DESC_FIRST_SEG		0x20000000
#define FGMAC4_TX_DESC_FIRST_SEG	0x20000000
#endif

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_RX_DESC_END_RING		0x00008000
#define FGMAC4_TX_DESC_END_RING		0x00200000
#else
#define FGMAC4_DESC_END_RING		0x02000000
#define FGMAC4_TX_DESC_END_RING		0x02000000
#define FGMAC4_RX_DESC_END_RING		0x02000000
#endif

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_DESC_TX_DP		0x04000000
#else
#define FGMAC4_DESC_TX_DP		0x00800000
#endif
/* RX descriptor error status
 *  AFM[bit30]:Destination Address Filter Fail(FGMAC4)
 *  ES[bit15]:Error Summary
 *  DE[bit14]:Not receive the whole frame to buffer
 *  SAF[bit13]:Source Address Filter Fail(FGMAC4)
 *  LE[bit12]:Length Error
 *  OE[bit11]:Overflow Error
 *  VLAN[bit10]:VLAN Frame. This driver does not support VLAN.(FGMAC4)
 *  FS[bit9]:First Descriptor(FGMAC4)
 *  LS[bit8]:Last Descriptor(FGMAC4)
 *  GF[bit7]:Giant Frame(FGMAC4)
 *  IPC[bit7]:IPC Error(FGMAC4)
 *  LC[bit6]:Late Collision(FGMAC4)
 *  FT[bit5]:Frame Type(FGMAC4)
 *  RWT[bit4]:Receive watchdog timeout
 *  RE[bit3]:Receive Error
 *  DE[bit2]:DribleBit Error
 *  CE[bit1]:CRC Error
 *  RMCE[bit0]:Rx MAC Address/Playload Checksum Error(FGMAC4)
 */
#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_RX_DESC_ERR_IPC		0x0000F85E
#define FGMAC4_RX_DESC_ERR		0x0000F85E
#define FGMAC4_RX_DESC_IPE_ERR		0x00000010
#define FGMAC4_RX_DESC_IPHE_ERR		0x00000008
#else
#define FGMAC4_RX_DESC_ERR_IPC		0x0000F8DF
#define FGMAC4_RX_DESC_ERR		0x0000F8DE
#endif
#define FGMAC4_RX_DESC_AFM_ERR		0x40000000
#define FGMAC4_RX_DESC_ES_ERR		0x00008000
#define FGMAC4_RX_DESC_DE_ERR		0x00004000
#define FGMAC4_RX_DESC_SAF_ERR		0x00002000
#define FGMAC4_RX_DESC_LE_ERR		0x00001000
#define FGMAC4_RX_DESC_OE_ERR		0x00000800
#define FGMAC4_RX_DESC_VLAN		0x00000400
#define FGMAC4_RX_DESC_FS		0x00000200
#define FGMAC4_RX_DESC_LS		0x00000100
#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_RX_DESC_TS		0x00000080
#else
#define FGMAC4_RX_DESC_IPC_ERR		0x00000080
#endif
#define FGMAC4_RX_DESC_GF_ERR		0x00000080
#define FGMAC4_RX_DESC_LC_ERR		0x00000040
#define FGMAC4_RX_DESC_FT		0x00000020
#define FGMAC4_RX_DESC_RWT_ERR		0x00000010
#define FGMAC4_RX_DESC_RE_ERR		0x00000008
#define FGMAC4_RX_DESC_DBE_ERR		0x00000004
#define FGMAC4_RX_DESC_CE_ERR		0x00000002

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_RX_DESC_ESA		0x00000001
#else
#define FGMAC4_RX_DESC_RMCE_ERR		0x00000001
#endif

#define FGMAC4_RX_DESC_FL_SHIFT		16
#define FGMAC4_RX_DESC_FL_MASK		0x00003FFF

/* TX descriptor error status
 * IHE[bit16]:IP Header Error(FGMAC4)
 * ES[bit15]:Error Summary
 * JT[bit14]:Jabber timeout
 * FF[bit13]:Frame Flushed(FGMAC4)
 * PCE[bit12]:Payload Checksum Error(FGMAC4)
 * LC[bit11]:Loss of carrier
 * NC[bit10]:No carrier
 * LCO[bit9]:Late collision
 * EC[bit8]:Excessive collision
 * VF[bit7]:VLAN frame. This driver does not support VLAN.(FGMAC4)
 * ED[bit2]:Excessive deferral
 * UF[bit1]:Undeflow error
 */

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_TX_DESC_CIC0		0x00000000
#define FGMAC4_TX_DESC_CIC1		0x00400000
#define FGMAC4_TX_DESC_CIC2		0x00800000
#define FGMAC4_TX_DESC_CIC3		0x00C00000
#define FGMAC4_TX_DESC_CIC_MASK		0x00C00000
#else
#define FGMAC4_TX_DESC_CIC0		0x00000000
#define FGMAC4_TX_DESC_CIC1		0x08000000
#define FGMAC4_TX_DESC_CIC2		0x10000000
#define FGMAC4_TX_DESC_CIC3		0x18000000
#define FGMAC4_TX_DESC_CIC_MASK		0x18000000
#endif

#ifdef CONFIG_FGMAC4_ALT_DESC
#define FGMAC4_TX_DESC_ERR		0x0001CF86
#else
#define FGMAC4_TX_DESC_ERR		0x0000CF86
#endif
#define FGMAC4_TX_DESC_LC_ERR		0x00000800
#define FGMAC4_TX_DESC_NC_ERR		0x00000400
#define FGMAC4_TX_DESC_LCO_ERR		0x00000200
#define FGMAC4_TX_DESC_EC_ERR		0x00000100
/* VF[bit7]:VLAN frame. This driver does not support VLAN. */
#define FGMAC4_TX_DESC_VLAN		0x00000080

#define FGMAC4_TX_DESC_ED_ERR		0x00000004
#define FGMAC4_TX_DESC_UF_ERR		0x00000002

#define FGMAC4_TX_DESC_CC_SHIFT		3
#define FGMAC4_TX_DESC_CC_MASK		0x0000000F

/* get the number of TX/RX descriptor from Kconfig */
#ifndef CONFIG_FGMAC4_TDESC_NUM
#define FGMAC4_TDESC_NUM	256
#else
#define FGMAC4_TDESC_NUM	CONFIG_FGMAC4_TDESC_NUM
#endif /* CONFIG_FGMAC4_TDESC_NUM */
#ifndef CONFIG_FGMAC4_RDESC_NUM
#define FGMAC4_RDESC_NUM	256
#else
#define FGMAC4_RDESC_NUM	CONFIG_FGMAC4_RDESC_NUM
#endif /* CONFIG_FGMAC4_RDESC_NUM */

/* print message */
#define PMSG(dev, ctl, fmt, args...)	dev_printk(ctl, dev, fmt, ##args)

/* Debug message control */
#define DBGH	"DEBUG: "	/* The header of debug message */
#define DUMPH	"       "	/* The header of dump message  */

#ifdef DEBUG
#if (1 < DEBUG)
#define FGMAC4_DEBUG	1
#endif
#endif	/* DEBUG */

/* This driver can spew a whole lot of debugging output at you. If you
 * need maximum performance, you should disable the FGMAC4_DEBUG define.
 */
#ifdef FGMAC4_DEBUG
/* Format:DEBUG: THE MESSAGE */
#define DBG_PRINT(dev, fmt, args...)	dev_dbg(dev, DBGH fmt, ##args)
/* Format:DEBUG: FUNCTION START */
#define DBG_FUN_START(dev)		dev_dbg(dev, DBGH "%s START\n", \
							 __func__)
/* Format:DEBUG: FUNCTION END */
#define DBG_FUN_END(dev)		dev_dbg(dev, DBGH "%s END\n", \
							 __func__)
/* Format:DEBUG: FUNCTION ABNORMAL END */
#define DBG_FUN_ERREND(dev)		dev_dbg(dev, DBGH \
					"%s ABNORMAL END!!!\n", __func__)
#else
#define DBG_PRINT(dev, fmt, args...)	/* print nothing */
#define DBG_FUN_START(dev)		/* print nothing */
#define DBG_FUN_END(dev)		/* print nothing */
#define DBG_FUN_ERREND(dev)		/* print nothing */
#endif /* FGMAC4_DEBUG */

/* Dump control */
#define REG_DUMP		0x01	/* dump register value   */
#define DESC_DUMP		0x02	/* dump descriptor value */
#define MEM_DUMP		0x04	/* dump memory value     */

#ifdef FGMAC4_DUMP
static void debug_dump(struct device *dev, void __iomem *base_addr, int ctl,
		     int *txring, int *rxring, char *startp, unsigned int size);

#define DBG_DUMP(dev, a, b, c, d, e, f)	debug_dump(dev, (a), (b), (c), \
							(d), (e), (f))
#else
#define DBG_DUMP(dev, a, b, c, d, e, f)	/* dump nothing  */
#endif /* FGMAC4_DUMP */

/* struct definition for tx and rx descriptor
 */
struct fgmac4_desc {
	u32 opts1;
	u32 opts2;
	u32 addr1;
	u32 addr2;
	/*used by T serial FGMAC4 */
#ifdef CONFIG_FGMAC4_ALT_DESC
	/* for rx descriptor, this is extended status descriptor;
	 * for tx descriptor, this is a reserved descriptor.
	 */
	u32 ex_status_rsv;
	u32 reserved;		/* reserved */
	u32 time_stmp_low;	/* time stamp low */
	u32 time_stmp_high;	/* time stamp high */
#endif
};

struct fgmac4_buff_info {
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	unsigned char *data;
	unsigned int len;
#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */
	struct sk_buff *skb;	/* socket buffer's pointer           */
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */
	dma_addr_t mapping;	/* socket buffer's streaming mapping */
};

struct fgmac4_phy_info {
	unsigned int adv;
	unsigned int speed;
	unsigned int duplex;
	unsigned int autoneg;
	unsigned int phy_addr;
};

#ifdef CONFIG_FGMAC4_MMC
struct fgmac4_mmc_stats {
	/* Statistics maintained by MMC. */
	u64 txoctetcount_gb;
	u64 txframecount_gb;
	u64 txbroadcastframes_g;
	u64 txmulticastframes_g;
	u64 tx64octets_gb;
	u64 tx65to127octets_gb;
	u64 tx256to511octets_gb;
	u64 tx512to1023octets_gb;
	u64 tx1024tomaxoctets_gb;
	u64 txunicastframes_gb;
	u64 txmulticastframes_gb;
	u64 txbroadcastframes_gb;
	u64 txunderflowerror;
	u64 txsinglecol_g;
	u64 txmulticol_g;
	u64 txdeferred;
	u64 txlatecol;
	u64 txexesscol;
	u64 txcarriererror;
	u64 txoctetcount_g;
	u64 txframecount_g;
	u64 txexecessdef;
	u64 txpauseframes;
	u64 txvlanframes_g;
	u64 rxframecount_gb;
	u64 rxoctetcount_gb;
	u64 rxoctetcount_g;
	u64 rxbroadcastframes_g;
	u64 rxmulticastframes_g;
	u64 rxcrcerror;
	u64 rxalignmenterror;
	u64 rxrunterror;
	u64 rxjabbererror;
	u64 rxundersize_g;
	u64 rxoversize_g;
	u64 rx64octets_gb;
	u64 rx65to127octets_gb;
	u64 rx128to255octets_gb;
	u64 rx256to511octets_gb;
	u64 rx512to1023octets_gb;
	u64 rx1024tomaxoctets_gb;
	u64 rxunicastframes_g;
	u64 rxlengtherror;
	u64 rxoutofrangetype;
	u64 rxpauseframes;
	u64 rxfifooverflow;
	u64 rxvlanframes_gb;
	u64 rxwatchdogerror;
	u64 rxipv4_gd_frms;
	u64 rxipv4_hdrerr_frms;
	u64 rxipv4_nopay_frms;
	u64 rxipv4_frag_frms;
	u64 rxipv4_udsbl_frms;
	u64 rxipv6_gd_frms;
	u64 rxipv6_hdrerr_frms;
	u64 rxipv6_nopay_frms;
	u64 rxudp_gd_frms;
	u64 rxudp_err_frms;
	u64 rxtcp_gd_frms;
	u64 rxtcp_err_frms;
	u64 rxicmp_gd_frms;
	u64 rxicmp_err_frms;
	u64 rxipv4_gd_octets;
	u64 rxipv4_hdrerr_octets;
	u64 rxipv4_nopay_octets;
	u64 rxipv4_frag_octets;
	u64 rxipv4_udsbl_octets;
	u64 rxipv6_gd_octets;
	u64 rxipv6_hdrerr_octets;
	u64 rxipv6_nopay_octets;
	u64 rxudp_gd_octets;
	u64 rxudp_err_octets;
	u64 rxtcp_gd_octets;
	u64 rxtcp_err_octets;
	u64 rxicmp_gd_octets;
	u64 rxicmp_err_octets;
};
#endif /* CONFIG_FGMAC4_MMC */

struct fgmac4_private {
	u32 tx_head;			/* The head of free TX Desc ring     */
	u32 tx_tail;			/* The tail of used TX Desc ring     */
	u32 rx_tail;			/* The head of free RX Desc ring     */
	dma_addr_t ring_dma;		/* base address of descriptor memory */
	struct fgmac4_desc *rx_ring;	/* RX Desc ring's pointer            */
	struct fgmac4_desc *tx_ring;	/* TX Desc ring's pointer            */
	u32 rx_buf_sz;			/* MAX size of socket buffer         */
	u32 tx_buf_sz;			/* MAX size of tx socket buffer      */
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	unsigned char *rxbuf_p;		/* Rx buffer pointer */
	unsigned char *txbuf_p;		/* Tx buffer pointer */
	dma_addr_t rxbuf_dma;		/* Rx buffer dma mapping address */
	dma_addr_t txbuf_dma;		/* Tx buffer dma mapping address */
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */
	u32 buff1_cnt;			/* Bytes of buffer 1 in T/RDES1 */
	u32 buff2_cnt;			/* Bytes of buffer 2 in T/RDES1 */

	/* TX socketbuffer array */
	struct fgmac4_buff_info tx_skb[FGMAC4_TDESC_NUM];
	/* RX socketbuffer array */
	struct fgmac4_buff_info rx_skb[FGMAC4_RDESC_NUM];

	int phy_map;			/* install PHY's information         */
	int mdc_clk;			/* MDC clock                         */
	void __iomem *base_addr;	/* physical address of FGMAC4 reg    */
	spinlock_t lock;		/* spin lock                         */

	struct net_device *dev;		/* net_device pointer                */
	struct device *device;		/* device pointer                    */
	struct net_device_stats net_stats;	/* TX&RX packet's statistics */

	/* use mii.c common fuction to support Ethtool by this struct        */
	struct mii_if_info mii;		/* the information about MII         */
	struct fgmac4_phy_info phy_info;/* the information about phy         */
	unsigned int init_media; /* 0: media not init, 1: media init */

	/* check carrier timely */
	struct timer_list timer;
#ifdef CONFIG_FGMAC4_NAPI
	struct napi_struct napi;	/* for NAPI API                      */
#endif /* CONFIG_FGMAC4_NAPI */
	u32 wol_irq;			/* irq no. of wakeup-on-lan	     */
	u32 wol_opts;			/* current wol settings              */
	u32 rx_csum;			/* rx offload engine, 1:on, 0:off    */

	/* multicast setting stored*/
	u32 mhtrh;
	u32 mhtrl;
	u32 mffr;

	struct mii_bus	*mdio_bus;
	int		mdio_irq[PHY_MAX_ADDR];
	struct clk *h_clk;		/* sys(AHB) clock                    */
	struct clk *osc_clk;		/* osc clock                         */
	struct clk *p_clk;
#ifdef CONFIG_FGMAC4_MMC
	struct fgmac4_mmc_stats mmc_stats;
#endif /* CONFIG_FGMAC4_MMC */
	int eee_capable;
	int eee_enabled;

	int gpio_phy_enable;
	int gpio_phy_nrst;
};

/* The definition used in this driver
 */

#define BUFFER_SIZE L1_CACHE_ALIGN(CONFIG_FGMAC4_BUFFER_SIZE)

#ifdef CONFIG_FGMAC4_ALT_DESC
/* Max bytes of buffer1, and it is L1_CACHE_BYTES aligned. */
#define BUFFER1_MAX_CNT (L1_CACHE_ALIGN((1 << 13) - 1 - L1_CACHE_BYTES))

/* Shift of Transmit/Receive Buffer 2 Size)*/
#define TRBS2_SHIFT 16

#else /* !CONFIG_FGMAC4_ALT_DESC */
/* Max bytes of buffer1, and it is L1_CACHE_BYTES aligned. */
#define BUFFER1_MAX_CNT (L1_CACHE_ALIGN((1 << 12) - 1 - L1_CACHE_BYTES))

/* Shift of Transmit/Receive Buffer 2 Size)*/
#define TRBS2_SHIFT 11
#endif /* CONFIG_FGMAC4_ALT_DESC */

#define ETH_JUMBO_DATA_LEN	9000

#define HASH_TABLE_SIZE 64

/* After reset device, have to wait 2000ns before access the register        */
#define DELAY_TIME		2			/* 2us = 2000ns      */

/* hope the option will end within this time */
#define WAIT_TIME		1000
#define ANEG_WAIT_TIME		(500*1000)

/* check carrier every 1s */
#define CHECK_CARRIER_TIME	(jiffies + HZ)

/* the number of free descriptors */
#define TX_RING_AVAIL(RP)                                               \
	(((RP)->tx_tail <= (RP)->tx_head) ?                             \
	 (RP)->tx_tail + (FGMAC4_RDESC_NUM - 1) - (RP)->tx_head :       \
	 (RP)->tx_tail - (RP)->tx_head - 1)

#define NEXT_TX(N)	(((N) + 1) % FGMAC4_TDESC_NUM)
#define NEXT_RX(N)	(((N) + 1) % FGMAC4_RDESC_NUM)

/* the size of descriptor spaces */
#define FGMAC4_DESC_BYTES  (((sizeof(struct fgmac4_desc) * FGMAC4_TDESC_NUM)) +\
			   ((sizeof(struct fgmac4_desc) * FGMAC4_RDESC_NUM)))

/* Read MMC register */
#define READ_MMC(REG)	(fgmac4_reg_read(lp->base_addr, (REG)))

/* Send engin error number */
#define FGMAC4_TX_OK	NETDEV_TX_OK		/* driver took care of packet*/
#define FGMAC4_TX_BUSY	NETDEV_TX_BUSY		/* driver tx path was busy   */

#endif /* __NET_FGMAC4_H */
