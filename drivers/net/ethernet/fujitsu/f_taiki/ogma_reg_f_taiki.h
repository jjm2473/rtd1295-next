/**
 * ogma_reg_f_taiki.h
 *
 *  Copyright (c) 2012 - 2013 Fujitsu Semiconductor Limited.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
 */
#ifndef OGMA_REG_F_TAIKI_H
#define OGMA_REG_F_TAIKI_H

#define OGMA_REG_ADDR_TOP_STATUS                 (0x80)
#define OGMA_REG_ADDR_TOP_INTEN                  (0x81)
#define OGMA_REG_ADDR_TOP_INTEN_SET              (0x8d)
#define OGMA_REG_ADDR_TOP_INTEN_CLR              (0x8e)
#define OGMA_REG_ADDR_NRM_TX_STATUS              (0x100)
#define OGMA_REG_ADDR_NRM_TX_INTEN               (0x101)
#define OGMA_REG_ADDR_NRM_TX_INTEN_SET           (0x10a)
#define OGMA_REG_ADDR_NRM_TX_INTEN_CLR           (0x10b)
#define OGMA_REG_ADDR_NRM_RX_STATUS              (0x110)
#define OGMA_REG_ADDR_NRM_RX_INTEN               (0x111)
#define OGMA_REG_ADDR_NRM_RX_INTEN_SET           (0x11a)
#define OGMA_REG_ADDR_NRM_RX_INTEN_CLR           (0x11b)
#define OGMA_REG_ADDR_PKTC_CMD_BUF               (0x34)
#define OGMA_REG_ADDR_DMAC_HM_CMD_BUF            (0x84)
#define OGMA_REG_ADDR_DMAC_MH_CMD_BUF            (0x87)
#define OGMA_REG_ADDR_DIS_CORE                   (0x86)
#define OGMA_REG_ADDR_CLK_EN                     (0x40)
#define OGMA_REG_ADDR_SOFT_RST                   (0x41)
#define OGMA_REG_ADDR_PKT_CTRL                   (0x50)
#define OGMA_REG_ADDR_COM_INIT                   (0x48)
#define OGMA_REG_ADDR_DMA_TMR_CTRL               (0x83)
#define OGMA_REG_ADDR_F_TAIKI_MC_VER             (0x8b)
#define OGMA_REG_ADDR_F_TAIKI_VER                (0x8c)
#define OGMA_REG_ADDR_DMA_HM_CTRL                (0x85)
#define OGMA_REG_ADDR_DMA_MH_CTRL                (0x88)
#define OGMA_REG_ADDR_NRM_TX_PKTCNT              (0x104)
#define OGMA_REG_ADDR_NRM_TX_DONE_TXINT_PKTCNT   (0x106)
#define OGMA_REG_ADDR_NRM_RX_RXINT_PKTCNT        (0x116)
#define OGMA_REG_ADDR_NRM_TX_TXINT_TMR           (0x108)
#define OGMA_REG_ADDR_NRM_RX_RXINT_TMR           (0x118)
#define OGMA_REG_ADDR_NRM_TX_DONE_PKTCNT         (0x105)
#define OGMA_REG_ADDR_NRM_RX_PKTCNT              (0x115)
#define OGMA_REG_ADDR_NRM_TX_TMR                 (0x107)
#define OGMA_REG_ADDR_NRM_RX_TMR                 (0x117)
#define OGMA_REG_ADDR_NRM_TX_DESC_START          (0x102)
#define OGMA_REG_ADDR_NRM_RX_DESC_START          (0x112)
#define OGMA_REG_ADDR_RESERVED_RX_DESC_START     (0x122)
#define OGMA_REG_ADDR_RESERVED_TX_DESC_START     (0x132)
#define OGMA_REG_ADDR_NRM_TX_CONFIG              (0x10c)
#define OGMA_REG_ADDR_NRM_RX_CONFIG              (0x11c)
#define OGMA_REG_ADDR_MAC_DATA                   (0x470)
#define OGMA_REG_ADDR_MAC_CMD                    (0x471)
#define OGMA_REG_ADDR_MAC_FLOW_TH                (0x473)
#define OGMA_REG_ADDR_MAC_INTF_SEL               (0x475)
#define OGMA_REG_ADDR_MAC_DESC_INIT              (0x47f)
#define OGMA_REG_ADDR_MAC_DESC_SOFT_RST          (0x481)
#define OGMA_REG_ADDR_MODE_TRANS_COMP_STATUS     (0x140)

#endif /* OGMA_REG_F_TAIKI_H */
