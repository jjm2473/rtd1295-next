/*
 * mipilli_mb86s70.h F_MIPILLI_LP Controller Driver
 * Copyright (C) 2013 Fujitsu Semi, Ltd
 * Author: Slash Huang <slash.huang@tw.fujitsu.com>
 *
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This code is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef MIPILLI_MB86S70_H_
#define MIPILLI_MB86S70_H_

#include <linux/cdev.h>
#include <linux/compat.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include "mipilli_api.h"
#include "mipilli_registers.h"

#define MB86S70_DRV_VER 1000
#define DRIVER_NAME "lli"
#define MODULE_NAME "mb86s70-mipilli"

/* #define DBG_IO_CTL */
#define DBG_PROB 1
#define DBG_FLAG 1
#define ERR_FLAG 1
#define DBG_S_MOUNT 1
#define DBG_M_UNMOUNT 1
#define DBG_M_MOUNT 1
#define DBG_CONFIG_UPDATE 1

#if DBG_FLAG
#define DBG_MSG(fmt, arg...) printk(fmt, ##arg)
#else
#define DBG_MSG(fmt, arg...)
#endif

#if ERR_FLAG
#define ERR_MSG(fmt, arg...) printk(fmt, ##arg)
#else
#define ERR_MSG(fmt, arg...)
#endif


#define ACT_LLI_CH  (gp_mb86s70_lli_dev->act_lli_ch)
#define AS_MASTER   1
#define AS_SLAVE    0
#define LOCAL_ADDR_BIT  0x8000
#define RETRY_COUNTER   200
#define MPHY_WRITE_TAG_LOCAL    0x01
#define MPHY_WRITE_TAG_REMOTE   0x02
#define LLI_CLK_NUM 3

struct lli_data {
	dev_t                devt;
	struct device        *dev;
	struct cdev          cdev;
	struct lli_cfg       *lli_defcfg;
	struct class         *lli_class;
	struct completion    msg_cmp;
	struct task_struct   *ptr_slave_th;
	struct semaphore     openf_sem;
	void __iomem         *regs;
	struct resource      *rcs;
	/* void __iomem        *lli_mem_if; */
	u32                   irq;
	u32                   lli_interrupt_status;
	u8                    mount_stat;
	u8                    master_or_slave; /* 0:master 1:slave */
	spinlock_t            mb86s70_irq_lock;
	char                 lli_chan_name[10];
	struct clk           *clocks[3];
};

struct lli_dev {
	s16                  lli_drv_ver;
	s16                  cur_lli_major_num;
	s8                   act_lli_ch;
	struct semaphore    buf_sem;
	struct lli_data    *chan_data;
};

struct ioctl_memmap {
	u32 index;
	u32 fromAddr;
	u32 toAddr;
	u32 size;
};

struct ioctl_cfg {
	/*0:read form driver , 1:write to driver */
	u8                direct;
	struct lli_cfg  lli_defcfg;
};

struct ioctl_reg {
	u32    reg_addr;
	u32    reg_data;
};


extern struct lli_dev   *gp_mb86s70_lli_dev;



#define TX_LANE_CNT 1
#define RX_LANE_CNT 1
#define CMP_TIMEOUT msecs_to_jiffies(10)



/* IO_CONTROL_COMMAND */
#define IOCTL_MAG   'P'

#define LLI_IOCTL_M_MOUNT           _IO(IOCTL_MAG, 0) /*M*/
#define LLI_IOCTL_M_UNMOUNT         _IO(IOCTL_MAG, 1) /*M*/
#define LLI_IOCTL_PHY_TEST_START    _IO(IOCTL_MAG, 2) /*M*/
#define LLI_IOCTL_PHY_TEST_STOP     _IO(IOCTL_MAG, 3) /*M*/
#define LLI_IOCTL_S_MOUNT           _IO(IOCTL_MAG, 4) /*S*/
#define LLI_IOCTL_S_MAIN_LOOP       _IO(IOCTL_MAG, 5) /*S*/
#define LLI_IOCTL_SET_LOCAL_CONFIG  _IO(IOCTL_MAG, 6)
#define LLI_IOCTL_CONFIG_UPDATE     _IO(IOCTL_MAG, 7)
#define LLI_IOCTL_CHANGE_MEMMAP     _IO(IOCTL_MAG, 8)
#define LLI_IOCTL_SET_REMOTE_SIG    _IO(IOCTL_MAG, 9)

/* write SOC register */
#define LLI_IOCTL_WRITE_REG _IOWR(IOCTL_MAG, 10, struct ioctl_reg)
/* read SOC register */
#define LLI_IOCTL_READ_REG _IOWR(IOCTL_MAG, 11, struct ioctl_reg)

#define LLI_IOCTL_GET_VERSION _IOWR(IOCTL_MAG, 12, u32)
/* only for test */
#define LLI_IOCTL_MEM_TEST_CREATE _IOWR(IOCTL_MAG, 13, u32)


#define MASTER_MOUNT_SUCCESS        0
#define MASTER_MOUNT_MNTINT_TIMEOUT -1
#define MASTER_MOUNT_NO_CSA_MOUNTED -2
#define MASTER_MOUNT_NO_MNTINT      -3


#define MASTER_UNMOUNT_SUCCESS          0
#define MASTER_UNMOUNT_UNMNTINT_TIMEOUT -1
#define MASTER_UNMOUNT_CSA_MOUNTED      -2
#define MASTER_UNMOUNT_NO_MNTINT        -3


#define SLAVE_MOUNT_SUCCESS         0
#define SLAVE_MOUNT_MNTINT_TIMEOUT  -1
#define SLAVE_MOUNT_NO_CSA_MOUNTED  -2
#define SLAVE_MOUNT_CSA_HIB8EXIT    -3
#define SLAVE_MOUNT_HIB8EXIT_INT_TIME_OUT   -4
#define SLAVE_MOUNT_NO_MNTINT               -5

#define CONFIG_UPDATE_SUCCESS           0
#define CONFIG_UPDATE_PLU_INT_TIMEOUT   -1
#define CONFIG_UPDATE_CSA_LINKUPDATE    -2
#define CONFIG_UPDATE_SHADOW_FAIL       -3
#define CONFIG_UPDATE_NO_PLU_INT        -4

#endif
