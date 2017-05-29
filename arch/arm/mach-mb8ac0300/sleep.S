/*
 * linux/arch/arm/mach-mb8ac0300/sleep.S
 *
 * Copyright (C) 2012-2013 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/linkage.h>
#include <linux/platform_data/mb8ac0300-iomap.h>

#define IOMEM(x) (x)

#define	DDR3_SDRAMC_DENALI_CTL_00		0
#define	DDR3_SDRAMC_DENALI_CTL_23		0x5c
#define	DDR3_SDRAMC_DENALI_CTL_90		0x168
#define	DDR3_SDRAMC_DENALI_CTL_92		0x170
#define	DDR3_SDRAMC_REG_STR			0x400
#define	DDR3_SDRAMC_REG_MCC			0x404
#define	DDR3_SDRAMC_REG_RST			0x408
#define DDR3_SDRAMC_REG_PWR0			0x40c
#define DDR3_SDRAMC_REG_PWR1			0x410
#define	DDR3_SDRAMC_REG_UP			0x440

#define	DDR3_SDRAMC_DENALI_CTL_23_SREFRESH	1
#define	DDR3_SDRAMC_DENALI_CTL_90_CKE_STATUS	1<<16
#define	DDR3_SDRAMC_DENALI_CTL_92_CLK_DISABLE	1<<24
#define	DDR3_SDRAMC_REG_STR_CONTROLLER_BUSY	0x8

	.text
	/*
	 * mb8ac0300_cpu_save
	 * r0 : v:p offset
	 */

ENTRY(mb8ac0300_cpu_save)
	stmfd	sp!, { r4 - r11, lr }
	/*
	 * r1 virtual:physics offset
	 * r3 virtual return function
	 */
	@mov	r1, r0			@ mb8ac0300 don't support dormant mode
	@ldr	r3, =resume_with_mmu
	@bl	cpu_suspend		@ we comment out cpu_suspend

	ldr	r0, =MB8AC0300_DDRC_VIRT
	ldr	r1, =MB8AC0300_XSRAM_VIRT
	ldr	lr, =resume_with_mmu
	mov	pc, r1

	/*
	 * return to the caller, after having the MMU
	 * turned on, this restores the last bits from the stack
	 */
resume_with_mmu:

	ldmfd	sp!, { r4 - r11, lr }
	mov	pc, lr

ENTRY(mb8ac0300_cpu_wakeup)
	b	cpu_resume

	/*
	 * mb8ac0300_cpu_sleep
	 * r0 : DDRC virtual address
	 */
ENTRY(mb8ac0300_cpu_sleep)
l_ddrc_busy:
	ldr	r1, [r0, #DDR3_SDRAMC_REG_STR]
	ands	r2, r1, #DDR3_SDRAMC_REG_STR_CONTROLLER_BUSY
	bne	l_ddrc_busy

@ enter self-refresh mode
	ldr	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_23]
	orr	r2, r2, #DDR3_SDRAMC_DENALI_CTL_23_SREFRESH
	str	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_23]
l_cke_status_is_not_zero:
	ldr	r1, [r0, #DDR3_SDRAMC_DENALI_CTL_90]
	ands	r2, r1, #DDR3_SDRAMC_DENALI_CTL_92_CLK_DISABLE
	bne	l_cke_status_is_not_zero

@ disable DDR clock
	ldr	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_92]
	orr	r2, r2, #DDR3_SDRAMC_DENALI_CTL_92_CLK_DISABLE
	str	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_92]

@ DDR driver suspend
	ldr	r1, [r0, #DDR3_SDRAMC_REG_PWR0]
	ldr	r2, lc_ddrc_pwr0_val
	orr	r1, r1, r2
	str	r1, [r0, #DDR3_SDRAMC_REG_PWR0]

@ I/O driver/receiver suspend
	ldr	r1, [r0, #DDR3_SDRAMC_REG_PWR1]
	ldr	r2, lc_ddrc_pwr1_val
	orr	r1, r1, r2
	str	r1, [r0, #DDR3_SDRAMC_REG_PWR1]

@ into WFI sleep mode
	wfi

@ DDR driver resume
	ldr	r1, [r0, #DDR3_SDRAMC_REG_PWR0]
	ldr	r2, lc_ddrc_pwr0_val
	bic	r1, r1, r2
	str	r1, [r0, #DDR3_SDRAMC_REG_PWR0]

@ I/O driver/receiver resume
	ldr	r1, [r0, #DDR3_SDRAMC_REG_PWR1]
	ldr	r2, lc_ddrc_pwr1_val
	bic	r1, r1, r2
	str	r1, [r0, #DDR3_SDRAMC_REG_PWR1]

@ enable DDR clock
	ldr	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_92]
	bic	r2, r2, #DDR3_SDRAMC_DENALI_CTL_92_CLK_DISABLE
	str	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_92]

@ Reset Sequence
	ldr	r2, lc_reg_rst1
	str	r2, [r0, #DDR3_SDRAMC_REG_RST]

 @ sleep 120ns,3*20=60cycle 120ns
	mov	r2, #20
l_sleep_1:
	nop
	nop
	subs	r2, r2, #1
	bne	l_sleep_1

	ldr	r2, lc_reg_rst2
	str	r2, [r0, #DDR3_SDRAMC_REG_RST]

 @ sleep 66us,3*11k=33kcycle 66us
	ldr	r2, lc_11000
l_sleep_2:
	nop
	nop
	subs	r2, r2, #1
	bne	l_sleep_2

	ldr	r2, lc_reg_rst3
	str	r2, [r0, #DDR3_SDRAMC_REG_RST]

 @ sleep 1500us,3*225k=675kcycle 1.5ms
	ldr	r2, lc_225000
l_sleep_3:
	nop
	nop
	subs	r2, r2, #1
	bne	l_sleep_3

@ PZQ initial calibration
	mov	r2, #0
	str	r2, [r0, #DDR3_SDRAMC_REG_UP]

@ sleep x ns(not strictly defined),3*10=30cycle 60ns
	mov	r2, #10
l_sleep_4:
	nop
	nop
	subs	r2, r2, #1
	bne	l_sleep_4

	mov	r2, #0x1
	str	r2, [r0, #DDR3_SDRAMC_REG_MCC]

@ exit self-refresh mode
	ldr	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_23]
	bic	r2, r2, #DDR3_SDRAMC_DENALI_CTL_23_SREFRESH
	str	r2, [r0, #DDR3_SDRAMC_DENALI_CTL_23]

l_cke_status_is_zero:
	ldr	r1, [r0, #DDR3_SDRAMC_DENALI_CTL_90]
	ands	r2, r1, #DDR3_SDRAMC_DENALI_CTL_90_CKE_STATUS
	beq	l_cke_status_is_zero

	mov	pc, lr

	.align 2
lc_ddrc_pwr0_val:
	.long	0x1ffffff
			@ 0-14:Address(o_MA0-14) driver suspend
			@15-17:Bank Address(o_MBA0-2) driver suspend
			@18   :Write enable(o_MXWE) driver suspend
			@19   :CAS(o_MXCAS) driver suspend
			@20   :RAS(o_MXRAS) driver suspend
			@21-22:CS(o_MXCS0-1) driver suspend
			@23-24:Data(o_MODT0,1) driver suspend
			@25-26:Clock enable(o_MCKE0-1) driver not suspend
lc_ddrc_pwr1_val:
	.long	0x10fff01
			@    0:Clock(o_OTXCK) driver not suspend
			@ 8-15:DQDQS driver suspend
			@16-19:Loop back driver suspend
			@24   :ZQ bias driver suspend
lc_reg_rst1:
	.long	0x0000F
lc_reg_rst2:
	.long	0x3000F
lc_reg_rst3:
	.long	0x7000F
lc_225000:
	.long	225000
lc_11000:
	.long	11000

mb8ac0300_cpu_sleep_size:
	.global	mb8ac0300_cpu_sleep_size
	.long   . - mb8ac0300_cpu_sleep
