/********************************************************************************************
 * arch/arm/src/cxd56xx/chip/cxd5602_topreg.h
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_CHIP_CXD5602_TOPREG_H
#define __ARCH_ARM_SRC_CXD56XX_CHIP_CXD5602_TOPREG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <sdk/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define CXD56_TOPREG_PWD_CTL	(CXD56_TOPREG_BASE + 0x0)
#define CXD56_TOPREG_ANA_PW_CTL	(CXD56_TOPREG_BASE + 0x4)
#define CXD56_TOPREG_ANA_EN_CTL	(CXD56_TOPREG_BASE + 0x8)
#define CXD56_TOPREG_SYSCPU_RAMMODE_SEL	(CXD56_TOPREG_BASE + 0x10)
#define CXD56_TOPREG_TOP_SCU_RAMMODE_SEL	(CXD56_TOPREG_BASE + 0x18)
#define CXD56_TOPREG_HOSTIFC_RAMMODE_SEL	(CXD56_TOPREG_BASE + 0x1c)
#define CXD56_TOPREG_PMU_FAST	(CXD56_TOPREG_BASE + 0x20)
#define CXD56_TOPREG_PMU_PW_CTL	(CXD56_TOPREG_BASE + 0x30)
#define CXD56_TOPREG_PMU_INT_STAT	(CXD56_TOPREG_BASE + 0x40)
#define CXD56_TOPREG_PMU_RAW_INT_STAT	(CXD56_TOPREG_BASE + 0x44)
#define CXD56_TOPREG_PMU_INT_CLR	(CXD56_TOPREG_BASE + 0x48)
#define CXD56_TOPREG_PMU_INT_MASK	(CXD56_TOPREG_BASE + 0x4c)
#define CXD56_TOPREG_PWD_RESET0	(CXD56_TOPREG_BASE + 0x60)
#define CXD56_TOPREG_PMU_DBG	(CXD56_TOPREG_BASE + 0x70)
#define CXD56_TOPREG_PMU_TIMEOUT_CTL0	(CXD56_TOPREG_BASE + 0x74)
#define CXD56_TOPREG_PMU_TIMEOUT_CTL1	(CXD56_TOPREG_BASE + 0x78)
#define CXD56_TOPREG_PMU_TIMEOUT_CTL2	(CXD56_TOPREG_BASE + 0x7c)
#define CXD56_TOPREG_PMU_FSM	(CXD56_TOPREG_BASE + 0x80)
#define CXD56_TOPREG_PMU_PW_STAT	(CXD56_TOPREG_BASE + 0x84)
#define CXD56_TOPREG_PMU_WAIT0	(CXD56_TOPREG_BASE + 0x88)
#define CXD56_TOPREG_PMU_WAIT1	(CXD56_TOPREG_BASE + 0x8c)
#define CXD56_TOPREG_PMU_WAIT2	(CXD56_TOPREG_BASE + 0x90)
#define CXD56_TOPREG_PMU_WAIT3	(CXD56_TOPREG_BASE + 0x94)
#define CXD56_TOPREG_PMU_WAIT4	(CXD56_TOPREG_BASE + 0x98)
#define CXD56_TOPREG_PMU_WAIT5	(CXD56_TOPREG_BASE + 0x9c)
#define CXD56_TOPREG_PMU_WAIT6	(CXD56_TOPREG_BASE + 0xa0)
#define CXD56_TOPREG_PMU_WAIT7	(CXD56_TOPREG_BASE + 0xa4)
#define CXD56_TOPREG_PMU_WAIT8	(CXD56_TOPREG_BASE + 0xa8)
#define CXD56_TOPREG_PMU_WAIT9	(CXD56_TOPREG_BASE + 0xac)
#define CXD56_TOPREG_PMU_DBG_INITEN	(CXD56_TOPREG_BASE + 0xb0)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER0	(CXD56_TOPREG_BASE + 0xb4)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER1	(CXD56_TOPREG_BASE + 0xb8)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER2	(CXD56_TOPREG_BASE + 0xbc)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER3	(CXD56_TOPREG_BASE + 0xc0)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER4	(CXD56_TOPREG_BASE + 0xc4)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER5	(CXD56_TOPREG_BASE + 0xc8)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER6	(CXD56_TOPREG_BASE + 0xcc)
#define CXD56_TOPREG_PMU_DBG_ON_ORDER7	(CXD56_TOPREG_BASE + 0xd0)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER0	(CXD56_TOPREG_BASE + 0xd4)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER1	(CXD56_TOPREG_BASE + 0xd8)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER2	(CXD56_TOPREG_BASE + 0xdc)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER3	(CXD56_TOPREG_BASE + 0xe0)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER4	(CXD56_TOPREG_BASE + 0xe4)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER5	(CXD56_TOPREG_BASE + 0xe8)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER6	(CXD56_TOPREG_BASE + 0xec)
#define CXD56_TOPREG_PMU_DBG_OFF_ORDER7	(CXD56_TOPREG_BASE + 0xf0)
#define CXD56_TOPREG_PMU_DBG_LUMPEN	(CXD56_TOPREG_BASE + 0xf4)
#define CXD56_TOPREG_PMU_DBG_ADD	(CXD56_TOPREG_BASE + 0xfc)
#define CXD56_TOPREG_PMU_DBG_ADD_WAIT0	(CXD56_TOPREG_BASE + 0x100)
#define CXD56_TOPREG_PMU_DBG_ADD_WAIT1	(CXD56_TOPREG_BASE + 0x104)
#define CXD56_TOPREG_PWD_STAT	(CXD56_TOPREG_BASE + 0x200)
#define CXD56_TOPREG_PWD_PGACK_STAT	(CXD56_TOPREG_BASE + 0x204)
#define CXD56_TOPREG_ANA_PW_STAT	(CXD56_TOPREG_BASE + 0x208)
#define CXD56_TOPREG_SYSCPU_RAMMODE_STAT	(CXD56_TOPREG_BASE + 0x20c)
#define CXD56_TOPREG_TOP_SCU_RAMMODE_STAT	(CXD56_TOPREG_BASE + 0x214)
#define CXD56_TOPREG_HOSTIFC_RAMMODE_STAT	(CXD56_TOPREG_BASE + 0x218)
#define CXD56_TOPREG_PMU_HW_STAT	(CXD56_TOPREG_BASE + 0x21c)
#define CXD56_TOPREG_YOBI2_0	(CXD56_TOPREG_BASE + 0x230)
#define CXD56_TOPREG_ANA_PW_CTL_SEL_WAKE	(CXD56_TOPREG_BASE + 0x400)
#define CXD56_TOPREG_SYSCPU_RAMMODE_SEL_WAKE	(CXD56_TOPREG_BASE + 0x404)
#define CXD56_TOPREG_TOP_SCU_RAMMODE_SEL_WAKE	(CXD56_TOPREG_BASE + 0x410)
#define CXD56_TOPREG_CLSELDIV_WAKE	(CXD56_TOPREG_BASE + 0x414)
#define CXD56_TOPREG_CKDIV_CPU_DSP_BUS_WAKE	(CXD56_TOPREG_BASE + 0x418)
#define CXD56_TOPREG_CKSEL_ROOT_WAKE	(CXD56_TOPREG_BASE + 0x41c)
#define CXD56_TOPREG_PMIC_SLEEP_I2C0	(CXD56_TOPREG_BASE + 0x420)
#define CXD56_TOPREG_PMIC_SLEEP_I2C1	(CXD56_TOPREG_BASE + 0x424)
#define CXD56_TOPREG_PMIC_SLEEP_I2C2	(CXD56_TOPREG_BASE + 0x428)
#define CXD56_TOPREG_PMIC_SLEEP_I2C3	(CXD56_TOPREG_BASE + 0x42c)
#define CXD56_TOPREG_PMIC_WAKE_I2C0	(CXD56_TOPREG_BASE + 0x430)
#define CXD56_TOPREG_PMIC_WAKE_I2C1	(CXD56_TOPREG_BASE + 0x434)
#define CXD56_TOPREG_PMIC_WAKE_I2C2	(CXD56_TOPREG_BASE + 0x438)
#define CXD56_TOPREG_PMIC_WAKE_I2C3	(CXD56_TOPREG_BASE + 0x43c)
#define CXD56_TOPREG_PMIC_UNEXP_I2C0	(CXD56_TOPREG_BASE + 0x440)
#define CXD56_TOPREG_PMIC_UNEXP_I2C1	(CXD56_TOPREG_BASE + 0x444)
#define CXD56_TOPREG_PMIC_UNEXP_I2C2	(CXD56_TOPREG_BASE + 0x448)
#define CXD56_TOPREG_PMIC_UNEXP_I2C3	(CXD56_TOPREG_BASE + 0x44c)
#define CXD56_TOPREG_PMIC_UNEXP_I2C	(CXD56_TOPREG_BASE + 0x450)
#define CXD56_TOPREG_PMU_WAKE_TRIG_EN0	(CXD56_TOPREG_BASE + 0x454)
#define CXD56_TOPREG_PMU_WAKE_TRIG_EN1	(CXD56_TOPREG_BASE + 0x458)
#define CXD56_TOPREG_PMU_WAKE_TRIG_NEGEN0	(CXD56_TOPREG_BASE + 0x45c)
#define CXD56_TOPREG_PMU_WAKE_TRIG_NEGEN1	(CXD56_TOPREG_BASE + 0x460)
#define CXD56_TOPREG_PMU_WAKE_TRIG_NOISECUTEN0	(CXD56_TOPREG_BASE + 0x464)
#define CXD56_TOPREG_PMU_WAKE_TRIG_CPUINTSEL0	(CXD56_TOPREG_BASE + 0x468)
#define CXD56_TOPREG_PMU_WAKE_TRIG_CPUINTSEL1	(CXD56_TOPREG_BASE + 0x46c)
#define CXD56_TOPREG_PMU_WAKE_TRIG_CPUINTSEL2	(CXD56_TOPREG_BASE + 0x470)
#define CXD56_TOPREG_PMU_WAKE_TRIG_INTDET0	(CXD56_TOPREG_BASE + 0x474)
#define CXD56_TOPREG_PMU_WAKE_TRIG_INTDET1	(CXD56_TOPREG_BASE + 0x478)
#define CXD56_TOPREG_PMU_WAKE_TRIG_INTDET2	(CXD56_TOPREG_BASE + 0x47c)
#define CXD56_TOPREG_PMU_WAKE_PMIC_I2C	(CXD56_TOPREG_BASE + 0x480)
#define CXD56_TOPREG_BOOT_CAUSE	(CXD56_TOPREG_BASE + 0x484)
#define CXD56_TOPREG_PMU_CORE_CKEN	(CXD56_TOPREG_BASE + 0x4c0)
#define CXD56_TOPREG_CKSEL_ROOT	(CXD56_TOPREG_BASE + 0x4c4)
#define CXD56_TOPREG_CKSEL_PMU	(CXD56_TOPREG_BASE + 0x4c8)
#define CXD56_TOPREG_CKSEL_SYSIOP	(CXD56_TOPREG_BASE + 0x4cc)
#define CXD56_TOPREG_CKSEL_SYSIOP_SUB	(CXD56_TOPREG_BASE + 0x4d0)
#define CXD56_TOPREG_CKSEL_SCU	(CXD56_TOPREG_BASE + 0x4d4)
#define CXD56_TOPREG_CKDIV_CPU_DSP_BUS	(CXD56_TOPREG_BASE + 0x4d8)
#define CXD56_TOPREG_CKDIV_COM	(CXD56_TOPREG_BASE + 0x4dc)
#define CXD56_TOPREG_CKDIV_HOSTIFC	(CXD56_TOPREG_BASE + 0x4e0)
#define CXD56_TOPREG_CKDIV_SCU	(CXD56_TOPREG_BASE + 0x4e4)
#define CXD56_TOPREG_CKDIV_PMU	(CXD56_TOPREG_BASE + 0x4e8)
#define CXD56_TOPREG_CRG_INT_CLR0	(CXD56_TOPREG_BASE + 0x4ec)
#define CXD56_TOPREG_CRG_INT_MASK0	(CXD56_TOPREG_BASE + 0x4f0)
#define CXD56_TOPREG_CRG_INT_STAT_MSK0	(CXD56_TOPREG_BASE + 0x4f4)
#define CXD56_TOPREG_CRG_INT_STAT_RAW0	(CXD56_TOPREG_BASE + 0x4f8)
#define CXD56_TOPREG_CRG_INT_CLR1	(CXD56_TOPREG_BASE + 0x4fc)
#define CXD56_TOPREG_CRG_INT_MASK1	(CXD56_TOPREG_BASE + 0x500)
#define CXD56_TOPREG_CRG_INT_STAT_MSK1	(CXD56_TOPREG_BASE + 0x504)
#define CXD56_TOPREG_CRG_INT_STAT_RAW1	(CXD56_TOPREG_BASE + 0x508)
#define CXD56_TOPREG_CPU_GATECLK	(CXD56_TOPREG_BASE + 0x50c)
#define CXD56_TOPREG_USBPHY_CKEN	(CXD56_TOPREG_BASE + 0x510)
#define CXD56_TOPREG_CRG_MON	(CXD56_TOPREG_BASE + 0x514)
#define CXD56_TOPREG_GEAR_STAT	(CXD56_TOPREG_BASE + 0x518)
#define CXD56_TOPREG_XOSC_CTRL	(CXD56_TOPREG_BASE + 0x580)
#define CXD56_TOPREG_XOSC_CTRL2	(CXD56_TOPREG_BASE + 0x584)
#define CXD56_TOPREG_SYS_PLL_CTRL1	(CXD56_TOPREG_BASE + 0x588)
#define CXD56_TOPREG_SYS_PLL_CTRL2	(CXD56_TOPREG_BASE + 0x58c)
#define CXD56_TOPREG_RCOSC_CTRL1	(CXD56_TOPREG_BASE + 0x590)
#define CXD56_TOPREG_RCOSC_CTRL2	(CXD56_TOPREG_BASE + 0x594)
#define CXD56_TOPREG_RF_GPMBI_EN	(CXD56_TOPREG_BASE + 0x598)
#define CXD56_TOPREG_BUSPROT_SDMAC	(CXD56_TOPREG_BASE + 0x5c0)
#define CXD56_TOPREG_BUSPROT_HDMAC	(CXD56_TOPREG_BASE + 0x5c4)
#define CXD56_TOPREG_BUSPROT_SYDMAC	(CXD56_TOPREG_BASE + 0x5c8)
#define CXD56_TOPREG_BUSPROT_SYSUBDMAC	(CXD56_TOPREG_BASE + 0x5cc)
#define CXD56_TOPREG_BUSPROT_SAKE	(CXD56_TOPREG_BASE + 0x5d0)
#define CXD56_TOPREG_BUSPROT_KAKI	(CXD56_TOPREG_BASE + 0x5d4)
#define CXD56_TOPREG_BUSPROT_BKUPSRAM	(CXD56_TOPREG_BASE + 0x5d8)
#define CXD56_TOPREG_BUSPROT_SPIFLAIF	(CXD56_TOPREG_BASE + 0x5dc)
#define CXD56_TOPREG_BUSPROT_TOPREG_0	(CXD56_TOPREG_BASE + 0x5e0)
#define CXD56_TOPREG_BUSPROT_TOPREG_1	(CXD56_TOPREG_BASE + 0x5e4)
#define CXD56_TOPREG_BUSPROT_TOPREG_2	(CXD56_TOPREG_BASE + 0x5e8)
#define CXD56_TOPREG_BUSPROT_TOPREG_3	(CXD56_TOPREG_BASE + 0x5ec)
#define CXD56_TOPREG_BUSPROT_TOPREG_4	(CXD56_TOPREG_BASE + 0x5f0)
#define CXD56_TOPREG_GPIO_PROT_0	(CXD56_TOPREG_BASE + 0x5f4)
#define CXD56_TOPREG_GPIO_PROT_1	(CXD56_TOPREG_BASE + 0x5f8)
#define CXD56_TOPREG_GPIO_PROT_2	(CXD56_TOPREG_BASE + 0x5fc)
#define CXD56_TOPREG_GPIO_PROT_3	(CXD56_TOPREG_BASE + 0x600)
#define CXD56_TOPREG_BUSPROT_CHECKER	(CXD56_TOPREG_BASE + 0x610)
#define CXD56_TOPREG_WDT_SRST_EN	(CXD56_TOPREG_BASE + 0x640)
#define CXD56_TOPREG_FORCE_CKEN	(CXD56_TOPREG_BASE + 0x644)
#define CXD56_TOPREG_SDEBUG_PASS_BYPASS	(CXD56_TOPREG_BASE + 0x648)
#define CXD56_TOPREG_SDEBUG_CTRL	(CXD56_TOPREG_BASE + 0x64c)
#define CXD56_TOPREG_DBG_HOSTIF_SEL	(CXD56_TOPREG_BASE + 0x650)
#define CXD56_TOPREG_WDT_MASK	(CXD56_TOPREG_BASE + 0x654)
#define CXD56_TOPREG_CKGATE_CTL	(CXD56_TOPREG_BASE + 0x660)
#define CXD56_TOPREG_M0_BOOT_MODE	(CXD56_TOPREG_BASE + 0x680)
#define CXD56_TOPREG_M0_BOOT_MODE_EN	(CXD56_TOPREG_BASE + 0x684)
#define CXD56_TOPREG_M0_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0x688)
#define CXD56_TOPREG_M0_BOOT_MODE_EXT	(CXD56_TOPREG_BASE + 0x690)
#define CXD56_TOPREG_M0_BOOT_REC	(CXD56_TOPREG_BASE + 0x694)
#define CXD56_TOPREG_I2CRPT_SADR	(CXD56_TOPREG_BASE + 0x6c0)
#define CXD56_TOPREG_I2CRPT_REP	(CXD56_TOPREG_BASE + 0x6c4)
#define CXD56_TOPREG_PMIC_I2C	(CXD56_TOPREG_BASE + 0x6d0)
#define CXD56_TOPREG_RAMMODE	(CXD56_TOPREG_BASE + 0x6e4)
#define CXD56_TOPREG_SWRESET_BUS	(CXD56_TOPREG_BASE + 0x700)
#define CXD56_TOPREG_SWRESET_SCU	(CXD56_TOPREG_BASE + 0x704)
#define CXD56_TOPREG_BUSROM_CKEN	(CXD56_TOPREG_BASE + 0x710)
#define CXD56_TOPREG_SYSIOP_CKEN	(CXD56_TOPREG_BASE + 0x714)
#define CXD56_TOPREG_SCU_CKEN	(CXD56_TOPREG_BASE + 0x71c)
#define CXD56_TOPREG_RTC0_CTL	(CXD56_TOPREG_BASE + 0x730)
#define CXD56_TOPREG_FUSERDCFG0	(CXD56_TOPREG_BASE + 0x740)
#define CXD56_TOPREG_FUSERDCFG1	(CXD56_TOPREG_BASE + 0x744)
#define CXD56_TOPREG_FUSERDCFG2	(CXD56_TOPREG_BASE + 0x748)
#define CXD56_TOPREG_VID0	(CXD56_TOPREG_BASE + 0x750)
#define CXD56_TOPREG_VID1	(CXD56_TOPREG_BASE + 0x754)
#define CXD56_TOPREG_M0_BOOT_FLASH_DIS	(CXD56_TOPREG_BASE + 0x758)
#define CXD56_TOPREG_LDOADJ0	(CXD56_TOPREG_BASE + 0x760)
#define CXD56_TOPREG_LDOADJ1	(CXD56_TOPREG_BASE + 0x764)
#define CXD56_TOPREG_FQFIX_CTL0	(CXD56_TOPREG_BASE + 0x770)
#define CXD56_TOPREG_FQFIX_AUTO	(CXD56_TOPREG_BASE + 0x774)
#define CXD56_TOPREG_FQFIX_SINGLE	(CXD56_TOPREG_BASE + 0x778)
#define CXD56_TOPREG_FQFIX_STATUS	(CXD56_TOPREG_BASE + 0x77c)
#define CXD56_TOPREG_SYSTEM_CONFIG	(CXD56_TOPREG_BASE + 0x790)
#define CXD56_TOPREG_MON_SEL	(CXD56_TOPREG_BASE + 0x7a0)
#define CXD56_TOPREG_IOCSYS_MONSEL0	(CXD56_TOPREG_BASE + 0x7a4)
#define CXD56_TOPREG_IOCSYS_MONSEL1	(CXD56_TOPREG_BASE + 0x7a8)
#define CXD56_TOPREG_IOCSYS_INTSEL0	(CXD56_TOPREG_BASE + 0x7b0)
#define CXD56_TOPREG_IOCSYS_INTSEL1	(CXD56_TOPREG_BASE + 0x7b4)
#define CXD56_TOPREG_IOCSYS_IOMD0	(CXD56_TOPREG_BASE + 0x7c0)
#define CXD56_TOPREG_IOCSYS_IOMD1	(CXD56_TOPREG_BASE + 0x7c4)
#define CXD56_TOPREG_IOOEN_SYS	(CXD56_TOPREG_BASE + 0x7e0)
#define CXD56_TOPREG_IO_RTC_CLK_IN	(CXD56_TOPREG_BASE + 0x800)
#define CXD56_TOPREG_IO_I2C4_BCK	(CXD56_TOPREG_BASE + 0x804)
#define CXD56_TOPREG_IO_I2C4_BDT	(CXD56_TOPREG_BASE + 0x808)
#define CXD56_TOPREG_IO_PMIC_INT	(CXD56_TOPREG_BASE + 0x80c)
#define CXD56_TOPREG_IO_RTC_IRQ_OUT	(CXD56_TOPREG_BASE + 0x810)
#define CXD56_TOPREG_IO_AP_CLK	(CXD56_TOPREG_BASE + 0x814)
#define CXD56_TOPREG_IO_GNSS_1PPS_OUT	(CXD56_TOPREG_BASE + 0x818)
#define CXD56_TOPREG_IO_SPI0_CS_X	(CXD56_TOPREG_BASE + 0x844)
#define CXD56_TOPREG_IO_SPI0_SCK	(CXD56_TOPREG_BASE + 0x848)
#define CXD56_TOPREG_IO_SPI0_MOSI	(CXD56_TOPREG_BASE + 0x84c)
#define CXD56_TOPREG_IO_SPI0_MISO	(CXD56_TOPREG_BASE + 0x850)
#define CXD56_TOPREG_IO_SPI1_CS_X	(CXD56_TOPREG_BASE + 0x854)
#define CXD56_TOPREG_IO_SPI1_SCK	(CXD56_TOPREG_BASE + 0x858)
#define CXD56_TOPREG_IO_SPI1_IO0	(CXD56_TOPREG_BASE + 0x85c)
#define CXD56_TOPREG_IO_SPI1_IO1	(CXD56_TOPREG_BASE + 0x860)
#define CXD56_TOPREG_IO_SPI1_IO2	(CXD56_TOPREG_BASE + 0x864)
#define CXD56_TOPREG_IO_SPI1_IO3	(CXD56_TOPREG_BASE + 0x868)
#define CXD56_TOPREG_IO_SPI2_CS_X	(CXD56_TOPREG_BASE + 0x86c)
#define CXD56_TOPREG_IO_SPI2_SCK	(CXD56_TOPREG_BASE + 0x870)
#define CXD56_TOPREG_IO_SPI2_MOSI	(CXD56_TOPREG_BASE + 0x874)
#define CXD56_TOPREG_IO_SPI2_MISO	(CXD56_TOPREG_BASE + 0x878)
#define CXD56_TOPREG_IO_HIF_IRQ_OUT	(CXD56_TOPREG_BASE + 0x87c)
#define CXD56_TOPREG_IO_HIF_GPIO0	(CXD56_TOPREG_BASE + 0x880)
#define CXD56_TOPREG_IO_SEN_IRQ_IN	(CXD56_TOPREG_BASE + 0x894)
#define CXD56_TOPREG_IO_SPI3_CS0_X	(CXD56_TOPREG_BASE + 0x898)
#define CXD56_TOPREG_IO_SPI3_CS1_X	(CXD56_TOPREG_BASE + 0x89c)
#define CXD56_TOPREG_IO_SPI3_CS2_X	(CXD56_TOPREG_BASE + 0x8a0)
#define CXD56_TOPREG_IO_SPI3_SCK	(CXD56_TOPREG_BASE + 0x8a4)
#define CXD56_TOPREG_IO_SPI3_MOSI	(CXD56_TOPREG_BASE + 0x8a8)
#define CXD56_TOPREG_IO_SPI3_MISO	(CXD56_TOPREG_BASE + 0x8ac)
#define CXD56_TOPREG_IO_I2C0_BCK	(CXD56_TOPREG_BASE + 0x8b0)
#define CXD56_TOPREG_IO_I2C0_BDT	(CXD56_TOPREG_BASE + 0x8b4)
#define CXD56_TOPREG_IO_PWM0	(CXD56_TOPREG_BASE + 0x8b8)
#define CXD56_TOPREG_IO_PWM1	(CXD56_TOPREG_BASE + 0x8bc)
#define CXD56_TOPREG_IO_PWM2	(CXD56_TOPREG_BASE + 0x8c0)
#define CXD56_TOPREG_IO_PWM3	(CXD56_TOPREG_BASE + 0x8c4)
#define CXD56_TOPREG_IO_DBG_SWOCLK	(CXD56_TOPREG_BASE + 0x8d4)
#define CXD56_TOPREG_IO_DBG_SWO	(CXD56_TOPREG_BASE + 0x8d8)
#define CXD56_TOPREG_IO_IS_CLK	(CXD56_TOPREG_BASE + 0x8e0)
#define CXD56_TOPREG_IO_IS_VSYNC	(CXD56_TOPREG_BASE + 0x8e4)
#define CXD56_TOPREG_IO_IS_HSYNC	(CXD56_TOPREG_BASE + 0x8e8)
#define CXD56_TOPREG_IO_IS_DATA0	(CXD56_TOPREG_BASE + 0x8ec)
#define CXD56_TOPREG_IO_IS_DATA1	(CXD56_TOPREG_BASE + 0x8f0)
#define CXD56_TOPREG_IO_IS_DATA2	(CXD56_TOPREG_BASE + 0x8f4)
#define CXD56_TOPREG_IO_IS_DATA3	(CXD56_TOPREG_BASE + 0x8f8)
#define CXD56_TOPREG_IO_IS_DATA4	(CXD56_TOPREG_BASE + 0x8fc)
#define CXD56_TOPREG_IO_IS_DATA5	(CXD56_TOPREG_BASE + 0x900)
#define CXD56_TOPREG_IO_IS_DATA6	(CXD56_TOPREG_BASE + 0x904)
#define CXD56_TOPREG_IO_IS_DATA7	(CXD56_TOPREG_BASE + 0x908)
#define CXD56_TOPREG_IO_UART2_TXD	(CXD56_TOPREG_BASE + 0x90c)
#define CXD56_TOPREG_IO_UART2_RXD	(CXD56_TOPREG_BASE + 0x910)
#define CXD56_TOPREG_IO_UART2_CTS	(CXD56_TOPREG_BASE + 0x914)
#define CXD56_TOPREG_IO_UART2_RTS	(CXD56_TOPREG_BASE + 0x918)
#define CXD56_TOPREG_IO_SPI4_CS_X	(CXD56_TOPREG_BASE + 0x91c)
#define CXD56_TOPREG_IO_SPI4_SCK	(CXD56_TOPREG_BASE + 0x920)
#define CXD56_TOPREG_IO_SPI4_MOSI	(CXD56_TOPREG_BASE + 0x924)
#define CXD56_TOPREG_IO_SPI4_MISO	(CXD56_TOPREG_BASE + 0x928)
#define CXD56_TOPREG_IO_EMMC_CLK	(CXD56_TOPREG_BASE + 0x92c)
#define CXD56_TOPREG_IO_EMMC_CMD	(CXD56_TOPREG_BASE + 0x930)
#define CXD56_TOPREG_IO_EMMC_DATA0	(CXD56_TOPREG_BASE + 0x934)
#define CXD56_TOPREG_IO_EMMC_DATA1	(CXD56_TOPREG_BASE + 0x938)
#define CXD56_TOPREG_IO_EMMC_DATA2	(CXD56_TOPREG_BASE + 0x93c)
#define CXD56_TOPREG_IO_EMMC_DATA3	(CXD56_TOPREG_BASE + 0x940)
#define CXD56_TOPREG_IO_SDIO_CLK	(CXD56_TOPREG_BASE + 0x944)
#define CXD56_TOPREG_IO_SDIO_CMD	(CXD56_TOPREG_BASE + 0x948)
#define CXD56_TOPREG_IO_SDIO_DATA0	(CXD56_TOPREG_BASE + 0x94c)
#define CXD56_TOPREG_IO_SDIO_DATA1	(CXD56_TOPREG_BASE + 0x950)
#define CXD56_TOPREG_IO_SDIO_DATA2	(CXD56_TOPREG_BASE + 0x954)
#define CXD56_TOPREG_IO_SDIO_DATA3	(CXD56_TOPREG_BASE + 0x958)
#define CXD56_TOPREG_IO_SDIO_CD	(CXD56_TOPREG_BASE + 0x95c)
#define CXD56_TOPREG_IO_SDIO_WP	(CXD56_TOPREG_BASE + 0x960)
#define CXD56_TOPREG_IO_SDIO_CMDDIR	(CXD56_TOPREG_BASE + 0x964)
#define CXD56_TOPREG_IO_SDIO_DIR0	(CXD56_TOPREG_BASE + 0x968)
#define CXD56_TOPREG_IO_SDIO_DIR1_3	(CXD56_TOPREG_BASE + 0x96c)
#define CXD56_TOPREG_IO_SDIO_CLKI	(CXD56_TOPREG_BASE + 0x970)
#define CXD56_TOPREG_IO_I2S0_BCK	(CXD56_TOPREG_BASE + 0x974)
#define CXD56_TOPREG_IO_I2S0_LRCK	(CXD56_TOPREG_BASE + 0x978)
#define CXD56_TOPREG_IO_I2S0_DATA_IN	(CXD56_TOPREG_BASE + 0x97c)
#define CXD56_TOPREG_IO_I2S0_DATA_OUT	(CXD56_TOPREG_BASE + 0x980)
#define CXD56_TOPREG_IO_I2S1_BCK	(CXD56_TOPREG_BASE + 0x984)
#define CXD56_TOPREG_IO_I2S1_LRCK	(CXD56_TOPREG_BASE + 0x988)
#define CXD56_TOPREG_IO_I2S1_DATA_IN	(CXD56_TOPREG_BASE + 0x98c)
#define CXD56_TOPREG_IO_I2S1_DATA_OUT	(CXD56_TOPREG_BASE + 0x990)
#define CXD56_TOPREG_IO_MCLK	(CXD56_TOPREG_BASE + 0x994)
#define CXD56_TOPREG_IO_PDM_CLK	(CXD56_TOPREG_BASE + 0x998)
#define CXD56_TOPREG_IO_PDM_IN	(CXD56_TOPREG_BASE + 0x99c)
#define CXD56_TOPREG_IO_PDM_OUT	(CXD56_TOPREG_BASE + 0x9a0)
#define CXD56_TOPREG_IO_USB_VBUSINT	(CXD56_TOPREG_BASE + 0x9a4)
#define CXD56_TOPREG_FUSEWRST	(CXD56_TOPREG_BASE + 0xa00)
#define CXD56_TOPREG_FUSEWRAD	(CXD56_TOPREG_BASE + 0xa04)
#define CXD56_TOPREG_FUSEWRDT	(CXD56_TOPREG_BASE + 0xa08)
#define CXD56_TOPREG_FUSEWRPG	(CXD56_TOPREG_BASE + 0xa0c)
#define CXD56_TOPREG_YOBI2_1	(CXD56_TOPREG_BASE + 0xb00)
#define CXD56_TOPREG_GNSSDSP_RAMMODE_SEL	(CXD56_TOPREG_BASE + 0xc00)
#define CXD56_TOPREG_CKSEL_GNSS_BB	(CXD56_TOPREG_BASE + 0xc04)
#define CXD56_TOPREG_CKDIV_ITP	(CXD56_TOPREG_BASE + 0xc0c)
#define CXD56_TOPREG_GNS_ITP_CKEN	(CXD56_TOPREG_BASE + 0xc10)
#define CXD56_TOPREG_RF_CTRL	(CXD56_TOPREG_BASE + 0xc20)
#define CXD56_TOPREG_GDSP_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0xc30)
#define CXD56_TOPREG_GNSSDSP_RAMMODE_STAT	(CXD56_TOPREG_BASE + 0xc40)
#define CXD56_TOPREG_LOGGERIF	(CXD56_TOPREG_BASE + 0xc50)
#define CXD56_TOPREG_YOBI2_2	(CXD56_TOPREG_BASE + 0xc60)
#define CXD56_TOPREG_ADSP1_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0x1010)
#define CXD56_TOPREG_ADSP2_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0x1014)
#define CXD56_TOPREG_ADSP3_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0x1018)
#define CXD56_TOPREG_ADSP4_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0x101c)
#define CXD56_TOPREG_ADSP5_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0x1020)
#define CXD56_TOPREG_YOBI2_3	(CXD56_TOPREG_BASE + 0x1040)
#define CXD56_TOPREG_ADSP0_BOOT_ENTRY_POINT	(CXD56_TOPREG_BASE + 0x1400)
#define CXD56_TOPREG_USB_VBUS	(CXD56_TOPREG_BASE + 0x1410)
#define CXD56_TOPREG_FUSERD00	(CXD56_TOPREG_BASE + 0x1420)
#define CXD56_TOPREG_FUSERD01	(CXD56_TOPREG_BASE + 0x1424)
#define CXD56_TOPREG_FUSERD02	(CXD56_TOPREG_BASE + 0x1428)
#define CXD56_TOPREG_FUSERD03	(CXD56_TOPREG_BASE + 0x142c)
#define CXD56_TOPREG_FUSERD04	(CXD56_TOPREG_BASE + 0x1430)
#define CXD56_TOPREG_FUSERD05	(CXD56_TOPREG_BASE + 0x1434)
#define CXD56_TOPREG_FUSERD06	(CXD56_TOPREG_BASE + 0x1438)
#define CXD56_TOPREG_FUSERD07	(CXD56_TOPREG_BASE + 0x143c)
#define CXD56_TOPREG_FUSERD08	(CXD56_TOPREG_BASE + 0x1440)
#define CXD56_TOPREG_FUSERD09	(CXD56_TOPREG_BASE + 0x1444)
#define CXD56_TOPREG_FUSERD10	(CXD56_TOPREG_BASE + 0x1448)
#define CXD56_TOPREG_FUSERD11	(CXD56_TOPREG_BASE + 0x144c)
#define CXD56_TOPREG_FUSERD12	(CXD56_TOPREG_BASE + 0x1450)
#define CXD56_TOPREG_FUSERD13	(CXD56_TOPREG_BASE + 0x1454)
#define CXD56_TOPREG_FUSERD14	(CXD56_TOPREG_BASE + 0x1458)
#define CXD56_TOPREG_FUSERD15	(CXD56_TOPREG_BASE + 0x145c)
#define CXD56_TOPREG_AUDIO_IF_SEL	(CXD56_TOPREG_BASE + 0x1470)
#define CXD56_TOPREG_IOOEN_APP	(CXD56_TOPREG_BASE + 0x1474)
#define CXD56_TOPREG_IOFIX_APP	(CXD56_TOPREG_BASE + 0x1478)
#define CXD56_TOPREG_IOCAPP_MONSEL0	(CXD56_TOPREG_BASE + 0x1480)
#define CXD56_TOPREG_IOCAPP_MONSEL1	(CXD56_TOPREG_BASE + 0x1484)
#define CXD56_TOPREG_IOCAPP_INTSEL0	(CXD56_TOPREG_BASE + 0x1490)
#define CXD56_TOPREG_IOCAPP_INTSEL1	(CXD56_TOPREG_BASE + 0x1494)
#define CXD56_TOPREG_IOCAPP_IOMD	(CXD56_TOPREG_BASE + 0x14a0)
#define CXD56_TOPREG_YOBI2_4	(CXD56_TOPREG_BASE + 0x14c0)
#define CXD56_TOPREG_GP_I2C4_BCK	(CXD56_TOPREG_BASE + 0x2000)
#define CXD56_TOPREG_GP_I2C4_BDT	(CXD56_TOPREG_BASE + 0x2004)
#define CXD56_TOPREG_GP_PMIC_INT	(CXD56_TOPREG_BASE + 0x2008)
#define CXD56_TOPREG_GP_RTC_IRQ_OUT	(CXD56_TOPREG_BASE + 0x200c)
#define CXD56_TOPREG_GP_AP_CLK	(CXD56_TOPREG_BASE + 0x2010)
#define CXD56_TOPREG_GP_GNSS_1PPS_OUT	(CXD56_TOPREG_BASE + 0x2014)
#define CXD56_TOPREG_GP_SPI0_CS_X	(CXD56_TOPREG_BASE + 0x2040)
#define CXD56_TOPREG_GP_SPI0_SCK	(CXD56_TOPREG_BASE + 0x2044)
#define CXD56_TOPREG_GP_SPI0_MOSI	(CXD56_TOPREG_BASE + 0x2048)
#define CXD56_TOPREG_GP_SPI0_MISO	(CXD56_TOPREG_BASE + 0x204c)
#define CXD56_TOPREG_GP_SPI1_CS_X	(CXD56_TOPREG_BASE + 0x2050)
#define CXD56_TOPREG_GP_SPI1_SCK	(CXD56_TOPREG_BASE + 0x2054)
#define CXD56_TOPREG_GP_SPI1_IO0	(CXD56_TOPREG_BASE + 0x2058)
#define CXD56_TOPREG_GP_SPI1_IO1	(CXD56_TOPREG_BASE + 0x205c)
#define CXD56_TOPREG_GP_SPI1_IO2	(CXD56_TOPREG_BASE + 0x2060)
#define CXD56_TOPREG_GP_SPI1_IO3	(CXD56_TOPREG_BASE + 0x2064)
#define CXD56_TOPREG_GP_SPI2_CS_X	(CXD56_TOPREG_BASE + 0x2068)
#define CXD56_TOPREG_GP_SPI2_SCK	(CXD56_TOPREG_BASE + 0x206c)
#define CXD56_TOPREG_GP_SPI2_MOSI	(CXD56_TOPREG_BASE + 0x2070)
#define CXD56_TOPREG_GP_SPI2_MISO	(CXD56_TOPREG_BASE + 0x2074)
#define CXD56_TOPREG_GP_HIF_IRQ_OUT	(CXD56_TOPREG_BASE + 0x2078)
#define CXD56_TOPREG_GP_HIF_GPIO0	(CXD56_TOPREG_BASE + 0x207c)
#define CXD56_TOPREG_GP_SEN_IRQ_IN	(CXD56_TOPREG_BASE + 0x2090)
#define CXD56_TOPREG_GP_SPI3_CS0_X	(CXD56_TOPREG_BASE + 0x2094)
#define CXD56_TOPREG_GP_SPI3_CS1_X	(CXD56_TOPREG_BASE + 0x2098)
#define CXD56_TOPREG_GP_SPI3_CS2_X	(CXD56_TOPREG_BASE + 0x209c)
#define CXD56_TOPREG_GP_SPI3_SCK	(CXD56_TOPREG_BASE + 0x20a0)
#define CXD56_TOPREG_GP_SPI3_MOSI	(CXD56_TOPREG_BASE + 0x20a4)
#define CXD56_TOPREG_GP_SPI3_MISO	(CXD56_TOPREG_BASE + 0x20a8)
#define CXD56_TOPREG_GP_I2C0_BCK	(CXD56_TOPREG_BASE + 0x20ac)
#define CXD56_TOPREG_GP_I2C0_BDT	(CXD56_TOPREG_BASE + 0x20b0)
#define CXD56_TOPREG_GP_PWM0	(CXD56_TOPREG_BASE + 0x20b4)
#define CXD56_TOPREG_GP_PWM1	(CXD56_TOPREG_BASE + 0x20b8)
#define CXD56_TOPREG_GP_PWM2	(CXD56_TOPREG_BASE + 0x20bc)
#define CXD56_TOPREG_GP_PWM3	(CXD56_TOPREG_BASE + 0x20c0)
#define CXD56_TOPREG_GP_IS_CLK	(CXD56_TOPREG_BASE + 0x20c4)
#define CXD56_TOPREG_GP_IS_VSYNC	(CXD56_TOPREG_BASE + 0x20c8)
#define CXD56_TOPREG_GP_IS_HSYNC	(CXD56_TOPREG_BASE + 0x20cc)
#define CXD56_TOPREG_GP_IS_DATA0	(CXD56_TOPREG_BASE + 0x20d0)
#define CXD56_TOPREG_GP_IS_DATA1	(CXD56_TOPREG_BASE + 0x20d4)
#define CXD56_TOPREG_GP_IS_DATA2	(CXD56_TOPREG_BASE + 0x20d8)
#define CXD56_TOPREG_GP_IS_DATA3	(CXD56_TOPREG_BASE + 0x20dc)
#define CXD56_TOPREG_GP_IS_DATA4	(CXD56_TOPREG_BASE + 0x20e0)
#define CXD56_TOPREG_GP_IS_DATA5	(CXD56_TOPREG_BASE + 0x20e4)
#define CXD56_TOPREG_GP_IS_DATA6	(CXD56_TOPREG_BASE + 0x20e8)
#define CXD56_TOPREG_GP_IS_DATA7	(CXD56_TOPREG_BASE + 0x20ec)
#define CXD56_TOPREG_GP_UART2_TXD	(CXD56_TOPREG_BASE + 0x20f0)
#define CXD56_TOPREG_GP_UART2_RXD	(CXD56_TOPREG_BASE + 0x20f4)
#define CXD56_TOPREG_GP_UART2_CTS	(CXD56_TOPREG_BASE + 0x20f8)
#define CXD56_TOPREG_GP_UART2_RTS	(CXD56_TOPREG_BASE + 0x20fc)
#define CXD56_TOPREG_GP_SPI4_CS_X	(CXD56_TOPREG_BASE + 0x2100)
#define CXD56_TOPREG_GP_SPI4_SCK	(CXD56_TOPREG_BASE + 0x2104)
#define CXD56_TOPREG_GP_SPI4_MOSI	(CXD56_TOPREG_BASE + 0x2108)
#define CXD56_TOPREG_GP_SPI4_MISO	(CXD56_TOPREG_BASE + 0x210c)
#define CXD56_TOPREG_GP_EMMC_CLK	(CXD56_TOPREG_BASE + 0x2110)
#define CXD56_TOPREG_GP_EMMC_CMD	(CXD56_TOPREG_BASE + 0x2114)
#define CXD56_TOPREG_GP_EMMC_DATA0	(CXD56_TOPREG_BASE + 0x2118)
#define CXD56_TOPREG_GP_EMMC_DATA1	(CXD56_TOPREG_BASE + 0x211c)
#define CXD56_TOPREG_GP_EMMC_DATA2	(CXD56_TOPREG_BASE + 0x2120)
#define CXD56_TOPREG_GP_EMMC_DATA3	(CXD56_TOPREG_BASE + 0x2124)
#define CXD56_TOPREG_GP_SDIO_CLK	(CXD56_TOPREG_BASE + 0x2128)
#define CXD56_TOPREG_GP_SDIO_CMD	(CXD56_TOPREG_BASE + 0x212c)
#define CXD56_TOPREG_GP_SDIO_DATA0	(CXD56_TOPREG_BASE + 0x2130)
#define CXD56_TOPREG_GP_SDIO_DATA1	(CXD56_TOPREG_BASE + 0x2134)
#define CXD56_TOPREG_GP_SDIO_DATA2	(CXD56_TOPREG_BASE + 0x2138)
#define CXD56_TOPREG_GP_SDIO_DATA3	(CXD56_TOPREG_BASE + 0x213c)
#define CXD56_TOPREG_GP_SDIO_CD	(CXD56_TOPREG_BASE + 0x2140)
#define CXD56_TOPREG_GP_SDIO_WP	(CXD56_TOPREG_BASE + 0x2144)
#define CXD56_TOPREG_GP_SDIO_CMDDIR	(CXD56_TOPREG_BASE + 0x2148)
#define CXD56_TOPREG_GP_SDIO_DIR0	(CXD56_TOPREG_BASE + 0x214c)
#define CXD56_TOPREG_GP_SDIO_DIR1_3	(CXD56_TOPREG_BASE + 0x2150)
#define CXD56_TOPREG_GP_SDIO_CLKI	(CXD56_TOPREG_BASE + 0x2154)
#define CXD56_TOPREG_GP_I2S0_BCK	(CXD56_TOPREG_BASE + 0x2158)
#define CXD56_TOPREG_GP_I2S0_LRCK	(CXD56_TOPREG_BASE + 0x215c)
#define CXD56_TOPREG_GP_I2S0_DATA_IN	(CXD56_TOPREG_BASE + 0x2160)
#define CXD56_TOPREG_GP_I2S0_DATA_OUT	(CXD56_TOPREG_BASE + 0x2164)
#define CXD56_TOPREG_GP_I2S1_BCK	(CXD56_TOPREG_BASE + 0x2168)
#define CXD56_TOPREG_GP_I2S1_LRCK	(CXD56_TOPREG_BASE + 0x216c)
#define CXD56_TOPREG_GP_I2S1_DATA_IN	(CXD56_TOPREG_BASE + 0x2170)
#define CXD56_TOPREG_GP_I2S1_DATA_OUT	(CXD56_TOPREG_BASE + 0x2174)
#define CXD56_TOPREG_GP_MCLK	(CXD56_TOPREG_BASE + 0x2178)
#define CXD56_TOPREG_GP_PDM_CLK	(CXD56_TOPREG_BASE + 0x217c)
#define CXD56_TOPREG_GP_PDM_IN	(CXD56_TOPREG_BASE + 0x2180)
#define CXD56_TOPREG_GP_PDM_OUT	(CXD56_TOPREG_BASE + 0x2184)
#define CXD56_TOPREG_GP_USB_VBUSINT	(CXD56_TOPREG_BASE + 0x2188)
#define CXD56_TOPREG_YOBI3	(CXD56_TOPREG_BASE + 0x21fc)

/* Topreg sub */
#define CXD56_TOPREG_PSW_CHECK	        (CXD56_TOPREG_SUB_BASE + 0x0000)
#define CXD56_TOPREG_UNEXP_PSW_DIG	    (CXD56_TOPREG_SUB_BASE + 0x0004)
#define CXD56_TOPREG_UNEXP_PSW_ANA	    (CXD56_TOPREG_SUB_BASE + 0x0008)
#define CXD56_TOPREG_UNEXP_OTHER	    (CXD56_TOPREG_SUB_BASE + 0x000c)
#define CXD56_TOPREG_UNEXP_CLR	        (CXD56_TOPREG_SUB_BASE + 0x0010)
#define CXD56_TOPREG_PMU_WAIT10	        (CXD56_TOPREG_SUB_BASE + 0x0020)
#define CXD56_TOPREG_PMU_WAIT11	        (CXD56_TOPREG_SUB_BASE + 0x0024)
#if 0
#define CXD56_TOPREG_PMU_DBG_INITEN	    (CXD56_TOPREG_SUB_BASE + 0x0030)
#define CXD56_TOPREG_PMU_DBG_LUMPEN	    (CXD56_TOPREG_SUB_BASE + 0x0034)
#endif
#define CXD56_TOPREG_SWRESET_DBG	    (CXD56_TOPREG_SUB_BASE + 0x0400)
#define CXD56_TOPREG_SWRESET_GNSDSP	    (CXD56_TOPREG_SUB_BASE + 0x0404)
#define CXD56_TOPREG_SWRESET_APP	    (CXD56_TOPREG_SUB_BASE + 0x0408)
#define CXD56_TOPREG_SYSCPU_CKEN	    (CXD56_TOPREG_SUB_BASE + 0x0410)
#define CXD56_TOPREG_APP_CKEN	        (CXD56_TOPREG_SUB_BASE + 0x0414)
#define CXD56_TOPREG_APP_CKSEL	        (CXD56_TOPREG_SUB_BASE + 0x0418)
#define CXD56_TOPREG_APP_DIV	        (CXD56_TOPREG_SUB_BASE + 0x041c)
#define CXD56_TOPREG_SYSIOP_SUB_CKEN	(CXD56_TOPREG_SUB_BASE + 0x0420)
#define CXD56_TOPREG_ROSC_MON	        (CXD56_TOPREG_SUB_BASE + 0x0428)
#define CXD56_TOPREG_TDC_MON	        (CXD56_TOPREG_SUB_BASE + 0x042c)
#define CXD56_TOPREG_PMU_WAKE_TRIG0_CLR	(CXD56_TOPREG_SUB_BASE + 0x0430)
#define CXD56_TOPREG_PMU_WAKE_TRIG1_CLR	(CXD56_TOPREG_SUB_BASE + 0x0434)
#define CXD56_TOPREG_PMU_WAKE_TRIG0_RAW	(CXD56_TOPREG_SUB_BASE + 0x0438)
#define CXD56_TOPREG_PMU_WAKE_TRIG1_RAW	(CXD56_TOPREG_SUB_BASE + 0x043c)
#define CXD56_TOPREG_PMU_WAKE_TRIG0	    (CXD56_TOPREG_SUB_BASE + 0x0440)
#define CXD56_TOPREG_PMU_WAKE_TRIG1	    (CXD56_TOPREG_SUB_BASE + 0x0444)
#define CXD56_TOPREG_RTC1_CTL	        (CXD56_TOPREG_SUB_BASE + 0x0470)
#define CXD56_TOPREG_GNSS_RAMMODE_SEL	(CXD56_TOPREG_SUB_BASE + 0x0c00)
#define CXD56_TOPREG_SWRESET_GNSDSP2	(CXD56_TOPREG_SUB_BASE + 0x0c10)
#define CXD56_TOPREG_SWRESET_BB	        (CXD56_TOPREG_SUB_BASE + 0x0c14)
#define CXD56_TOPREG_GNSDSP_CKEN	    (CXD56_TOPREG_SUB_BASE + 0x0c20)
#define CXD56_TOPREG_GNSS_BB_CKEN	    (CXD56_TOPREG_SUB_BASE + 0x0c24)
#define CXD56_TOPREG_GNSS_DIV           (CXD56_TOPREG_SUB_BASE + 0x0c28)
#define CXD56_TOPREG_GNSS_RAMMODE_STAT	(CXD56_TOPREG_SUB_BASE + 0x0c30)
#define CXD56_TOPREG_APPDSP_RAMMODE_SEL0 (CXD56_TOPREG_SUB_BASE + 0x1400)
#define CXD56_TOPREG_APPDSP_RAMMODE_SEL1 (CXD56_TOPREG_SUB_BASE + 0x1404)
#define CXD56_TOPREG_APPDSP_RAMMODE_STAT0 (CXD56_TOPREG_SUB_BASE + 0x1420)
#define CXD56_TOPREG_APPDSP_RAMMODE_STAT1 (CXD56_TOPREG_SUB_BASE + 0x1424)
#define CXD56_TOPREG_BUSERR0            (CXD56_TOPREG_SUB_BASE + 0x1470)
#define CXD56_TOPREG_BUSERR1            (CXD56_TOPREG_SUB_BASE + 0x1474)
#define CXD56_TOPREG_BUSERR2            (CXD56_TOPREG_SUB_BASE + 0x1478)
#define CXD56_TOPREG_CHIP_ID            (CXD56_TOPREG_SUB_BASE + 0x1490)
#define CXD56_TOPREG_CUID0              (CXD56_TOPREG_SUB_BASE + 0x1494)
#define CXD56_TOPREG_CUID1              (CXD56_TOPREG_SUB_BASE + 0x1498)
#define CXD56_TOPREG_UDID0              (CXD56_TOPREG_SUB_BASE + 0x149c)
#define CXD56_TOPREG_UDID1              (CXD56_TOPREG_SUB_BASE + 0x14a0)
#define CXD56_TOPREG_FUSE_STATUS        (CXD56_TOPREG_SUB_BASE + 0x14a4)
#define CXD56_TOPREG_SDBG_ENB           (CXD56_TOPREG_SUB_BASE + 0x14a8)
#define CXD56_TOPREG_DBG_MONSEL         (CXD56_TOPREG_SUB_BASE + 0x14c0)

/* PWD_CTL, PWD_STAT */

#define PWD_APP_AUDIO  (1u<<14)
#define PWD_GNSS       (1u<<13)
#define PWD_GNSS_ITP   (1u<<12)
#define PWD_APP_SUB    (1u<<10)
#define PWD_APP_DSP    (1u<<9)
#define PWD_APP        (1u<<8)
#define PWD_SYSIOP_SUB (1u<<6)
#define PWD_SYSIOP     (1u<<5)
#define PWD_CORE       (1u<<4)
#define PWD_SCU        (1u<<0)

/* ANA_PW_CTL */

#define ANA_PW_LPADC  (1u<<13)
#define ANA_PW_HPADC  (1u<<12)
#define ANA_PW_RF_PLL (1u<<9)
#define ANA_PW_RF_LO  (1u<<8)
#define ANA_PW_RF_ADC (1u<<7)
#define ANA_PW_RF_IF  (1u<<6)
#define ANA_PW_RF_MIX (1u<<5)
#define ANA_PW_RF_LNA (1u<<4)
#define ANA_PW_SYSPLL (1u<<2)
#define ANA_PW_XOSC   (1u<<1)
#define ANA_PW_RCOSC  (1u<<0)

/* ANA_EN_CTL */

#define ON_GP_MBI_EN_SET (1u<<27)
#define OFF_GP_MBI_EN_CLR (1u<<26)
#define ON_XO_OSC_EN_SET (1u<<25)
#define OFF_XO_OSC_EN_CLR (1u<<24)
#define ON_XO_OSCOUT_EN_SET (1u<<20)
#define OFF_XO_OSCOUT_EN_CLR (1u<<19)
#define ON_XO_EXT_EN_SET (1u<<18)
#define OFF_XO_EXT_EN_CLR (1u<<17)
#define OFF_XO_CLK_EN_CLR (1u<<16)
#define ON_SP_ENPLL_SET (1u<<8)
#define OFF_SP_ENCLK_CLR (1u<<7)
#define ON_SP_ENOTHER_SET (1u<<6)
#define OFF_SP_ENOTHER_CLR (1u<<5)
#define OFF_SP_ENPLL_CLR (1u<<4)
#define ON_RO_XEN_CLR (1u<<2)
#define OFF_RO_XEN_SET (1u<<1)
#define OFF_RO_CLK_XEN_SET (1u<<0)

/* PMU_INT_STAT */

#define PMU_INT_BOOTEN (1u<<4)
#define PMU_INT_UNEXP_I2C_PMIC (1u<<3)
#define PMU_INT_UNEXP_TIMEOUT  (1u<<2)
#define PMU_INT_NOGO           (1u<<1)
#define PMU_INT_DONE           (1u<<0)

/* CKSEL_ROOT */

#define ENABLE_RF_PLL1 (1u<<4)
#define ENABLE_SOURCE_SEL (1u<<16)

/* CRG_INT_CLR0 */

#define CRG_CK_PCLK_UART0    (1u<<0)
#define CRG_CK_UART0         (1u<<1)
#define CRG_CK_BRG_HOST      (1u<<2)
#define CRG_CK_PCLK_HOSTIFC  (1u<<3)
#define CRG_CK_HOSTIFC_SEQ   (1u<<4)
#define CRG_CK_I2CS          (1u<<5)
#define CRG_CK_RTC_ORG       (1u<<6)
#define CRG_CK_SYSIOP_RTC    (1u<<7)
#define CRG_CK_BRG_SCU       (1u<<8)
#define CRG_CK_SCU           (1u<<9)
#define CRG_CK_SCU_SPI       (1u<<10)
#define CRG_CK_SCU_I2C0      (1u<<11)
#define CRG_CK_SCU_I2C1      (1u<<12)
#define CRG_CK_SCU_SEQ       (1u<<13)
#define CRG_CK_SCU_SC        (1u<<14)
#define CRG_CK_32K           (1u<<15)
#define CRG_CK_U32KH         (1u<<16)
#define CRG_CK_U32KL         (1u<<17)
#define CRG_CK_TADC          (1u<<18)
#define CRG_CK_RTC_PCLK      (1u<<19)
#define CRG_CK_PMU_RTC_PCLK  (1u<<20)
#define CRG_CK_APP           (1u<<21)

/* CRG_INT_CLR1 */

#define CRG_CK_CPU_BUS       (1u<<0)
#define CRG_CK_CPU_BUS_TO    (1u<<1)
#define CRG_CK_RFPLL1        (1u<<2)
#define CRG_CK_RFPLL1_TO     (1u<<3)
#define CRG_CK_RTC_PRE       (1u<<4)
#define CRG_CK_RTC_PRE_TO    (1u<<5)
#define CRG_CK_APP_PRE       (1u<<6)
#define CRG_CK_APP_PRE_TO    (1u<<7)
#define CRG_CK_SEL_SP        (1u<<8)
#define CRG_CK_SEL_SP_TO     (1u<<9)
#define CRG_CK_SEL_RO_RTC    (1u<<10)
#define CRG_FREQFIX_ERR      (1u<<11)

/* SYS_PLL_CTRL1 */

#define ENABLE_DSPCLK (1u<<3)
#define ENABLE_GPADCLK (1u<<1)

/* RCOSC_CTRL2 */

#define DISABLE_SENSCLK (1u<<14)
#define DISABLE_LOGICLK (1u<<13)

/* SWRESET_BUS */

#define XRST_PMU_I2CM     (1u<<16)
#define XRST_I2CM         (1u<<11)
#define XRST_UART0        (1u<<10)
#define XRST_HOSTIFC_ISOP (1u<<9)
#define XRST_HOSTIFC      (1u<<8)
#define XRST_KAKI         (1u<<6)
#define XRST_UART1        (1u<<5)
#define XRST_SAKE         (1u<<2)
#define XRST_SFC          (1u<<1)
#define XRST_SPIM         (1u<<0)

/* SWRESET_SCU */

#define XRST_SCU_SPI      (1u<<8)
#define XRST_SCU_ISOP     (1u<<7)
#define XRST_SCU_I2C1     (1u<<6)
#define XRST_SCU_I2C2     (1u<<5)
#define XRST_SCU_LPADC    (1u<<4)
#define XRST_SCU_HPADC    (1u<<2)

/* SYSIOP_CKEN */

#define CKEN_HOSSPI        (1u<<17)
#define CKEN_HOSI2C        (1u<<16)
#define CKEN_HOSTIFC_SEQ   (1u<<15)
#define CKEN_BRG_SCU       (1u<<14)
#define CKEN_SYSIOP_RTC    (1u<<13)
#define CKEN_RCOSC_OUT     (1u<<12)
#define CKEN_AP_CLK        (1u<<11)
#define CKEN_RTC_ORG       (1u<<10)
#define CKEN_FREQDIS       (1u<<9)
#define CKEN_APB           (1u<<8)
#define CKEN_AHB_DMAC2     (1u<<7)
#define CKEN_AHB_DMAC1     (1u<<6)
#define CKEN_AHB_DMAC0     (1u<<5)
#define CKEN_BRG_HOST      (1u<<4)
#define CKEN_I2CS          (1u<<3)
#define CKEN_PCLK_HOSTIFC  (1u<<2)
#define CKEN_PCLK_UART0    (1u<<1)
#define CKEN_UART0         (1u<<0)

/* IOOEN_APP */

#define I2S1_LRCK (1u<<5)
#define I2S1_BCK (1u<<4)
#define I2S0_LRCK (1u<<1)
#define I2S0_BCK (1u<<0)

/* SYSIOP_SUB_CKEN */

#define CK_COM_UART_PCLK (1u<<16)
#define CK_SFC_HCLK_LOW  (1u<<9)
#define CK_SFC_SFCLK     (1u<<8)
#define CK_SFC_HCLK      (1u<<7)
#define CK_SFC           (CK_SFC_HCLK | CK_SFC_SFCLK | CK_SFC_HCLK_LOW)
#define CK_HCLK_SAKE     (1u<<6)
#define CK_I2CM          (1u<<5)
#define CK_SPIM          (1u<<4)
#define CK_UART1         (1u<<3)
#define CK_AHB_DMAC3     (1u<<2)
#define CK_COM_BRG       (1u<<1)
#define CK_AHB_BRG_COMIF (1u<<0)

/* PWD_CTL */

#define PWD_CTL_APP_SUB  (1u<<10)

/* SCU_CKEN */

#define SCU_SCU          (1u<<0)
#define SCU_I2C0         (1u<<1)
#define SCU_I2C1         (1u<<2)
#define SCU_SPI          (1u<<3)
#define SCU_SEQ          (1u<<4)
#define SCU_32K          (1u<<5)
#define SCU_U32KL        (1u<<6)
#define SCU_U32KH        (1u<<7)
#define SCU_SC           (1u<<8)

/* APP_CKEN */

#define APP_CKEN_CPU     (1u<<0)
#define APP_CKEN_MCLK    (1u<<1)
#define APP_CKEN_AHB     (1u<<3)

/* APP_CKSEL */

#define AUD_MCLK_MASK    (3u<<16)
#define AUD_MCLK_EXT     (0u<<16) /* External XTAL */
#define AUD_MCLK_XOSC    (1u<<16) /* Internal XOSC */
#define AUD_MCLK_RCOSC   (2u<<16) /* Internal RCOSC */

/* GNSDSP_CKEN */

#define GNSDSP_CKEN_P1   (1u<<5)
#define GNSDSP_CKEN_COP  (1u<<7)

#endif
