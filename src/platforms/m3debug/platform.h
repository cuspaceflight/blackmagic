/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "version.h"

#include <setjmp.h>

#define PLATFORM_HAS_TRACESWO
#define BOARD_IDENT       "Black Magic Probe (m3debug), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU   "Black Magic (Upgrade) for m3debug, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT         "Black Magic Firmware Upgrade (m3debug)"
#define DFU_IFACE_STRING  "@Internal Flash   /0x08000000/1*016Ka,3*016Kg,1*064Kg,7*128Kg"

/* Important pin mappings for m3debug implementation:
 *
 * LED0 = PA4 = Yellow LED = CAN active
 * LED1 = PA5 = Yellow LED = JTAG active
 * LED2 = PA6 = Red    LED = Error
 *
 * USB VBUS detect: PA9
 *
 * TPWR = PB0
 *
 * CAN_RXD = PB8
 * CAN_TXD = PB9
 *
 * JTMS = PC0
 * JTCK = PC1
 * JTDI = PC2
 * JTDO = PC3
 *
 * nRST and nTRST and SRST_OUT all unconnected.
 * No bootloader button connected.
 */

/* Hardware definitions... */
#define JTAG_PORT 	GPIOC
#define TDI_PORT	JTAG_PORT
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	JTAG_PORT
#define TDI_PIN		GPIO2
#define TMS_PIN		GPIO0
#define TCK_PIN		GPIO1
#define TDO_PIN		GPIO3

#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	GPIOC
#define TRST_PIN	GPIO4
#define SRST_PORT	GPIOC
#define SRST_PIN	GPIO5

#define LED_PORT	GPIOA
#define LED_PORT_UART	LED_PORT
#define LED_UART	GPIO4
#define LED_IDLE_RUN	GPIO5
#define LED_ERROR	GPIO6
#define LED_BOOTLOADER	GPIO7

#define TMS_SET_MODE() \
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, \
	                GPIO_PUPD_NONE, TMS_PIN);
#define SWDIO_MODE_FLOAT() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, \
	                GPIO_PUPD_NONE, SWDIO_PIN);

#define SWDIO_MODE_DRIVE() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, \
	                GPIO_PUPD_NONE, SWDIO_PIN);


#define USB_DRIVER      stm32f107_usb_driver
#define USB_IRQ         NVIC_OTG_FS_IRQ
#define USB_ISR         otg_fs_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB		(2 << 4)
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBCAN      (1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#define IRQ_PRI_TRACE           (0 << 4)

#define TRACE_TIM           TIM3
#define TRACE_TIM_CLK_EN()  rcc_periph_clock_enable(RCC_TIM3)
#define TRACE_IRQ           NVIC_TIM3_IRQ
#define TRACE_ISR           tim3_isr


#define DEBUG(...)

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)	{gpio_set_val(LED_PORT, LED_ERROR, state);}

static inline int platform_hwversion(void)
{
	return 0;
}

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#endif

