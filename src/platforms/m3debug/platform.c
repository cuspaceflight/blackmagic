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

#include "general.h"
#include "cdcacm.h"
#include "morse.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/otg_fs.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>

jmp_buf fatal_error_jmpbuf;

static void adc_init(void);
void usbcan_init(void);

const clock_scale_t hse_26mhz_in_168mhz_out = {
    .pllm = 13,
    .plln = 168,
    .pllp = 2,
    .pllq = 7,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .power_save = 1,
    .flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE |
                    FLASH_ACR_LATENCY_5WS,
    .apb1_frequency = 42000000,
    .apb2_frequency = 84000000,
};

void platform_init(void)
{
    /* Set up system clock to 168MHz */
	rcc_clock_setup_hse_3v3(&hse_26mhz_in_168mhz_out);

	/* Enable peripherals */
	rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_CRCEN);

	/* Set up USB Pins and alternate function*/
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO9);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
		GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	GPIOC_OSPEEDR &=~0xF30;
	GPIOC_OSPEEDR |= 0xA20;
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,
			TMS_PIN | TCK_PIN | TDI_PIN);

	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT,
			GPIO_PUPD_NONE,
			TDO_PIN);

	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,
			LED_UART | LED_IDLE_RUN | LED_ERROR);

    platform_srst_set_val(false);
    gpio_mode_setup(SRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SRST_PIN);
    gpio_set_output_options(SRST_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, SRST_PIN);

	platform_timing_init();
    usbcan_init();
	cdcacm_init();
    adc_init();
}

void platform_srst_set_val(bool assert) {
    gpio_set_val(SRST_PORT, SRST_PIN, !assert);
    if(assert) {
        for(int i=0; i<10000; i++) asm("nop");
    }
}
bool platform_srst_get_val(void) {
    return gpio_get(SRST_PORT, SRST_PIN) == 0;
}

static void adc_init()
{
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG,
			GPIO_PUPD_NONE, GPIO0);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL0, 5);
    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
    adc_set_right_aligned(ADC1);
    adc_power_on(ADC1);
}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.0V";
	const uint8_t channel = 8;
	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&channel);

	adc_start_conversion_regular(ADC1);

	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1));

	uint32_t val = adc_read_regular(ADC1) * 99; /* 0-4095 */
	ret[0] = '0' + val / 81910;
	ret[2] = '0' + (val / 8191) % 10;

	return ret;
}

void platform_request_boot(void)
{
    /* No bootloader on m3debug */
    for(;;);
}
