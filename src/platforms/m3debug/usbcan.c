#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "general.h"
#include "cdcacm.h"
#include "platform.h"

struct can_tx_msg {
    uint32_t std_id;
    uint32_t ext_id;
    uint8_t ide;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
};

struct can_rx_msg {
    uint32_t std_id;
    uint32_t ext_id;
    uint8_t ide;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t fmi;
};

struct can_tx_msg can_tx_msg;
struct can_rx_msg can_rx_msg;

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
    (void)coding;
}

void usbuart_usb_out_cb(usbd_device *dev, uint8_t ep)
{
    (void)dev;
    (void)ep;
}

void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
    (void)dev;
    (void)ep;
}


void usbcan_init(void)
{
    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO8 | GPIO9);
    nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
    nvic_enable_irq(NVIC_CAN1_RX1_IRQ);
    nvic_enable_irq(NVIC_CAN1_SCE_IRQ);
    nvic_enable_irq(NVIC_CAN1_TX_IRQ);
    nvic_set_priority(NVIC_CAN1_RX0_IRQ, IRQ_PRI_USBCAN);
    nvic_set_priority(NVIC_CAN1_RX1_IRQ, IRQ_PRI_USBCAN);
    nvic_set_priority(NVIC_CAN1_SCE_IRQ, IRQ_PRI_USBCAN);
    nvic_set_priority(NVIC_CAN1_TX_IRQ, IRQ_PRI_USBCAN);

    can_reset(CAN1);

    if(can_init(CAN1,
        false,              /* TTCM Time Triggered Comm Mode */
        true,               /* ABOM Auto Bus Off Management */
        false,              /* AWUM Auto Wake Up Mode */
        false,              /* NART No Auto ReTransmission */
        false,              /* RFLM Receive FIFO locked mode */
        false,              /* TXFP Transmit FIFO priority */
        CAN_BTR_SJW_1TQ,    /* Resync time quanta jump width */
        CAN_BTR_TS1_9TQ,    /* Time segment 1 time quanta width */
        CAN_BTR_TS2_6TQ,    /* Time segment 2 time quanta width */
        2,                  /* Baud rate prescaler */
        false,              /* Loopback */
        false               /* Silent */
        ))
    {
        gpio_set(LED_PORT, LED_ERROR);
        while(1);
    }

    can_filter_id_mask_32bit_init(CAN1,
        0,                  /* Filter ID */
        0,                  /* CAN ID */
        0,                  /* CAN ID Mask */
        0,                  /* FIFO assignment: FIFO0 */
        true);              /* Enable filter */

    can_enable_irq(CAN1, CAN_IER_FMPIE0);

}

void can1_rx0_isr(void);
void can1_rx1_isr(void);
void can1_sce_isr(void);
void can1_tx_isr(void);
