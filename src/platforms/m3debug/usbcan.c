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

#define USBCAN_TIMER_FREQ_HZ 1000000U /* 1MHz timer      */
#define USBCAN_RUN_FREQ_HZ   5000U    /* Run every 200us */

#define USBCAN_USE_FIFO      1

struct can_msg {
    union {
        struct {
            uint16_t id;
            uint8_t  rtr;
            uint8_t  len;
            uint8_t  data[8];
        };
        uint8_t raw[12];
    };
};

#define USBCAN_FIFO_SIZE 64
// Circular buffer of CAN messages received from CAN and to be sent to USB
#if USBCAN_USE_FIFO
static struct can_msg can_buf[USBCAN_FIFO_SIZE];
static uint8_t can_buf_in, can_buf_out;
#endif

static void usbcan_run(void);
static void usbcan_can_init(void);

// Function required by cdcacm.c for us to pretend to be usbuart
void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
    (void)coding;
}

// Function required by cdcacm.c for us to pretend to be usbuart
void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
    (void)dev;
    (void)ep;
}


/* Custom CAN init function as the timeout in libopencm3 is too
 * small to work on a 168MHz F4, but there's no good way to
 * override it without resorting to horrible hacks. Like this.
 */
static void usbcan_can_init()
{
    /* Turn on error LED first, turn it off once initialised,
     * if we hit an error it'll just stay on.
     */
    gpio_set(LED_PORT, LED_ERROR);

    /* Reset CAN peripheral */
    can_reset(CAN1);

    /* Leave sleep */
    CAN_MCR(CAN1) &= ~CAN_MCR_SLEEP;
    /* Wait to leave sleep */
    while((CAN_MSR(CAN1) & CAN_MSR_SLAK) == CAN_MSR_SLAK);
    /* Enter initialisation */
    CAN_MCR(CAN1) |= CAN_MCR_INRQ;
    /* Wait for acknowledgement */
    while((CAN_MSR(CAN1) & CAN_MSR_INAK) != CAN_MSR_INAK);
    /* Set config bits */
    CAN_MCR(CAN1) = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_INRQ;
    /* Set timing bits for 1Mbps */
    CAN_BTR(CAN1) = (1<<CAN_BTR_SJW_SHIFT) | (3<<CAN_BTR_TS1_SHIFT) |
                    (1<<CAN_BTR_TS2_SHIFT) | (5);
    /* Initialise the filter banks: one all-matching filter to FIFO0 */
    can_filter_init(CAN1, 0, true, false, 0, 0, 0, true);
    /* Leave initialisation */
    CAN_MCR(CAN1) &= ~CAN_MCR_INRQ;
    /* Wait for acknowledgement */
    while((CAN_MSR(CAN1) & CAN_MSR_INAK) == CAN_MSR_INAK);

    /* Enable interrupt on packet received */
    can_enable_irq(CAN1, CAN_IER_FMPIE0);

    /* Clear LED once initialisation is complete */
    gpio_clear(LED_PORT, LED_ERROR);
}


void usbcan_init(void)
{
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO8 | GPIO9);

    usbcan_can_init();

    nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
    nvic_set_priority(NVIC_CAN1_RX0_IRQ, IRQ_PRI_USBCAN);

#if USBCAN_USE_FIFO
    /* Enable timer for processing FIFO */
    /* How often does this timer interrupt? */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                         TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4,
        rcc_ppre2_frequency / USBCAN_TIMER_FREQ_HZ * 2 - 1);
    timer_set_period(TIM4,
        USBCAN_TIMER_FREQ_HZ / USBCAN_RUN_FREQ_HZ - 1);
    nvic_set_priority(NVIC_TIM4_IRQ, (3<<4));
    nvic_enable_irq(NVIC_TIM4_IRQ);
    timer_enable_counter(TIM4);
#endif
}

void can1_rx0_isr(void);
void tim4_isr(void);

/* Read a new CAN frame
 * In FIFO mode, store it in the FIFO
 * In non-FIFO mode, send it straight over USB. */
void can1_rx0_isr() {
    uint32_t id, fmi;
    bool ext, rtr;
    uint8_t length, data[8];
    struct can_msg msg;

    gpio_set(LED_PORT_UART, LED_UART);

    can_receive(CAN1, 0, true, &id, &ext, &rtr, &fmi, &length, data);

    /* Discard the packet if no USB connected */
    if(cdcacm_get_config() != 1) {
        return;
    }

    msg.id = id;
    msg.rtr = rtr;
    msg.len = length;
    memcpy(msg.data, data, 8);

#if USBCAN_USE_FIFO
    /* Check the FIFO isn't full.  If it is, discard the packet */
    if(((can_buf_in + 1) % USBCAN_FIFO_SIZE) != can_buf_out) {

        /* Insert into FIFO */
        can_buf[can_buf_in++] = msg;

        /* Wrap pointer */
        if(can_buf_in >= USBCAN_FIFO_SIZE) {
            can_buf_in = 0;
        }

    }

    /* Enable processing of FIFO data later */
    timer_enable_irq(TIM4, TIM_DIER_UIE);
#else
    uint8_t packet_buf[CDCACM_PACKET_SIZE];
    int packet_idx=0, i;
    packet_buf[packet_idx++] = 0x7E;
    for(i=0; i<12; i++) {
        uint8_t c = msg.raw[i];
        if(c == 0x7E) {
            packet_buf[packet_idx++] = 0x7D;
            packet_buf[packet_idx++] = 0x5E;
        } else if(c == 0x7D) {
            packet_buf[packet_idx++] = 0x7D;
            packet_buf[packet_idx++] = 0x5D;
        } else {
            packet_buf[packet_idx++] = c;
        }
    }
    usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,
                         packet_buf, packet_idx);

    gpio_clear(LED_PORT_UART, LED_UART);
#endif
}

/* Send CAN packets from computer to m3avionics */
void usbuart_usb_out_cb(usbd_device *dev, uint8_t ep)
{
    (void)ep;
    int i;
    struct can_msg msg;
    char buf[CDCACM_PACKET_SIZE];
    int len;
    static bool in_frame = false;
    static int msg_idx = 0;

    len = usbd_ep_read_packet(dev, CDCACM_UART_ENDPOINT,
                              buf, CDCACM_PACKET_SIZE);
    gpio_set(LED_PORT_UART, LED_UART);

    for(i=0; i<len; i++) {
        if(!in_frame) {
            if(buf[i] == 0x7E) {
                in_frame = true;
                msg_idx = 0;
            }
        } else {
            if(buf[i] == 0x7D) {
                msg.raw[msg_idx++] = buf[++i] ^ 0x20;
            } else {
                msg.raw[msg_idx++] = buf[i];
            }

            if(msg_idx >= 4 && msg_idx == msg.len + 4) {
                in_frame = false;
                can_transmit(CAN1, msg.id, false, msg.rtr, msg.len, msg.data);
            }
        }
    }

    gpio_clear(LED_PORT_UART, LED_UART);
}


/* Send CAN packets from m3avionics via FIFO to host over CDCACM */
static void usbcan_run() {
#if USBCAN_USE_FIFO
    /* Force empty buffer if no USB endpoint */
    if(cdcacm_get_config() != 1) {
        can_buf_out = can_buf_in;
    }

    if(can_buf_in == can_buf_out) {
        /* If FIFO empty, nothing to do */
        timer_disable_irq(TIM4, TIM_DIER_UIE);
        gpio_clear(LED_PORT_UART, LED_UART);
    } else {
        /* can_buf_in != can_buf_out, so we should have some packets! */
        /* Otherwise write CAN frames into USB packet buffer and send them */
        uint8_t packet_buf[CDCACM_PACKET_SIZE + 32];
        uint8_t packet_idx = 0;
        volatile uint8_t packet_size = 0;
        int i;

        // Add as many CAN messages to the USB packet as will fit or are
        // available.
        while(can_buf_in != can_buf_out) {
            // Pop the oldest message out of the buffer, wrap pointer if needed
            struct can_msg msg = can_buf[can_buf_out];
            can_buf_out += 1;
            if(can_buf_out >= USBCAN_FIFO_SIZE) {
                can_buf_out = 0;
            }

            packet_buf[packet_idx++] = 0x7E;
            for(i=0; i<12; i++) {
                uint8_t c = msg.raw[i];
                if(c == 0x7E) {
                    packet_buf[packet_idx++] = 0x7D;
                    packet_buf[packet_idx++] = 0x5E;
                } else if(c == 0x7D) {
                    packet_buf[packet_idx++] = 0x7D;
                    packet_buf[packet_idx++] = 0x5D;
                } else {
                    packet_buf[packet_idx++] = c;
                }
            }

            if(packet_idx >= CDCACM_PACKET_SIZE) {
                // The USB packet is (over)full, so we need can't send the last
                // packet.  Move the pointer backwards so this last packet gets
                // processed next time, wrapping if necessary.
                if(can_buf_out == 0) {
                    can_buf_out = USBCAN_FIFO_SIZE - 1;
                } else {
                    can_buf_out--;
                }
                break;  // Stop filling the packet and send it.

            } else {
                packet_size = packet_idx;
            }
        }
        // TODO: How are we getting to this point with packet_size == 0
        // This is what I am observing and I am deeply suspicious, but I'm not
        // sure how much it correlates with the weird lack-of-packets bug.
        usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,
                             packet_buf, packet_size);
    }
#endif
}

void tim4_isr() {
    timer_clear_flag(TIM4, TIM_SR_UIF);
    usbcan_run();
}
