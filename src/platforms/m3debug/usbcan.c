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

#define CAN_ID_M3FC      (1)
#define CAN_ID_M3IMU     (5)

#define CAN_MSG_ID(x)    (x<<5)

#define CAN_MSG_ID_STATUS                   CAN_MSG_ID(0)

#define CAN_MSG_ID_M3IMU_STATUS             (CAN_ID_M3IMU | CAN_MSG_ID_STATUS)
#define CAN_MSG_ID_M3FC_ACCEL               (CAN_ID_M3FC | CAN_MSG_ID(48))
#define CAN_MSG_ID_M3FC_BARO                (CAN_ID_M3FC | CAN_MSG_ID(49))

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

static void usbcan_can_init(void);

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
    (void)coding;
}

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
     * if we hit an error it'l just stay on.
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
}

void can1_rx0_isr(void);

/* Read a new CAN frame and store it in the FIFO. */
void can1_rx0_isr() {
    uint32_t id, fmi;
    bool ext, rtr;
    uint8_t length, data[8];
    struct can_msg msg;

    can_receive(CAN1, 0, true, &id, &ext, &rtr, &fmi, &length, data);

    /* Skip sensor logging packets that just spam the usb */
    if(id == CAN_MSG_ID_M3FC_ACCEL || id == CAN_MSG_ID_M3FC_BARO ||
       (((id & 0x01F) == CAN_ID_M3IMU) && (id != CAN_MSG_ID_M3IMU_STATUS))) {
        return;
    }

    /* Early return if no USB connected */
    if(cdcacm_get_config() != 1) {
        return;
    }

    gpio_set(LED_PORT_UART, LED_UART);

    msg.id = id;
    msg.rtr = rtr;
    msg.len = length;
    memcpy(msg.data, data, 8);

    uint8_t packet_buf[32];
    int packet_idx=0;
    packet_buf[packet_idx++] = 0x7E;
    for(int i=0; i<12; i++) {
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
}

/* Send CAN packets from computer to stack */
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
