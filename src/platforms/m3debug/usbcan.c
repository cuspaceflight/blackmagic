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

/* our on-the-wire format for can messages between this board and the computer.
 * note that over serial they then get framed by 0x7E, with subsequent 0x7E and
 * 0x7D bytes escaped by 0x7D and then given as 0x5E and 0x5D respectively.
 */
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

/* store up to CDCACM_PACKET_SIZE(=64 bytes) worth of packets before
 * sending over USB, in practice between 1 to 4 packets */
static uint8_t can_buf[CDCACM_PACKET_SIZE];
static uint8_t can_buf_idx = 0;

/* required for usb cdc or whatever */
void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
    (void)coding;
}

/* required for usb cdc or whatever */
void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
    (void)dev;
    (void)ep;
}

/* write the can buffer out over usb and reset the index */
static void dump_can_buffer(void) {
    if(cdcacm_get_config() == 1 &&
       can_buf_idx > 0          &&
       can_buf_idx <= CDCACM_PACKET_SIZE)
    {
        memset(can_buf + can_buf_idx, 0, sizeof(can_buf) - can_buf_idx);
        usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,
                             can_buf, sizeof(can_buf));
    }
    can_buf_idx = 0;
}

/* compute the total length required for the escaped can message over usb,
 * which is 1 (framing character) + 12 (data) + number of escaped bytes
 */
static size_t msg_escaped_length(struct can_msg *msg) {
    size_t len = 13;
    for(int i=0; i<12; i++) {
        if(msg->raw[i] == 0x7D || msg->raw[i] == 0x7E) {
            len++;
        }
    }
    return len;
}

/* check how much space is left in the can buffer */
static size_t can_buf_free_space(void) {
    return CDCACM_PACKET_SIZE - can_buf_idx;
}

/* Custom CAN init function as the timeout in libopencm3 is too
 * small to work on a 168MHz F4, but there's no good way to
 * override it without resorting to horrible hacks. Like this.
 */
static void usbcan_can_init(void)
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
    /* turn on the CAN peripheral */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO8 | GPIO9);
    usbcan_can_init();
    nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
    nvic_set_priority(NVIC_CAN1_RX0_IRQ, IRQ_PRI_USBCAN);

    /* Enable timer to dump the buffer every 100ms */
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, 2*(rcc_ppre2_frequency / 1000000) - 1);
    timer_set_period(TIM4, 100000 - 1);
    nvic_set_priority(NVIC_TIM4_IRQ, IRQ_PRI_USBCAN);
    nvic_enable_irq(NVIC_TIM4_IRQ);
    timer_enable_counter(TIM4);
    timer_enable_irq(TIM4, TIM_DIER_UIE);
}

/* Read a new CAN frame and store it in the buffer,
 * dumping to USB when the buffer fills up.
 */
void can1_rx0_isr(void) {
    uint32_t id, fmi;
    bool ext, rtr;
    uint8_t length, data[8];
    struct can_msg msg;

    gpio_set(LED_PORT_UART, LED_UART);

    /* receive incoming message into struct */
    can_receive(CAN1, 0, true, &id, &ext, &rtr, &fmi, &length, data);
    msg.id = id;
    msg.rtr = rtr;
    msg.len = length;
    memcpy(msg.data, data, length);
    memset(msg.data+length, 0, 8-length);

    /* Early return if no USB connected */
    if(cdcacm_get_config() != 1) {
        return;
    }

    /* flush buffer if not enough space for this packet */
    if(msg_escaped_length(&msg) > can_buf_free_space()) {
        dump_can_buffer();
    }

    /* save message into buffer */
    can_buf[can_buf_idx++] = 0x7E;
    for(int i=0; i<12; i++) {
        uint8_t c = msg.raw[i];
        if(c == 0x7E) {
            can_buf[can_buf_idx++] = 0x7D;
            can_buf[can_buf_idx++] = 0x5E;
        } else if(c == 0x7D) {
            can_buf[can_buf_idx++] = 0x7D;
            can_buf[can_buf_idx++] = 0x5D;
        } else {
            can_buf[can_buf_idx++] = c;
        }
    }

    /* if there's definitely not enough space left for a new packet, flush */
    if(can_buf_free_space() < 13) {
        dump_can_buffer();
    }

    gpio_clear(LED_PORT_UART, LED_UART);
}

/* dump the buffer over usb every 100ms too, if it's not empty */
void tim4_isr(void) {
    timer_clear_flag(TIM4, TIM_SR_UIF);
    dump_can_buffer();
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
