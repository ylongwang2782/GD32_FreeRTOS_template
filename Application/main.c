#include <stdint.h>
#include <stdio.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "freertos.h"
#include "gd32f4xx.h"
#include "systick.h"
#include "task.h"

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC
                        size). Used in RX only. */
};

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be
 * examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be
 * examined at a debug breakpoint. */
static uint16_t frame_len = 0;

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127

static uint8_t rx_buffer[FRAME_LEN_MAX];

void uart3_init() {
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_UART3);

    gpio_af_set(GPIOA, GPIO_AF_8, GPIO_PIN_0);
    gpio_af_set(GPIOA, GPIO_AF_8, GPIO_PIN_1);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_0);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_1);

    usart_deinit(UART3);
    usart_baudrate_set(UART3, 921600);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    usart_enable(UART3);
}

void spi3_init() {
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_SPI3);
    spi_parameter_struct spi_init_struct;

    // PE2 CLK  PE4 NSS  PE5 MISO   PE6 MOSI
    gpio_af_set(GPIOE, GPIO_AF_5, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6);

    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  GPIO_PIN_4);    // NSS
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_4);

    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  GPIO_PIN_3);    // RST
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_3);

    gpio_bit_set(GPIOE, GPIO_PIN_4);

    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.prescale = SPI_PSC_32;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init(SPI3, &spi_init_struct);

    spi_nss_output_disable(SPI3);
    spi_enable(SPI3);
}

void port_set_dw1000_fastrate_spi3(void) {
    spi_disable(SPI3);

    spi_parameter_struct spi_init_struct;
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.prescale = SPI_PSC_4;
    spi_init(SPI3, &spi_init_struct);

    spi_enable(SPI3);
}

void reset_DW1000(void) {
    gpio_bit_reset(GPIOE, GPIO_PIN_3);    // reset pin
    vTaskDelay(5);                        // hold
    gpio_bit_set(GPIOE, GPIO_PIN_3);
}

static void Slave_Task(void *pvParameters) {
    // init running led
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_13);
    gpio_bit_set(GPIOC, GPIO_PIN_13);

    // init uart3
    uart3_init();

    spi3_init();
    reset_DW1000();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        printf("dw1000 init failed");
        while (1) {
        };
    }
    port_set_dw1000_fastrate_spi3();

    /* Configure DW1000. See NOTE 7 below. */
    dwt_configure(&config);
    int i;

    while (1) {
        for (i = 0; i < FRAME_LEN_MAX; i++) {
            rx_buffer[i] = 0;
        }

        /* Activate reception immediately. See NOTE 3 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
        };

        if (status_reg & SYS_STATUS_RXFCG) {
            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            printf("recv len: %d\n", frame_len);

            if (frame_len <= FRAME_LEN_MAX) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            for (i = 0; i < frame_len; i++) {
                // printf("%x", rx_buffer[i]);
                usart_data_transmit(UART3, rx_buffer[i]);
                while (usart_flag_get(UART3, USART_FLAG_TBE) == RESET);
            }
        } else if (status_reg & SYS_STATUS_ALL_RX_TO) {
            printf("timeout");
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
            dwt_rxreset();
        } else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
            printf("error");
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
        }

        gpio_bit_toggle(GPIOC, GPIO_PIN_13);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

int main(void) {
    systick_config();

    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    xTaskCreate(Slave_Task, "SlaveTask", 256, NULL, 2, NULL);

    vTaskStartScheduler();
    for (;;) {
    };
    return 0;
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f) {
    usart_data_transmit(UART3, (uint8_t)ch);
    while (RESET == usart_flag_get(UART3, USART_FLAG_TBE));
    return ch;
}
