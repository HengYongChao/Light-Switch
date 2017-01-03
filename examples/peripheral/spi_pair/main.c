/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 * @defgroup spi_pair main.c
 * @{
 * @ingroup spi_master_example
 *
 * @brief SPI master example application to be used with the SPI slave example application.
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "bsp.h"
#include "app_timer.h"
#include "spi_master.h"
#include "nordic_common.h"

/*
 * This example uses only one instance of the SPI master.
 * Please make sure that only one instance of the SPI master is enabled in config file.
 */

#define APP_TIMER_PRESCALER      0                      /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS     BSP_APP_TIMERS_NUMBER  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE  2                      /**< Size of timer operation queues. */

//#define DELAY_US                 1500                /**< Timer Delay between transmissions (us). */

#define TX_RX_BUF_LENGTH         2                 /**< SPI transaction buffer length. */

#if !defined(SPI_MASTER_0_ENABLE) && !defined(SPI_MASTER_1_ENABLE)
    #error "Both SPI need to be enabled"
#endif

#define STORED 10 // number of values in the Look Up Table (LUT)
#define Fs 12500 // Sampling frequency (Hz) (with no other functions running, this averages 80us)

// Data buffers.
static uint8_t m_tx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer with data to transfer. */
static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */
static uint16_t sineWave[STORED] = {0};
static uint16_t triangleWave[STORED*4] = {0};
static uint16_t rTriangleWave[STORED*4] = {0};

static volatile bool m_transfer_completed = true; /**< A flag informing completed transfer. */
static volatile bool negate = false; // variable indicating inverted wave
static volatile uint16_t frequency = 50; // Frequency of the output wave
static volatile uint16_t lastFreq = 0;
static volatile uint16_t delay_us = 170; // delay in microseconds for slowing down a waveform

void sineInit(){
    float dr = 3.14/(2*STORED); // increment value for sine storing
	uint8_t i;
	for(i = 0; i<STORED; i++) {
		sineWave[i] = (uint16_t)(sin((float)i*dr)*2048.0+2047.0); // map sine to ADC values
	}	
}

/** Triangle Wave :
 *        /|   /|   /|   /|
 *       / |  / |  / |  / |
 *      /  | /  | /  | /  |
 *  ___/   |/   |/   |/   |___
 */ 
void triangleInit(){
    float dr = 4095/(STORED*4);
    uint8_t i;
    for(i = 0; i<STORED*4; i++){
	triangleWave[i] = (uint16_t)(i*dr);
    }
}

/**
 *    |\   |\   |\   |\
 *    | \  | \  | \  | \
 *    |  \ |  \ |  \ |  \
 * ___|   \|   \|   \|   \___
 */
void rTriangleInit(){
    float dr = 4095/(STORED*4);
    uint8_t i;
    for(i = 0; i<STORED*4; i++){
	rTriangleWave[i] = (uint16_t)(4095-i*dr);
    }
}

   
/**@brief Function for SPI master event callback.
 *
 * Upon receiving an SPI transaction complete event, checks if received data are valid.
 *
 * @param[in] spi_master_evt    SPI master driver event.
 */
static void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    //bool result = false;

    switch (spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:

            // Check if data are vaild.
            //result = buf_check(m_rx_data, spi_master_evt.data_count);
            //APP_ERROR_CHECK_BOOL(result);

            err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
            APP_ERROR_CHECK(err_code);

            // Inform application that transfer is completed.
            m_transfer_completed = true;
            break;

        default:
            // No implementation needed.
            break;
    }
}
    
/**@brief Function for initializing a SPI master driver.
 *
 * @param[in] spi_master_instance       An instance of SPI master module.
 * @param[in] spi_master_event_handler  An event handler for SPI master events.
 * @param[in] lsb                       Bits order LSB if true, MSB if false.
 */
static void spi_master_init()
{
    uint32_t err_code = NRF_SUCCESS;

    // Configure SPI master.
    spi_master_config_t spi_config ={                                                                           \
    SPI_FREQUENCY_FREQUENCY_M8, /**< Serial clock frequency 8 Mbps. Found in nrf51_bitfields.h. */      \
    SPIM0_SCK_PIN,       /**< SCK pin DISCONNECTED. */               \
    SPIM0_MISO_PIN,       /**< MISO pin DISCONNECTED. */              \
    SPIM0_MOSI_PIN,       /**< MOSI pin DISCONNECTED. */              \
    SPIM0_SS_PIN,       /**< Slave select pin DISCONNECTED. */      \
    APP_IRQ_PRIORITY_LOW,       /**< Interrupt priority LOW. */             \
    SPI_CONFIG_ORDER_MsbFirst,  /**< Bits order MSB. */                     \
    SPI_CONFIG_CPOL_ActiveHigh, /**< Serial clock polarity ACTIVEHIGH. */   \
    SPI_CONFIG_CPHA_Leading,    /**< Serial clock phase LEADING. */         \
    0                           /**< Don't disable all IRQs. */             \
};/*SPI_MASTER_INIT_DEFAULT;*/

    err_code = spi_master_open(SPI_MASTER_0, &spi_config);
    spi_master_evt_handler_reg(SPI_MASTER_0, spi_master_event_handler);
    APP_ERROR_CHECK(err_code);

    spi_config.SPI_Pin_SCK  = SPIM1_SCK_PIN;
    spi_config.SPI_Pin_MISO = SPIM1_MISO_PIN;
    spi_config.SPI_Pin_MOSI = SPIM1_MOSI_PIN;
    spi_config.SPI_Pin_SS   = SPIM1_SS_PIN;

    err_code = spi_master_open(SPI_MASTER_1, &spi_config);
    spi_master_evt_handler_reg(SPI_MASTER_1, spi_master_event_handler);
    APP_ERROR_CHECK(err_code);

}

/**@brief The function initializes TX buffer to values to be sent and clears RX buffer.
 *
 * @note Function initializes TX buffer to values from 'A' to ('A' + len - 1)
 *       and clears RX buffer (fill by 0).
 *
 * @param[in] p_tx_data     A pointer to a buffer TX.
 * @param[in] p_rx_data     A pointer to a buffer RX.
 * @param[in] len           A length of the data buffers.
 */
static void init_buffers(uint8_t * const p_tx_data, uint8_t * const p_rx_data, uint8_t wave, const uint16_t  len)
{
    uint16_t data = 0;
    uint16_t i;
    switch(wave) {
	case 0: // triangle wave
	    data = triangleWave[len];
	    break;
	case 1: // reverse triangle wave
	    data = rTriangleWave[len];
	    break;
    }
    uint16_t data2 = (0x3000 | (0x0FFF & data)) >> 8; // Add config bits
    uint8_t datarray[TX_RX_BUF_LENGTH] = {data2, data};

    for (i = 0; i < TX_RX_BUF_LENGTH; i++)
    {

	p_tx_data[i] = datarray[i];
        p_rx_data[i] = 0;
    }
}



/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code = NRF_SUCCESS;

    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, NULL);
        
    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

void sendData(uint16_t index){
    init_buffers(m_tx_data, m_rx_data, 0, index);

    uint32_t err_code = spi_master_send_recv(SPI_MASTER_0, m_tx_data, TX_RX_BUF_LENGTH, m_rx_data, TX_RX_BUF_LENGTH);
    APP_ERROR_CHECK(err_code);

    while(m_transfer_completed == false); // Wait for transmission
    m_transfer_completed = false;

    init_buffers(m_tx_data, m_rx_data, 1, index);

    err_code = spi_master_send_recv(SPI_MASTER_1, m_tx_data, TX_RX_BUF_LENGTH, m_rx_data, TX_RX_BUF_LENGTH);
    APP_ERROR_CHECK(err_code);

    while(m_transfer_completed == false); // Wait for transmission
    m_transfer_completed = false;

    nrf_delay_us(delay_us);

}


/**@brief Function for application main entry. Does not return. */
int main(void)
{
    //sineInit();
    triangleInit();
    rTriangleInit();
    // Setup bsp module.
    bsp_configuration();

    uint16_t i = 0;

    spi_master_init();

    while (true)
    {

	for(i = 0; i < STORED*4; i++) {
	    sendData(i);
	};


    }

}


/** @} */
