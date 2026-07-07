
/*
 * nrf.h
 *
 * Driver NRF24L01 cho STM32 HAL
 * RX Mode
 */

#ifndef __NRF24_H
#define __NRF24_H

#include "stm32h5xx_hal.h"

/*==========================================================
                        SPI HANDLE
==========================================================*/



/*==========================================================
                        GPIO
==========================================================*/

#define NRF_CSN_PORT       GPIOB
#define NRF_CSN_PIN        GPIO_PIN_5

#define NRF_CE_PORT        GPIOC
#define NRF_CE_PIN         GPIO_PIN_1

#define NRF_CSN_LOW()      HAL_GPIO_WritePin(NRF_CSN_PORT,NRF_CSN_PIN,GPIO_PIN_RESET)
#define NRF_CSN_HIGH()     HAL_GPIO_WritePin(NRF_CSN_PORT,NRF_CSN_PIN,GPIO_PIN_SET)

#define NRF_CE_LOW()       HAL_GPIO_WritePin(NRF_CE_PORT,NRF_CE_PIN,GPIO_PIN_RESET)
#define NRF_CE_HIGH()      HAL_GPIO_WritePin(NRF_CE_PORT,NRF_CE_PIN,GPIO_PIN_SET)

/*==========================================================
                        COMMAND
==========================================================*/

#define R_REGISTER         0x00
#define W_REGISTER         0x20

#define R_RX_PAYLOAD       0x61
#define W_TX_PAYLOAD       0xA0

#define FLUSH_TX           0xE1
#define FLUSH_RX           0xE2

#define R_RX_PL_WID        0x60

#define ACTIVATE           0x50

#define NOP                0xFF

/*==========================================================
                        REGISTER
==========================================================*/

#define CONFIG             0x00
#define EN_AA              0x01
#define EN_RXADDR          0x02
#define SETUP_AW           0x03
#define SETUP_RETR         0x04
#define RF_CH              0x05
#define RF_SETUP           0x06
#define STATUS             0x07

#define RX_ADDR_P0         0x0A
#define RX_ADDR_P1         0x0B

#define TX_ADDR            0x10

#define RX_PW_P0           0x11

#define FIFO_STATUS        0x17

#define DYNPD              0x1C
#define FEATURE            0x1D

/*==========================================================
                        STATUS BIT
==========================================================*/

#define RX_DR              6
#define TX_DS              5
#define MAX_RT             4

/*==========================================================
                        API
==========================================================*/

void NRF24_Init(void);

void NRF24_StartListening(void);

void NRF24_StopListening(void);

void NRF24_OpenReadingPipe(uint8_t pipe,uint8_t *addr);

uint8_t NRF24_ReadReg(uint8_t reg);

void NRF24_WriteReg(uint8_t reg,uint8_t value);

void NRF24_ReadBuf(uint8_t reg,uint8_t *buf,uint8_t len);

void NRF24_WriteBuf(uint8_t reg,uint8_t *buf,uint8_t len);

uint8_t NRF24_GetStatus(void);

uint8_t NRF24_Available(void);

uint8_t NRF24_GetDynamicPayloadSize(void);

void NRF24_Read(uint8_t *buf,uint8_t len);

void NRF24_FlushRX(void);

void NRF24_FlushTX(void);

#endif

