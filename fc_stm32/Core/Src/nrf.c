
/*
 * nrf.c
 *
 * Driver NRF24L01 cho STM32 HAL
 * RX Mode
 */

#include "nrf.h"

/*==========================================================
                    PRIVATE FUNCTION
==========================================================*/
extern SPI_HandleTypeDef hspi2;

static uint8_t SPI_RW(uint8_t data)
{
    uint8_t rx;

    HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, HAL_MAX_DELAY);

    return rx;
}

static void NRF24_Activate(void)
{
    uint8_t tx[2] = {ACTIVATE, 0x73};
    uint8_t rx[2];

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);

    NRF_CSN_HIGH();
}

/*==========================================================
                    REGISTER ACCESS
==========================================================*/

uint8_t NRF24_GetStatus(void)
{
    uint8_t tx = NOP;
    uint8_t rx;

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, HAL_MAX_DELAY);

    NRF_CSN_HIGH();

    return rx;
}

uint8_t NRF24_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = R_REGISTER | reg;
    tx[1] = NOP;

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);

    NRF_CSN_HIGH();

    return rx[1];
}

void NRF24_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = W_REGISTER | reg;
    tx[1] = value;

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);

    NRF_CSN_HIGH();
}
void NRF24_ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[33];
    uint8_t rx[33];

    tx[0] = R_REGISTER | reg;

    memset(&tx[1], NOP, len);

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2,
                            tx,
                            rx,
                            len + 1,
                            HAL_MAX_DELAY);

    NRF_CSN_HIGH();

    memcpy(buf, &rx[1], len);
}

void NRF24_WriteBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[33];
    uint8_t rx[33];

    tx[0] = W_REGISTER | reg;

    memcpy(&tx[1], buf, len);

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2,
                            tx,
                            rx,
                            len + 1,
                            HAL_MAX_DELAY);

    NRF_CSN_HIGH();
}

/*==========================================================
                        FIFO
==========================================================*/

void NRF24_FlushRX(void)
{
    uint8_t tx = FLUSH_RX;
    uint8_t rx;

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2,&tx,&rx,1,HAL_MAX_DELAY);

    NRF_CSN_HIGH();
}

void NRF24_FlushTX(void)
{
    uint8_t tx = FLUSH_TX;
    uint8_t rx;

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2,&tx,&rx,1,HAL_MAX_DELAY);

    NRF_CSN_HIGH();
}
/*==========================================================
                    CONFIGURATION
==========================================================*/

void NRF24_OpenReadingPipe(uint8_t pipe, uint8_t *addr)
{
    if(pipe > 5)
        return;

    if(pipe <= 1)
    {
        NRF24_WriteBuf(RX_ADDR_P0 + pipe, addr, 5);
    }
    else
    {
        NRF24_WriteReg(RX_ADDR_P0 + pipe, addr[0]);
    }

    if(pipe == 0)
    {
        NRF24_WriteReg(RX_PW_P0, 32);
    }

    uint8_t reg = NRF24_ReadReg(EN_RXADDR);

    reg |= (1 << pipe);

    NRF24_WriteReg(EN_RXADDR, reg);
}

void NRF24_StartListening(void)
{
    uint8_t config;

    config = NRF24_ReadReg(CONFIG);

    config |= (1 << 0);      // PRIM_RX
    config |= (1 << 1);      // PWR_UP

    NRF24_WriteReg(CONFIG, config);

    NRF24_WriteReg(STATUS, 0x70);

    NRF_CE_HIGH();

    HAL_Delay(2);
}

void NRF24_StopListening(void)
{
    NRF_CE_LOW();

    uint8_t config = NRF24_ReadReg(CONFIG);

    config &= ~(1 << 0);

    NRF24_WriteReg(CONFIG, config);

    HAL_Delay(2);
}

void NRF24_Init(void)
{
    NRF_CE_LOW();

    HAL_Delay(5);

    NRF24_Activate();

    NRF24_WriteReg(CONFIG,0x0E);

    NRF24_WriteReg(EN_AA,0x01);

    NRF24_WriteReg(EN_RXADDR,0x01);

    NRF24_WriteReg(SETUP_AW,0x03);

    NRF24_WriteReg(SETUP_RETR,0x2F);

    NRF24_WriteReg(RF_CH,76);

    NRF24_WriteReg(RF_SETUP,0x26);

    NRF24_WriteReg(FEATURE,0x06);

    NRF24_WriteReg(DYNPD,0x01);

    NRF24_WriteReg(STATUS,0x70);

    NRF24_FlushRX();

    NRF24_FlushTX();
}

/*==========================================================
                    RECEIVE
==========================================================*/

uint8_t NRF24_Available(void)
{
    uint8_t status;

    status = NRF24_ReadReg(STATUS);

    return (status & (1 << RX_DR));
}

uint8_t NRF24_GetDynamicPayloadSize(void)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = R_RX_PL_WID;
    tx[1] = NOP;

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2,
                            tx,
                            rx,
                            2,
                            HAL_MAX_DELAY);

    NRF_CSN_HIGH();

    if(rx[1] > 32)
    {
        NRF24_FlushRX();
        return 0;
    }

    return rx[1];
}

void NRF24_Read(uint8_t *buf, uint8_t len)
{
    uint8_t tx[33];
    uint8_t rx[33];

    tx[0] = R_RX_PAYLOAD;

    memset(&tx[1], NOP, len);

    NRF_CSN_LOW();

    HAL_SPI_TransmitReceive(&hspi2,
                            tx,
                            rx,
                            len + 1,
                            HAL_MAX_DELAY);

    NRF_CSN_HIGH();

    memcpy(buf, &rx[1], len);

    NRF24_WriteReg(STATUS, (1 << RX_DR));
}

