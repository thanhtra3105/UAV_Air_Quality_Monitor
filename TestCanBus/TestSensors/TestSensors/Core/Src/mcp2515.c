#include "mcp2515.h"

/* =========================================
 * SPI handle — extern tu main.c
 * ========================================= */
extern SPI_HandleTypeDef hspi1;

/* CS Pin: PA4 */
#define MCP_CS_PORT     GPIOA
#define MCP_CS_PIN      GPIO_PIN_4

#define CS_LOW()    HAL_GPIO_WritePin(MCP_CS_PORT, MCP_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(MCP_CS_PORT, MCP_CS_PIN, GPIO_PIN_SET)

/* =========================================
 * Internal: SPI transfer 1 byte
 * ========================================= */
static uint8_t SPI_Transfer(uint8_t txData)
{
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, HAL_MAX_DELAY);
    return rxData;
}

/* =========================================
 * Reset
 * ========================================= */
void MCP2515_Reset(void)
{
    CS_LOW();
    SPI_Transfer(MCP_RESET);
    CS_HIGH();
    HAL_Delay(10);
}

/* =========================================
 * Doc 1 thanh ghi
 * ========================================= */
uint8_t MCP2515_ReadReg(uint8_t reg)
{
    uint8_t val;
    CS_LOW();
    SPI_Transfer(MCP_READ);
    SPI_Transfer(reg);
    val = SPI_Transfer(0x00);
    CS_HIGH();
    return val;
}

/* =========================================
 * Ghi 1 thanh ghi
 * ========================================= */
void MCP2515_WriteReg(uint8_t reg, uint8_t val)
{
    CS_LOW();
    SPI_Transfer(MCP_WRITE);
    SPI_Transfer(reg);
    SPI_Transfer(val);
    CS_HIGH();
}

/* =========================================
 * Bit Modify
 * ========================================= */
void MCP2515_BitModify(uint8_t reg, uint8_t mask, uint8_t val)
{
    CS_LOW();
    SPI_Transfer(MCP_BITMOD);
    SPI_Transfer(reg);
    SPI_Transfer(mask);
    SPI_Transfer(val);
    CS_HIGH();
}

/* =========================================
 * Chuyen Operating Mode
 * ========================================= */
void MCP2515_SetMode(uint8_t mode)
{
    MCP2515_BitModify(MCP_CANCTRL, 0xE0, mode);
    HAL_Delay(10);
}

/* =========================================
 * Khoi tao MCP2515
 * 500 kbps @ 8 MHz crystal
 *
 * Bit timing (8 TQ total):
 *   TQ        = 2*(BRP+1)/Fosc = 2*1/8MHz = 250ns
 *   SYNC      = 1 TQ  (co dinh)
 *   PROP      = 1 TQ  (PRSEG = 0 → 0+1 = 1)
 *   PS1       = 3 TQ  (PHSEG1 = 2 → 2+1 = 3)
 *   PS2       = 3 TQ  (PHSEG2 = 2 → 2+1 = 3)
 *   Tbit      = 8 TQ * 250ns = 2000ns = 500 kbps ✓
 * ========================================= */
uint8_t MCP2515_Init(void)
{
    MCP2515_Reset();

    uint8_t mode = MCP2515_ReadReg(MCP_CANSTAT) & 0xE0;
    if (mode != MCP_MODE_CONFIG)
        return 0;

    MCP2515_WriteReg(MCP_CNF1, 0x00);  /* BRP=0, SJW=1TQ          */
    MCP2515_WriteReg(MCP_CNF2, 0x90);  /* BTLMODE=1, PS1=3TQ, PROP=1TQ */
    MCP2515_WriteReg(MCP_CNF3, 0x02);  /* PS2=3TQ                  */

    MCP2515_WriteReg(MCP_RXB0CTRL, 0x60); /* Nhan tat ca, tat filter */
    MCP2515_WriteReg(MCP_CANINTE,  0x00); /* Tat interrupt, dung polling */

    MCP2515_SetMode(MCP_MODE_NORMAL);

    mode = MCP2515_ReadReg(MCP_CANSTAT) & 0xE0;
    if (mode != MCP_MODE_NORMAL)
        return 0;

    return 1;
}

/* =========================================
 * Gui 1 CAN Frame qua TX Buffer 0
 *
 * id  : Standard CAN ID, 11-bit (0x000 - 0x7FF)
 * len : So byte data, toi da 8
 * data: Con tro den mang data
 *
 * Tra ve MCP_TX_OK (1) neu gui thanh cong
 * Tra ve MCP_TX_TIMEOUT (0) neu timeout sau 10ms
 *
 * Cach ghi CAN ID vao TXB0SIDH/SIDL:
 *   SIDH[7:0] = ID[10:3]
 *   SIDL[7:5] = ID[2:0], SIDL[3]=0 (Standard frame)
 * ========================================= */
uint8_t MCP2515_TransmitMsg(uint16_t id, uint8_t len, uint8_t *data)
{
    /* Gioi han len toi da 8 byte */
    if (len > 8) len = 8;

    /* Ghi CAN ID vao TXB0 */
    MCP2515_WriteReg(MCP_TXB0SIDH, (uint8_t)(id >> 3));
    MCP2515_WriteReg(MCP_TXB0SIDL, (uint8_t)(id << 5));

    /* Ghi DLC — data frame, khong phai remote frame */
    MCP2515_WriteReg(MCP_TXB0DLC, len & 0x0F);

    /* Ghi tung byte data */
    for (uint8_t i = 0; i < len; i++)
    {
        MCP2515_WriteReg(MCP_TXB0D0 + i, data[i]);
    }

    /* Xoa TXERR va MLOA truoc khi gui */
    MCP2515_BitModify(MCP_TXB0CTRL, MCP_TXB_TXERR | MCP_TXB_MLOA, 0x00);

    /* Ra lenh Request To Send TX Buffer 0 */
    CS_LOW();
    SPI_Transfer(MCP_RTS_TXB0);
    CS_HIGH();

    /* Cho den khi TXREQ bit = 0 (da gui xong) hoac timeout 10ms */
    uint8_t  timeout = 100;   /* 100 * 100us = 10ms */
    uint8_t  ctrl;

    while (timeout--)
    {
        ctrl = MCP2515_ReadReg(MCP_TXB0CTRL);

        if (!(ctrl & MCP_TXB_TXREQ))
            return MCP_TX_OK;   /* TXREQ = 0: gui xong */

        HAL_Delay(1);
    }

    /* Timeout: huy lenh gui */
    MCP2515_BitModify(MCP_TXB0CTRL, MCP_TXB_TXREQ, 0x00);
    return MCP_TX_TIMEOUT;
}

/* =========================================
 * Polling nhan CAN message tu RXB0
 * Tra ve 1 neu co message, 0 neu khong co
 * ========================================= */
uint8_t MCP2515_ReceiveMsg(uint16_t *id, uint8_t *len, uint8_t *data)
{
    uint8_t intf = MCP2515_ReadReg(MCP_CANINTF);
    if (!(intf & MCP_RX0IF))
        return 0;

    uint8_t sidh = MCP2515_ReadReg(MCP_RXB0SIDH);
    uint8_t sidl = MCP2515_ReadReg(MCP_RXB0SIDL);
    *id = ((uint16_t)sidh << 3) | ((sidl >> 5) & 0x07);

    *len = MCP2515_ReadReg(MCP_RXB0DLC) & 0x0F;
    if (*len > 8) *len = 8;

    for (uint8_t i = 0; i < *len; i++)
        data[i] = MCP2515_ReadReg(MCP_RXB0D0 + i);

    MCP2515_BitModify(MCP_CANINTF, MCP_RX0IF, 0x00);
    return 1;
}