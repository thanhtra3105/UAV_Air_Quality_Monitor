#ifndef MCP2515_H
#define MCP2515_H

#include "stm32f1xx_hal.h"

/* =========================================
 * MCP2515 Register Map
 * ========================================= */
#define MCP_RXF0SIDH    0x00
#define MCP_RXF0SIDL    0x01
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCP_CANINTE     0x2B
#define MCP_CANINTF     0x2C
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB0SIDL    0x62
#define MCP_RXB0DLC     0x65
#define MCP_RXB0D0      0x66
#define MCP_CANCTRL     0x0F
#define MCP_CANSTAT     0x0E

/* TX Buffer 0 Registers */
#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31
#define MCP_TXB0SIDL    0x32
#define MCP_TXB0DLC     0x35
#define MCP_TXB0D0      0x36

/* =========================================
 * MCP2515 SPI Commands
 * ========================================= */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BITMOD      0x05
#define MCP_RTS_TXB0    0x81   /* Request To Send TX Buffer 0 */
#define MCP_READ_RX0    0x90

/* =========================================
 * Operating Modes — CANCTRL[7:5]
 * ========================================= */
#define MCP_MODE_NORMAL     0x00
#define MCP_MODE_SLEEP      0x20
#define MCP_MODE_LOOPBACK   0x40
#define MCP_MODE_LISTENONLY 0x60
#define MCP_MODE_CONFIG     0x80

/* =========================================
 * Interrupt / Status Flags
 * ========================================= */
#define MCP_RX0IF       0x01
#define MCP_TX0IF       0x04

/* TXB0CTRL bits */
#define MCP_TXB_TXREQ   0x08   /* bit3: 1 = dang gui, 0 = da xong */
#define MCP_TXB_TXERR   0x10   /* bit4: loi khi gui               */
#define MCP_TXB_MLOA    0x20   /* bit5: mat arbitration            */

/* =========================================
 * Return codes cho TransmitMsg
 * ========================================= */
#define MCP_TX_OK       1
#define MCP_TX_TIMEOUT  0

/* =========================================
 * API Functions
 * ========================================= */
void    MCP2515_Reset(void);
uint8_t MCP2515_ReadReg(uint8_t reg);
void    MCP2515_WriteReg(uint8_t reg, uint8_t val);
void    MCP2515_BitModify(uint8_t reg, uint8_t mask, uint8_t val);
void    MCP2515_SetMode(uint8_t mode);
uint8_t MCP2515_Init(void);
uint8_t MCP2515_TransmitMsg(uint16_t id, uint8_t len, uint8_t *data);
uint8_t MCP2515_ReceiveMsg(uint16_t *id, uint8_t *len, uint8_t *data);

#endif /* MCP2515_H */