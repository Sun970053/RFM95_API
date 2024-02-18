/*
 * Reg_FSK.h
 *
 *  Created on: Feb 9, 2024
 *      Author: e1406
 */

#ifndef INC_REG_FSK_H_
#define INC_REG_FSK_H_

// FSK/OOK Registers
#define REG_BITRATE_MSB          0x02
#define REG_BITRATE_LSB          0x03
#define REG_FDEV_MSB             0x04
#define REG_FDEV_LSB             0x05

// Rx
#define REG_RX_CONFIG            0x0D
#define REG_RSSI_CONFIG          0x0E
#define REG_RSSI_COLLISION       0x0F
#define REG_RSSI_THRESH          0x10
#define REG_FSK_RSSI_VAL         0x11
#define REG_RX_BW                0x12
#define REG_AFC_BW               0x13
#define REG_OOK_PEAK             0x14
#define REG_OOK_FIX              0x15
#define REG_OOK_AVG              0x16
#define REG_AFC_FEI              0x1A
#define REG_AFC_MSB              0x1B
#define REG_AFC_LSB              0x1C
#define REG_FSK_FEI_MSB          0x1D
#define REG_FEi_LSB              0x1E
#define REG_PREAMBLE_DETECT      0x1F
#define REG_RX_TIMEOUT_1         0x20
#define REG_RX_TIMEOUT_2         0x21
#define REG_RX_TIMEOUT_3         0x22
#define REG_RX_DELAY             0x23

// RC Oscillator registers
#define REG_OSC                  0x24

// Packet Handling registers
#define REG_FSK_PREAMBLE_MSB     0x25
#define REG_FSK_PREAMBLE_LSB     0x26
#define REG_SYNC_CONFIG          0x27
#define REG_SYNC_VAL_1           0x28
#define REG_SYNC_VAL_2           0x29
#define REG_SYNC_VAL_3           0x2A
#define REG_SYNC_VAL_4           0x2B
#define REG_SYNC_VAL_5           0x2C
#define REG_SYNC_VAL_6           0x2D
#define REG_SYNC_VAL_7           0x2E
#define REG_SYNC_VAL_8           0x2F
#define REG_PCKT_CONFIG_1        0x30
#define REG_PCKT_CONFIG_2        0x31
#define REG_PAYLOAD_LEN          0x32
#define REG_NODE_ADRS            0x33
#define REG_BROADCAST_ADRS       0x34
#define REG_FIFO_THRESH          0x35

// Sequencer registers
#define REG_SEQ_CONFIG_1         0x36
#define REG_SEQ_CONFIG_2         0x37
#define REG_TIMER_RESOL          0x38
#define REG_TIMER_1_COEF         0x39
#define REG_TIMER_2_COEF         0x3A

// Service registers
#define REG_IMAGE_CAL            0x3B
#define REG_TEMP                 0x3C
#define REG_LOW_BAT              0x3D

// Status registers
#define REG_IRQ_FLAGS_1          0x3E
#define REG_IRQ_FLAGS_2          0x3F

// Additional registers
#define REG_PILL_HOP             0x44
#define REG_BITRATE_FRAC         0x5D

//#define REG_OP_MODE              0x01
typedef enum
{
    FSK_SLEEP_MODE,
	FSK_STDBY_MODE,
	FSK_FSTX_MODE,
	FSK_TX_MODE,
	FSK_FSRX_MODE,
	FSK_RX_MODE
}fsk_trx_mode;

#endif /* INC_REG_FSK_H_ */
