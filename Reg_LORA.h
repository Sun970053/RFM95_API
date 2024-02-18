/*
 * Reg_LORA.h
 *
 *  Created on: Feb 9, 2024
 *      Author: e1406
 */

#ifndef INC_REG_LORA_H_
#define INC_REG_LORA_H_

// LoRa Page Registers
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_RX_HEADER_CNT_MSB    0x14
#define REG_RX_HEADER_CNT_LSB    0x15
#define REG_RX_PACKET_CNT_MSB    0x16
#define REG_RX_PACKET_CNT_LSB    0x17
#define REG_MODEM_STAT           0x18
#define REG_PCKT_SNR_VAL         0x19
#define REG_PCKT_RSSI_VAL        0x1A
#define REG_RSSI_VAL             0x1B
#define REG_HOP_CHANNEL          0x1C
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_SYMB_TIMEOUT_LSB     0x1F
#define REG_LORA_PREAMBLE_MSB    0x20
#define REG_LORA_PREAMBLE_LSB    0x21
#define REG_LORA_PAYLOAD_LEN     0x22
#define REG_MAX_PAYLOAD_LEN      0x23
#define REG_HOP_PERIOD           0x24
#define REG_FIFO_RX_BYTE_ADDR    0x25
#define REG_MODEM_CONFIG_3       0x26
#define REG_FEI_MSB              0x28
#define REG_FEI_MID              0x29
#define REG_FEI_LSB              0x2A
#define REG_RSSI_WIDEBAND        0x2C
#define REG_DETECT_OPTIMIZE      0x31
#define REG_INVERT_IQ            0x33
#define REG_DETECTION_THRES      0x37
#define REG_SYNCWORD             0x39

/*
 * #define REG_OP_MODE              0x01
 * The operating modes of the LoRaTM modem are accessed by 
 * enabling LoRaTM mode.
 */
typedef enum
{
    SLEEP_MODE,
    STDBY_MODE,
    FSTX,
    TX_MODE,
    FSRX,
    RX_CONTINUOUS,
    RX_SINGLE,
    CAD
}lora_dev_mode;

/*
 * #define REG_IRQ_FLAGS_MASK       0x11
 * In the register RegIrqFlagsMask, setting a bit to ‘1’ 
 * will mask the interrupt, meaning this interrupt is 
 * disactivated.
 * 
 * #define REG_IRQ_FLAGS            0x12
 * In the register RegIrqFlags, a ‘1’ indicates a given IRQ 
 * has been trigged and then the IRQ must be clear by 
 * writing a ‘1’.
 */
typedef enum
{
    Irq_RxTimeout =         0x7F,
    Irq_RxDone =            0xBF,
    Irq_PayloadCrcError =   0xDF,
    Irq_ValidHeader =       0xEF,
    Irq_TxDone =            0xF7,
    Irq_CadDone =           0xFB,
    Irq_FhssChangeChannel = 0xFD,
    Irq_CadDetected =       0xFE
}lora_irq_flags;

/*
 * #define REG_MODEM_CONFIG_1       0x1D
 * Singal Bandwidth
 */
typedef enum
{
    BW_7_8_kHz =        0x00,
    BW_10_4_kHz =       0x10,
    BW_15_6_kHz =       0x20,
    BW_20_8_kHz =       0x30,
    BW_31_25_kHz =      0x40,
    BW_41_7_kHz =       0x50,
    BW_62_5_kHz =       0x60,
    BW_125_kHz =        0x70,
    BW_250_kHz =        0x80,
    BW_500_kHz =        0x90
}lora_bw;

/*
 * #define REG_MODEM_CONFIG_1       0x1D
 * Error coding rate
 */
typedef enum
{
    CR_4_5 =            (1 << 1),
    CR_4_6 =            (2 << 1),
    CR_4_7 =            (3 << 1),
    CR_4_8 =            (4 << 1)
}lora_coding_rate;

/*
 * #define REG_MODEM_CONFIG_1       0x1D
 * Implicit header mode on
 */
#define EXPLICIT_HEADER_MODE ((bool)0)
#define IMPLICIT_HEADER_MODE ((bool)1)

/*
 * #define REG_MODEM_CONFIG_2       0x1E
 * Spreading Factor value: Corresponds to the data rate
 */
typedef enum
{
    SF_6 =              0x60,
    SF_7 =              0x70,
    SF_8 =              0x80,
    SF_9 =              0x90,
    SF_10 =             0xA0,
    SF_11 =             0xB0,
    SF_12 =             0xC0
}lora_sf;

/*
 * #define REG_MODEM_CONFIG_2       0x1E
 * Tx packets send
 */
#define TX_CONTINUOUS_OFF 0
#define TX_CONTINUOUS_ON  (1 << 3)

/*
 * #define REG_MODEM_CONFIG_2       0x1E
 * Rx payload crc on
 */
#define RX_PAYLOARD_CRC_DISABLE 0
#define RX_PAYLOARD_CRC_ENABLE  (1 << 2)

#endif /* INC_REG_LORA_H_ */
