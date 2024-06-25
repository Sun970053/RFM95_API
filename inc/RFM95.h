/*
 * RFM95.h
 *
 *  Created on: Feb 8, 2024
 *      Author: e1406
 */

#ifndef INC_RFM95_H_
#define INC_RFM95_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Reg_FSK.h"
#include "Reg_LORA.h"


// FIFO
#define REG_FIFO                 0x00

// Comman Register Settings
#define REG_OP_MODE              0x01
#define REG_FREQ_MSB             0x06
#define REG_FREQ_MID             0x07
#define REG_FREQ_LSB             0x08

// Registers for the Transmitter
#define REG_PA_CONFIG            0x09
#define REG_PA_RAMP              0x0A
#define REG_OCP                  0x0B

// Registers for the Receiver
#define REG_LNA                  0x0C

// IO control registers
#define REG_DIO_MAPPING_1        0x40
#define REG_DIO_MAPPING_2        0x41

// Version register
#define REG_VERSION              0x42

// Additional registers
#define REG_TCXO                 0x4B
#define REG_PA_DAC               0x4D
#define REG_FORMER_TEMP          0x5B
#define REG_AGC_REF              0x61
#define REG_AGC_THRESH_1         0x62
#define REG_AGC_THRESH_2         0x63
#define REG_AGC_THRESH_3         0x64
#define REG_PILL                 0x70

// RFM95 hardware define
#define RFM95_TCXO               32000000

// MCU SPI communication operation timeout
#define SPI_TIMEOUT              2000
// Signal transmit timeout
#define TRANSMIT_TIMEOUT         4000
// Only available in RX single mode
#define RECEIVE_TIMEOUT          2000

/* 
 * #define REG_OP_MODE              0x01
 * Radio driver supported modems
 */
typedef enum
{
    MODEM_FSK,
    MODEM_LORA
}radio_modem;

//#define REG_LNA                  0x0C
typedef enum
{
    LNA_G1 =                    0x20,
    LNA_G2 =                    0x40,
    LNA_G3 =                    0x60,
    LNA_G4 =                    0x80,
    LNA_G5 =                    0xA0,
    LNA_G6 =                    0xC0
}lna_gain_setting;

/* 
 * IO control registers
 */
#define DIOx_MAPPING_00         0b00
#define DIOx_MAPPING_01         0b01
#define DIOx_MAPPING_10         0b10
#define DIOx_MAPPING_11         0b11

//#define REG_PA_DAC               0x4D
#define PA_20DBM_OFF            0x04
#define PA_20DBM_ON             0x07

/*
 * RFM-95 FSK/OOK parameter setting
 */
typedef struct
{
    uint32_t                frequency;
    uint8_t                 bw;
}FSK_Param_Setting_t;

/*
 * RFM-95 LoRa parameter setting
 */
typedef struct
{
    uint32_t                frequency;
    lora_sf                 spreadingFactor;
    lora_bw                 bandWidth;
    lora_coding_rate        codingRate;
    uint8_t                 txPower;
    uint8_t                 overCurrentProtection;
    bool                    lowDatarateOptimize;
    bool                    isRxContinous;
}LoRa_Param_Setting_t;

typedef struct
{
    uint16_t                preamble;
    /* For Tx & Rx side in Implicit Header mode, turn on CRC function. */
    /* For Tx side in Explicit Header mode, turn on CRC function. */
    bool                    rxCrcOn;
    /* CrcOnPayload is for RX side to check whether received packet header crc is correct. */
    /* The user should then check the Irq flag PayloadCrcError */
    /* Explicit header mode only */
    bool                    crcOnPayload;
    bool                    implicitHeaderModeOn;
    uint8_t                 pcktLength;
    uint16_t                rxSymbTimeout;
    float                   SNR;
    int16_t                 pcktRSSI;
    int16_t                 currentRSSI;
}LoRa_Pckt_Handler_t;

/*
 * GPIO status handler
 * Please document the configuration registers for 
 * each Digital Input/Output (DIO) pin and provide the 
 * current status, distinguishing between low and high states,
 * for each of these pins. 
 */
typedef struct
{
    bool                    (*DIO0)();
    bool                    (*DIO1)();
    bool                    (*DIO2)();
    bool                    (*DIO3)();
    bool                    (*DIO4)();
    bool                    (*DIO5)();
    uint8_t                 DioStatus[6];
}RFM95_DIO_Handler_t;

/*
 * Radio Settings
 */
typedef struct
{
    lora_dev_mode           LoraState;
    fsk_trx_mode            FskState;
    LoRa_Param_Setting_t    LoRa;
    LoRa_Pckt_Handler_t     LoRaPcktHandler;
    FSK_Param_Setting_t     FSK;
    lna_gain_setting        LNA;
    radio_modem             Modem;
}RFM95_Settings_t;

/*
 * Function definition between MCU and RFM-95 and radio hardware
 * They need to be initalized before we can use this API.
 */
typedef struct
{
    uint8_t                 (*SPI_Write)(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout);
    uint8_t                 (*SPI_Read)(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
    uint8_t                 (*SPI_WriteRead)(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
    uint8_t                 (*SPI_CheckState)(void);
    uint8_t                 spi_ok;
    void                    (*DelayUs)(uint32_t delay);
    void                    (*NSEL)(bool val);
    void                    (*NRST)(bool val);
    RFM95_DIO_Handler_t     DIO;
    RFM95_Settings_t        Settings;
}RFM95_t;

/*
 * API error codes
 */
typedef enum
{
    RFM95_OK =                    0,
    RFM95_ERR_INVALID_INPUT =     1,
    RFM95_ERR_READ_REG =          10,
    RFM95_ERR_WRITE_REG =         11,
    RFM95_ERR_BURST_READ_REG =    20,
    RFM95_ERR_BURST_WRITE_REG =   21,
    RFM95_ERR_TX_TIMEOUT =        30,
    RFM95_ERR_RX_TIMEOUT =        40,
    RFM95_ERR_RX_PAYLOAD_CRC =    41,
    RFM95_ERR_RX_INVALID_HEADER = 42,
    RFM95_ERR_RX_FAIL =           43,
	RFM95_ERR_FREQUENCY =         50,
    RFM95_ERR_CHIP_VERSION =      255
}error_t;

uint8_t RFM95_LoRa_Init(RFM95_t* rfm95);
void RFM95_Reset(RFM95_t* rfm95);
uint8_t RFM95_setModem(RFM95_t* rfm95, radio_modem modem);
uint8_t RFM95_LoRa_setOpMode(RFM95_t* rfm95, uint8_t mode);
uint8_t RFM95_setFrequency(RFM95_t* rfm95, uint32_t freq);
uint32_t RFM95_getFrequency(RFM95_t* rfm95);
uint8_t RFM95_setTXPower(RFM95_t* rfm95, uint8_t power);
uint8_t RFM95_setOCP(RFM95_t* rfm95, uint8_t current);
uint8_t RFM95_setLNAGain(RFM95_t* rfm95, lna_gain_setting gain);
uint8_t RFM95_LoRa_setSpreadingFactor(RFM95_t* rfm95, uint8_t sf);
uint8_t RFM95_LoRa_setRxPayloadCrcOn(RFM95_t* rfm95, bool val);
uint8_t RFM95_LoRa_setBandwidth(RFM95_t* rfm95, lora_bw bw);
uint8_t RFM95_LoRa_setCodingRate(RFM95_t* rfm95, lora_coding_rate cr);
uint8_t RFM95_LoRa_setHeaderMode(RFM95_t* rfm95, bool val);
uint8_t RFM95_LoRa_setPreamble(RFM95_t* rfm95, uint16_t preamblelen);
uint8_t RFM95_LoRa_setLowDataRateOpt(RFM95_t* rfm95, bool val);
uint8_t RFM95_checkVersion(RFM95_t* rfm95);
uint8_t RFM95_LoRa_transmit(RFM95_t* rfm95, uint8_t* buffer, uint8_t bufferlen);

/// @brief The user can choose which mode they want to use.
/// @param rfm95 
/// @param isRxContinuous 
/// @return ERROR code
uint8_t RFM95_LoRa_prepareReceive(RFM95_t* rfm95, bool isRxContinuous);

/// @brief Please place it after detecting that the DIO0 pin status is pull-up.
/// @param rfm95 
/// @param buffer 
/// @param bufferlen 
/// @return ERROR code
uint8_t RFM95_LoRa_receive(RFM95_t* rfm95, uint8_t* buffer, uint8_t bufferlen);

/// @brief LoRa Sync Word
///        Value 0x34 is reserved for LoRaWAN networks
/// @param rfm95
/// @param syncword
/// @return ERROR code
uint8_t RFM95_LoRa_setSyncWord(RFM95_t* rfm95, uint8_t syncword);

/// @brief Estimation of SNR on last packet received.
/// @param rfm95
/// @return Signal Noise Ratio (SNR) value
float RFM95_getPcktSNR(RFM95_t* rfm95);

/// @brief PacketRssi (in RegPktRssiValue), is an averaged version
///        of Rssi (in RegRssiValue).
///
/// @param rfm95
/// @return Received Signal Strength Indication (RSSI) value
int16_t RFM95_getPcktRSSI(RFM95_t* rfm95);

/// @brief Rssi can be read at any time (during packet reception or not),
///        and should be averaged to give more precise results.
/// @param rfm95
/// @return Received Signal Strength Indication (RSSI) value
int16_t RFM95_getCurrentRSSI(RFM95_t* rfm95);

uint32_t RFM95_LoRa_random(RFM95_t* rfm95);

#endif /* INC_RFM95_H_ */
