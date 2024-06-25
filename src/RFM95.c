/*
 * RFM95.c
 *
 *  Created on: Feb 8, 2024
 *      Author: e1406
 */
#include "RFM95.h"

// single access
uint8_t rfm95_readReg(RFM95_t* rfm95, uint8_t reg, uint8_t* readVal);
uint8_t rfm95_writeReg(RFM95_t* rfm95, uint8_t reg, uint8_t* writeVal);
// burst access
uint8_t rfm95_burst_readReg(RFM95_t* rfm95, uint8_t reg, uint8_t* readVal, uint8_t length);
uint8_t rfm95_burst_writeReg(RFM95_t* rfm95, uint8_t reg, uint8_t* writeVal, uint8_t length);
// fifo access
uint8_t rfm95_fifo_readReg(RFM95_t* rfm95, uint8_t* readVal, uint8_t length);
uint8_t rfm95_fifo_writeReg(RFM95_t* rfm95, uint8_t* writeVal, uint8_t length);

// setting rx operation timeout: TimeOut = SymbTimeout ⋅ Ts
uint8_t rfm95_lora_symb_timout(RFM95_t* rfm95, uint16_t value);
/* 
 * #define REG_DIO_MAPPING_1        0x40
 * #define REG_DIO_MAPPING_2        0x41
 * IO control registers
 * Setting DIO Mapping LoRa Mode
 */
uint8_t rfm95_lora_dio_mapping(RFM95_t* rfm95, uint8_t dioNB, uint8_t dioStatus);

void RFM95_Reset(RFM95_t* rfm95)
{
    rfm95->NRST(0);
    // delay 1 ms
    rfm95->DelayUs(1000);
    rfm95->NRST(1);
    // delay 10 ms
    rfm95->DelayUs(10000);
}

uint8_t RFM95_LoRa_setOpMode(RFM95_t* rfm95, uint8_t mode)
{
    uint8_t readVal;
    if(rfm95_readReg(rfm95, REG_OP_MODE, &readVal)) 
        return RFM95_ERR_READ_REG;
    switch(mode)
    {
        case SLEEP_MODE:
            break;
        case STDBY_MODE:
            break;
        case FSTX:
            break;
        case TX_MODE:
            break;
        case FSRX:
            break;
        case RX_CONTINUOUS:
            break;
        case RX_SINGLE:
            break;
        case CAD:
            break;
        default:
            return RFM95_ERR_INVALID_INPUT;
    }
    readVal = (readVal & 0xF8) | mode;
    if(rfm95_writeReg(rfm95, REG_OP_MODE, &readVal))
        return RFM95_ERR_WRITE_REG;
    rfm95->Settings.LoraState = mode;
    return RFM95_OK;
}

uint8_t RFM95_setModem(RFM95_t* rfm95, radio_modem modem)
{
    uint8_t readVal;
    if(rfm95_readReg(rfm95, REG_OP_MODE, &readVal))
        return RFM95_ERR_READ_REG;
    switch (modem)
    {
    case MODEM_LORA:
        readVal = (readVal & 0x7F) | (MODEM_LORA << 7);
        break;
    case MODEM_FSK:
        readVal = (readVal & 0x7F) | (MODEM_FSK << 7);
        break;
    default:
        return RFM95_ERR_INVALID_INPUT;
    }
    if(rfm95_writeReg(rfm95, REG_OP_MODE, &readVal) )
        return RFM95_ERR_WRITE_REG;
    rfm95->Settings.Modem = modem;
    return RFM95_OK;
}

uint8_t RFM95_setFrequency(RFM95_t* rfm95, uint32_t freq)
{
    uint8_t writeVal[3];
    uint32_t fr = (uint32_t)(freq * 0.016384f);
    writeVal[0] = (uint8_t)((fr >> 16) & 0xFF);
    writeVal[1] = (uint8_t)((fr >> 8) & 0xFF);
    writeVal[2] = (uint8_t)(fr & 0xFF);
    if(rfm95_burst_writeReg(rfm95, REG_FREQ_MSB, &writeVal[0], 3))
        return RFM95_ERR_WRITE_REG;
    rfm95->Settings.LoRa.frequency = freq;
    return RFM95_OK;
}

uint32_t RFM95_getFrequency(RFM95_t* rfm95)
{
	uint32_t readFreq;
	uint8_t readVal[3];
	rfm95_burst_readReg(rfm95, REG_FREQ_MSB, &readVal[0], 3);
	readFreq = (uint32_t)((readVal[0] << 16) | (readVal[1] << 8) | (readVal[0]));
	readFreq = (uint32_t)(readFreq/0.016384f);
	if((rfm95->Settings.LoRa.frequency - 100000) < readFreq &&
			(rfm95->Settings.LoRa.frequency + 100000) > readFreq)
	{
		return rfm95->Settings.LoRa.frequency;
	}
	else
	{
		return RFM95_ERR_FREQUENCY;
	}
}

uint8_t RFM95_setTXPower(RFM95_t* rfm95, uint8_t power)
{
    uint8_t writeVal[2];
    uint8_t readVal[2];
    if(power < 2 || power > 20) return RFM95_ERR_INVALID_INPUT;
    else if(power <= 20)
    {
        writeVal[0] = (uint8_t)(0xFF);
        if(rfm95_readReg(rfm95, REG_PA_DAC, &readVal[0]))
            return RFM95_ERR_READ_REG;
        // 0xF8: RegPaDac PaDac mask
        writeVal[1] = (uint8_t)((readVal[0] & 0xF8) | PA_20DBM_ON);
        // Raise the output power to maximum
        if(rfm95_writeReg(rfm95, REG_PA_CONFIG, &writeVal[0]) ||
        rfm95_writeReg(rfm95, REG_PA_DAC, &writeVal[1]))
            return RFM95_ERR_WRITE_REG;
    }
    else if(power <= 17)
    {
        // Control output power
        if(rfm95_readReg(rfm95, REG_PA_CONFIG, &readVal[0]) ||
        rfm95_readReg(rfm95, REG_PA_DAC, &readVal[1]))
            return RFM95_ERR_READ_REG;
        // Calculate PA_CONFIG and PA_DAC register value
        writeVal[0] = (uint8_t)((readVal[0] & 0x70) | (1 << 7) | (power - 2));
        writeVal[1] = (uint8_t)((readVal[1] & 0xF8) | PA_20DBM_OFF);
        if(rfm95_writeReg(rfm95, REG_PA_CONFIG, &writeVal[0]) ||
        rfm95_writeReg(rfm95, REG_PA_DAC, &writeVal[1]))
            return RFM95_ERR_WRITE_REG;
    }
    else if(power <= 14)
    {
    	// Control output power
		if(rfm95_readReg(rfm95, REG_PA_DAC, &readVal[1]))
			return RFM95_ERR_READ_REG;
		// Calculate value for high efficiency PA
		writeVal[0] = (uint8_t)(0x70 | (0 << 7) | (power - 1));
		writeVal[1] = (uint8_t)((readVal[1] & 0xF8) | PA_20DBM_OFF);
		if(rfm95_writeReg(rfm95, REG_PA_CONFIG, &writeVal[0]) ||
		rfm95_writeReg(rfm95, REG_PA_DAC, &writeVal[1]))
			return RFM95_ERR_WRITE_REG;
    }

    rfm95->Settings.LoRa.txPower = power;
    return RFM95_OK;
}

uint8_t RFM95_setOCP(RFM95_t* rfm95, uint8_t current)
{
    uint8_t writeVal;
    // Trimming of OCP current: mA
    if(current < 45 || current > 240) return RFM95_ERR_INVALID_INPUT;
    else if(current <= 120)
    {
        writeVal = (uint8_t)(((current - 45) / 5) | (1 << 5));
    }
    else if(current <= 240)
    {
        writeVal = (uint8_t)(((current - 30) / 10) | (1 << 5));
    }
    if(rfm95_writeReg(rfm95, REG_OCP, &writeVal))
        return RFM95_ERR_WRITE_REG;
    rfm95->Settings.LoRa.overCurrentProtection = current;
    return RFM95_OK;
}

uint8_t RFM95_setLNAGain(RFM95_t* rfm95, lna_gain_setting gain)
{
    switch(gain)
    {
        case LNA_G1:
        	break;
        case LNA_G2:
            break;
        case LNA_G3:
            break;
        case LNA_G4:
            break;
        case LNA_G5:
            break;
        case LNA_G6:
            break;
        default:
            return RFM95_ERR_INVALID_INPUT;
    }
    uint8_t readVal, writeVal;
    if(rfm95_readReg(rfm95, REG_LNA, &readVal))
        return RFM95_ERR_READ_REG;
    // LnaGain + LnaBoostHf boostOn (0x3). Their Mask: 0x1C
    writeVal = (uint8_t)((readVal & 0x1C) | gain | 0x3);
    if(rfm95_writeReg(rfm95, REG_LNA, &writeVal))
        return RFM95_ERR_WRITE_REG;

    rfm95->Settings.LNA = gain;
    return RFM95_OK;
}

uint8_t RFM95_LoRa_setSpreadingFactor(RFM95_t* rfm95, uint8_t sf)
{
    if(sf > 12 || sf < 6) return RFM95_ERR_INVALID_INPUT;

    uint8_t readVal, writeVal;
    if(rfm95_readReg(rfm95, REG_MODEM_CONFIG_2, &readVal))
        return RFM95_ERR_READ_REG;
    // Spreadinf Factor Mask: 0xF0
    writeVal = (uint8_t)((readVal & 0x0F) | (uint8_t)(sf << 4));
    if(rfm95_writeReg(rfm95, REG_MODEM_CONFIG_2, &writeVal))
        return RFM95_ERR_WRITE_REG;

    rfm95->Settings.LoRa.spreadingFactor = sf;
    return RFM95_OK;
}

uint8_t RFM95_LoRa_setRxPayloadCrcOn(RFM95_t* rfm95, bool val)
{
    uint8_t readVal, writeVal;
    if(rfm95_readReg(rfm95, REG_MODEM_CONFIG_2, &readVal))
        return RFM95_ERR_READ_REG;
    // Rx Payload CrcOn Mask: 0xFB
    writeVal = (uint8_t)((readVal & 0xFB) | (uint8_t)(val << 2));
    if(rfm95_writeReg(rfm95, REG_MODEM_CONFIG_2, &writeVal))
        return RFM95_ERR_WRITE_REG;
    
    rfm95->Settings.LoRaPcktHandler.rxCrcOn = val;
    return RFM95_OK;
}

uint8_t rfm95_lora_symb_timout(RFM95_t* rfm95, uint16_t value)
{
    if(value > 0x03FF) value = 0x03FF;
    uint8_t readVal;
    uint8_t writeVal[2];
    if(rfm95_readReg(rfm95, REG_MODEM_CONFIG_2, &readVal))
        return RFM95_ERR_READ_REG;

    // SymbTimeout MSB
    writeVal[0] = (uint8_t)((readVal & 0xFC) | (uint8_t)(value >> 8));
    // SymbTimeout LSB
    writeVal[1] = (uint8_t)(value & 0x00FF);
    if(rfm95_writeReg(rfm95, REG_MODEM_CONFIG_2, &writeVal[0]) ||
    rfm95_writeReg(rfm95, REG_SYMB_TIMEOUT_LSB, &writeVal[1]))
        return RFM95_ERR_WRITE_REG;
    
    rfm95->Settings.LoRaPcktHandler.rxSymbTimeout = value;
    return RFM95_OK;
}

uint8_t RFM95_LoRa_setBandwidth(RFM95_t* rfm95, lora_bw bw)
{
    switch(bw)
    {
    case BW_7_8_kHz:
        break;
    case BW_10_4_kHz:
        break;
    case BW_15_6_kHz:
        break;
    case BW_20_8_kHz:
        break;
    case BW_31_25_kHz:
        break;
    case BW_41_7_kHz:
        break;
    case BW_62_5_kHz:    
        break;
    case BW_125_kHz:
        break;
    case BW_250_kHz:
        break;
    case BW_500_kHz:
        break;
    default:
        return RFM95_ERR_INVALID_INPUT;
    }
    uint8_t readVal, writeVal;
    if(rfm95_readReg(rfm95, REG_MODEM_CONFIG_1, &readVal))
        return RFM95_ERR_READ_REG;
    // Bandwidth register value location
    writeVal = (uint8_t)((readVal & 0x0F) | bw);
    if(rfm95_writeReg(rfm95, REG_MODEM_CONFIG_1, &writeVal))
        return RFM95_ERR_WRITE_REG;
    
    rfm95->Settings.LoRa.bandWidth = bw;
    return RFM95_OK;
}

uint8_t RFM95_LoRa_setCodingRate(RFM95_t* rfm95, lora_coding_rate cr)
{
    switch (cr)
    {
    case CR_4_5:
        break;
    case CR_4_6:
        break;   
    case CR_4_7:
        break;
    case CR_4_8:
        break;            
    default:
        return RFM95_ERR_INVALID_INPUT;
    }
    uint8_t readVal, writeVal;
    if(rfm95_readReg(rfm95, REG_MODEM_CONFIG_1, &readVal))
        return RFM95_ERR_READ_REG;
    // Coding Rate register location
    writeVal = (uint8_t)((readVal & 0xF1) | cr);
    if(rfm95_writeReg(rfm95, REG_MODEM_CONFIG_1, &writeVal))
        return RFM95_ERR_WRITE_REG;

    rfm95->Settings.LoRa.codingRate = cr;
    return RFM95_OK;
}

uint8_t RFM95_LoRa_setHeaderMode(RFM95_t* rfm95, bool val)
{
    uint8_t readVal, writeVal;
    if(rfm95_readReg(rfm95, REG_MODEM_CONFIG_1, &readVal))
        return RFM95_ERR_READ_REG;
    // Header Mode Mask: 0xFE
    writeVal = (uint8_t)((readVal & 0xFE) | val);
    if(rfm95_writeReg(rfm95, REG_MODEM_CONFIG_1, &writeVal))
        return RFM95_ERR_WRITE_REG;

    rfm95->Settings.LoRaPcktHandler.implicitHeaderModeOn = val;
    return RFM95_OK;
}

uint8_t RFM95_LoRa_setPreamble(RFM95_t* rfm95, uint16_t preamblelen)
{
    uint8_t writeVal[2];
    writeVal[0] = (uint8_t)(preamblelen >> 8);
    writeVal[1] = (uint8_t)(preamblelen & 0x00FF);
    if(rfm95_writeReg(rfm95, REG_LORA_PREAMBLE_MSB, &writeVal[0]) ||
    rfm95_writeReg(rfm95, REG_LORA_PREAMBLE_LSB, &writeVal[1]))
        return RFM95_ERR_WRITE_REG;

    rfm95->Settings.LoRaPcktHandler.preamble = preamblelen;
    return RFM95_OK;
}

uint8_t RFM95_LoRa_setLowDataRateOpt(RFM95_t* rfm95, bool val)
{
	uint8_t readVal, writeVal;
	if(rfm95_readReg(rfm95, REG_MODEM_CONFIG_3, &readVal))
		return RFM95_ERR_READ_REG;
	// Low data rate optimize register location
	writeVal = (uint8_t)((readVal & 0xF7) | (val << 3));
	if(rfm95_writeReg(rfm95, REG_MODEM_CONFIG_3, &writeVal))
		return RFM95_ERR_WRITE_REG;
	rfm95->Settings.LoRa.lowDatarateOptimize = val;
		return RFM95_OK;
}

uint8_t rfm95_lora_dio_mapping(RFM95_t* rfm95, uint8_t dioNB, uint8_t dioStatus)
{
    uint8_t reg, dio_mapping_mask, dio_mapping_status;
    if(dioStatus > 3) return RFM95_ERR_INVALID_INPUT;
    switch(dioNB)
    {
        case 0:
        case 1:
        case 2:
        case 3:
            reg = REG_DIO_MAPPING_1;
            dio_mapping_mask = (uint8_t)(~(0x3 << (6 - 2 * dioNB)));
            dio_mapping_status = dioStatus << (6 - 2 * dioNB);
            break;
        case 4:
        case 5:
            reg = REG_DIO_MAPPING_2;
            dio_mapping_mask = (uint8_t)(~(0x3 << (6 - 2 * (dioNB - 4))));
            dio_mapping_status = dioStatus << (6 - 2 * (dioNB - 4));
            break;
        default:
            return RFM95_ERR_INVALID_INPUT;
    }

    uint8_t readVal, writeVal;
    if(rfm95_readReg(rfm95, reg, &readVal))
        return RFM95_ERR_READ_REG;
    // ex. LoRa DIO0 set RxDone(0x00), other pins are set to default
    writeVal = (uint8_t)((readVal & dio_mapping_mask) | dio_mapping_status);
    if(rfm95_writeReg(rfm95, reg, &writeVal))
        return RFM95_ERR_WRITE_REG;

    rfm95->DIO.DioStatus[dioNB] = dioStatus;
    return RFM95_OK;
}

uint8_t RFM95_checkVersion(RFM95_t* rfm95)
{
    uint8_t readVal;
    if(rfm95_readReg(rfm95, REG_VERSION, &readVal))
        return RFM95_ERR_READ_REG;
    if(readVal != 0x12) 
        return RFM95_ERR_CHIP_VERSION;
    
    return RFM95_OK;
}

uint8_t RFM95_LoRa_transmit(RFM95_t* rfm95, uint8_t* buffer, uint8_t bufferlen){
    uint8_t ret;
    lora_dev_mode currentMode = rfm95->Settings.LoraState;
    uint8_t irq_txdone = (uint8_t)(Irq_TxDone), fifptr = 0;
    // set RegIrqFlagsMask: Only TxDoneMask is activated.
    ret = rfm95_writeReg(rfm95, REG_IRQ_FLAGS_MASK, &irq_txdone);
    if(ret) return ret;
    // set DIO0 Tx_Done IRQ: 0b01
    ret = rfm95_lora_dio_mapping(rfm95, 0, DIOx_MAPPING_01);
    if(ret) return ret;
    // set standby mode
    ret = RFM95_LoRa_setOpMode(rfm95, STDBY_MODE);
    if(ret) return ret;
    // set packet length
    ret = rfm95_writeReg(rfm95, REG_LORA_PAYLOAD_LEN, &bufferlen);
    if(ret) return ret;
    // set FifoAddrPtr to FifoTxBaseAddr 
    ret = rfm95_writeReg(rfm95, REG_FIFO_TX_BASE_ADDR, &fifptr);
    if(ret) return ret;
    ret = rfm95_writeReg(rfm95, REG_FIFO_ADDR_PTR, &fifptr);
    if(ret) return ret;
    // Write packets to RegFifo in FIFO access
    ret = rfm95_fifo_writeReg(rfm95, buffer, bufferlen);
    if(ret) return ret;
    // set Tx Mode
    ret = RFM95_LoRa_setOpMode(rfm95, TX_MODE);
    if(ret) return ret;

    uint16_t timer = 0;
    // Transmit a signal must be within 2 second (2000 ms). 
    while(timer < TRANSMIT_TIMEOUT)
    {
        if(rfm95->DIO.DIO0() == true)
        {
            uint8_t readVal;
            uint8_t irq_txdone = (uint8_t)(~Irq_TxDone);
            rfm95_readReg(rfm95, REG_IRQ_FLAGS, &readVal);
            if((readVal & irq_txdone) != 0x00)
            {
                // clear the TxDone interrupt
                rfm95_writeReg(rfm95, REG_IRQ_FLAGS, &irq_txdone);
                RFM95_LoRa_setOpMode(rfm95, currentMode);
                return RFM95_OK;
            }
        }
        // Delay 1 ms
        rfm95->DelayUs(1000);
        timer++;
    }
    RFM95_LoRa_setOpMode(rfm95, SLEEP_MODE);
    return RFM95_ERR_TX_TIMEOUT;
}

uint8_t RFM95_LoRa_prepareReceive(RFM95_t* rfm95, bool isRxContinous)
{
    uint8_t ret;
    if(isRxContinous)
    {
    	uint8_t irq = Irq_RxDone & Irq_PayloadCrcError & Irq_ValidHeader;
    	uint8_t fifoptr = 0;
        // set RegIrqFlagsMask: RxDoneMask, PayloadCrcErrorMask and ValidHeaderMask are activated.
        ret = rfm95_writeReg(rfm95, REG_IRQ_FLAGS_MASK, &irq);
        if(ret) return ret;
        // set DIO0 Rx_Done IRQ: 0b00
        ret = rfm95_lora_dio_mapping(rfm95, 0, DIOx_MAPPING_00);
        if(ret) return ret;
        // set standby mode
        ret = RFM95_LoRa_setOpMode(rfm95, STDBY_MODE);
        if(ret) return ret;
        // set FifoAddrPtr to FifoRxBaseAddr
        ret = rfm95_writeReg(rfm95, REG_FIFO_ADDR_PTR, &fifoptr);
        if(ret) return ret;
        ret = rfm95_writeReg(rfm95, REG_FIFO_RX_BASE_ADDR, &fifoptr);
        if(ret) return ret;

        // set Rx Mode
		ret = RFM95_LoRa_setOpMode(rfm95, RX_CONTINUOUS);
		if(ret) return ret;
        rfm95->Settings.LoRa.isRxContinous = true;
    }
    else
    {
        // TODO
    }

    return RFM95_OK;
}

uint8_t RFM95_LoRa_receive(RFM95_t* rfm95, uint8_t* buffer, uint8_t bufferlen)
{
    uint8_t ret, readIrqFlag, rxNbBytes, rxCurrentAddr, minLen, irq_rxFlags;
    // check current IRQ flags.
    ret = rfm95_readReg(rfm95, REG_IRQ_FLAGS, &readIrqFlag);
    if(ret) return ret;
    // RxDone
    if((readIrqFlag & (uint8_t)(~Irq_RxDone | ~Irq_ValidHeader)) != 0x00)
    {
        // Set RegFifoAddrPtr to RegFifoRxCurrentAddr
        rfm95_readReg(rfm95, REG_RX_NB_BYTES, &rxNbBytes);
        rfm95_readReg(rfm95, REG_FIFO_RX_CURRENT_ADDR, &rxCurrentAddr);
        rfm95_writeReg(rfm95, REG_FIFO_ADDR_PTR, &rxCurrentAddr);
        minLen = (bufferlen >= rxNbBytes) ? rxNbBytes : bufferlen;
        
        ret = rfm95_fifo_readReg(rfm95, buffer, minLen);
        if(ret) return ret;
        // CRC Error
        if((readIrqFlag & (uint8_t)(~Irq_PayloadCrcError)) != 0x00)
        {
        	// clear the RxFlags interrupt
        	irq_rxFlags = (uint8_t)(~Irq_RxDone | ~Irq_PayloadCrcError | Irq_ValidHeader);
			rfm95_writeReg(rfm95, REG_IRQ_FLAGS, &irq_rxFlags);
            return RFM95_ERR_RX_PAYLOAD_CRC;
        }
        else
        {
        	// clear the RxFlags interrupt
			irq_rxFlags = (uint8_t)(~Irq_RxDone | Irq_ValidHeader);
			rfm95_writeReg(rfm95, REG_IRQ_FLAGS, &irq_rxFlags);
	        return RFM95_OK;
        }
    }
    // fail to receive signal
    else
    {
    	// clear the RxFlags interrupt
		irq_rxFlags = (uint8_t)(~Irq_RxDone);
        RFM95_LoRa_setOpMode(rfm95, SLEEP_MODE);
        return RFM95_ERR_RX_FAIL;
    }
}

uint8_t RFM95_LoRa_setSyncWord(RFM95_t* rfm95 ,uint8_t syncword)
{
	if(rfm95_writeReg(rfm95, REG_SYNCWORD, &syncword))
		return RFM95_ERR_WRITE_REG;
	else
		return RFM95_OK;
}

float RFM95_getPcktSNR(RFM95_t* rfm95)
{
	uint8_t readVal;
	rfm95_readReg(rfm95, REG_PCKT_SNR_VAL, &readVal);
	rfm95->Settings.LoRaPcktHandler.SNR = (float)((int8_t)readVal * 0.25);
	return rfm95->Settings.LoRaPcktHandler.SNR;
}

int16_t RFM95_getPcktRSSI(RFM95_t* rfm95)
{

	uint8_t pcktRSSI;
	if(rfm95_readReg(rfm95, REG_PCKT_RSSI_VAL, &pcktRSSI))
				return RFM95_ERR_READ_REG;
	float pcktSNR = RFM95_getPcktSNR(rfm95);;
	// high frequency
	if(rfm95->Settings.LoRa.frequency <= 1020000000 &&
			rfm95->Settings.LoRa.frequency >= 862000000)
	{
		if((pcktSNR >= 0.0f))
			rfm95->Settings.LoRaPcktHandler.pcktRSSI = (int16_t)(-157 + pcktRSSI);
		else
			rfm95->Settings.LoRaPcktHandler.pcktRSSI = (int16_t)(-157 + pcktRSSI + pcktSNR * 0.25);
		return rfm95->Settings.LoRaPcktHandler.pcktRSSI;
	}
	// low frequency
	else if(rfm95->Settings.LoRa.frequency <= 525000000 &&
			rfm95->Settings.LoRa.frequency >= 137000000)
	{
		if(pcktSNR >= 0.0f)
			rfm95->Settings.LoRaPcktHandler.pcktRSSI = (int16_t)(-164 + pcktRSSI);
		else
			rfm95->Settings.LoRaPcktHandler.pcktRSSI = (int16_t)(-164 + pcktRSSI + pcktSNR * 0.25);
		return rfm95->Settings.LoRaPcktHandler.pcktRSSI;
	}
	else
	{
		return RFM95_ERR_FREQUENCY;
	}
}

int16_t RFM95_getCurrentRSSI(RFM95_t* rfm95)
{
	uint8_t RSSI;
	if(rfm95_readReg(rfm95, REG_RSSI_VAL, &RSSI))
		return RFM95_ERR_READ_REG;
	// high frequency
	if(rfm95->Settings.LoRa.frequency <= 1020000000 &&
			rfm95->Settings.LoRa.frequency >= 862000000)
	{
		rfm95->Settings.LoRaPcktHandler.currentRSSI = (int16_t)(-157 + RSSI);
		return rfm95->Settings.LoRaPcktHandler.currentRSSI;
	}// low frequency
	else if(rfm95->Settings.LoRa.frequency <= 525000000 &&
			rfm95->Settings.LoRa.frequency >= 137000000)
	{
		rfm95->Settings.LoRaPcktHandler.currentRSSI = (int16_t)(-164 + RSSI);
		return rfm95->Settings.LoRaPcktHandler.currentRSSI;
	}
	else
	{
		return RFM95_ERR_FREQUENCY;
	}
}

uint32_t RFM95_LoRa_random(RFM95_t* rfm95)
{
	uint32_t randomNB = 0;
	uint8_t currentIRQ, currentMode, writeIRQ = 0xFF;
	// Remain current RegIrqFlagsMask value
	if(rfm95_readReg(rfm95, REG_IRQ_FLAGS_MASK, &currentIRQ))
		return RFM95_ERR_READ_REG;
	// Disable LoRa modem interrupts
	if(rfm95_writeReg(rfm95, REG_IRQ_FLAGS_MASK, &writeIRQ))
		return RFM95_ERR_WRITE_REG;
	// Set it in rxContinouts
	currentMode = rfm95->Settings.LoraState;
	RFM95_LoRa_setOpMode(rfm95, RX_CONTINUOUS);

	for(int i = 0; i < 32; i++)
	{
		rfm95->DelayUs(1000);
		// Get the LSB of current RSSI value iteratively.
		randomNB |= ((uint32_t)RFM95_getCurrentRSSI(rfm95) & 0x1) << i;
	}
	// Restore to original mode
	RFM95_LoRa_setOpMode(rfm95, currentMode);
	// Restore LoRa modem interrupts
	rfm95_writeReg(rfm95, REG_IRQ_FLAGS_MASK, &currentIRQ);
	return randomNB;
}

uint8_t RFM95_LoRa_Init(RFM95_t* rfm95)
{
    uint8_t ret;
    // reset
    RFM95_Reset(rfm95);
    // enter sleep mode, regs should be written only in Sleep and Standby modes.
	ret = RFM95_LoRa_setOpMode(rfm95, SLEEP_MODE);
	if(ret) return ret;
//	printf("RFM95_LoRa_setOpMode ..ok\r\n");
    // set lora mode
    ret = RFM95_setModem(rfm95, MODEM_LORA);
    if(ret) return ret;
//    printf("RFM95_setModem ..ok\r\n");
    // set frequency
    ret = RFM95_setFrequency(rfm95, 923400000);
    if(ret) return ret;
    printf("Freq: %lu \r\n", RFM95_getFrequency(rfm95));
    // set output power gain
    ret = RFM95_setTXPower(rfm95, 20);
    if(ret) return ret;
//    printf("RFM95_setTXPower ..ok\r\n");
    // set over current protection
    ret = RFM95_setOCP(rfm95, 200);
    if(ret) return ret;
//    printf("RFM95_setOCP ..ok\r\n");
    // set LNA gain
    ret = RFM95_setLNAGain(rfm95, LNA_G1);
    if(ret) return ret;
//    printf("RFM95_setLNAGain ..ok\r\n");
    // set spreading factor
    ret = RFM95_LoRa_setSpreadingFactor(rfm95, 11);
    if(ret) return ret;
//    printf("RFM95_LoRa_setSpreadingFactor ..ok\r\n");
    // set rx crc on
    ret = RFM95_LoRa_setRxPayloadCrcOn(rfm95, true);
    if(ret) return ret;
//    printf("RFM95_LoRa_setRxPayloadCrcOn ..ok\r\n");
    // set timeout
    ret = rfm95_lora_symb_timout(rfm95, 0x3FF);
    if(ret) return ret;
//    printf("rfm95_lora_symb_timout ..ok\r\n");
    // set bandwidth
    ret = RFM95_LoRa_setBandwidth(rfm95, BW_125_kHz);
    if(ret) return ret;
//    printf("RFM95_LoRa_setBandwidth ..ok\r\n");
    // set coding rate
    ret = RFM95_LoRa_setCodingRate(rfm95, CR_4_5);
    if(ret) return ret;
//    printf("RFM95_LoRa_setCodingRate ..ok\r\n");
    // set explicit mode
    ret = RFM95_LoRa_setHeaderMode(rfm95, EXPLICIT_HEADER_MODE);
    if(ret) return ret;
//    printf("RFM95_LoRa_setHeaderMode ..ok\r\n");
    // By default the packet is configured with a 12 symbol long sequence
    // If it exceeds over 19, SLGv1 won't be able to receive packet successfully.
    ret = RFM95_LoRa_setPreamble(rfm95, 6);
    if(ret) return ret;
//    printf("RFM95_LoRa_setPreamble ..ok\r\n");
    // set DIO0 Rx_Done IRQ: 0b00
    ret = RFM95_LoRa_setLowDataRateOpt(rfm95, true);
    if(ret) return ret;

    ret = rfm95_lora_dio_mapping(rfm95, 0, DIOx_MAPPING_00);
    if(ret) return ret;
//    printf("rfm95_lora_dio_mapping ..ok\r\n");
//    uint8_t readVal;
//    rfm95_readReg(rfm95, REG_DIO_MAPPING_1, &readVal);
//    printf("1st dio mapping: %02x\r\n", readVal);
//    readVal |= 0x3F;
//    rfm95_writeReg(rfm95, REG_DIO_MAPPING_1, &readVal);
//    readVal = 0;
//    rfm95_readReg(rfm95, REG_DIO_MAPPING_1, &readVal);
//    printf("2nd dio mapping: %02x\r\n", readVal);

//    // Frequency hopping
//    uint8_t hop = 10;
//    uint8_t pllHop;
//    rfm95_readReg(rfm95, REG_PILL_HOP, &pllHop);
//    printf("PLL HOP: %02x\r\n", pllHop);
//    pllHop = (pllHop & 0x7F) | 0x80;
//    rfm95_writeReg(rfm95, REG_PILL_HOP, &pllHop);
//    rfm95_writeReg(rfm95, REG_HOP_PERIOD, &hop);
    // enter standby mode
    ret = RFM95_LoRa_setOpMode(rfm95, STDBY_MODE);
    if(ret) return ret;
//    printf("RFM95_LoRa_setOpMode ..ok\r\n");
    // check version
    ret = RFM95_checkVersion(rfm95);
    if(ret) return ret;
    printf("RFM95_checkVersion ..ok\r\n");

    return RFM95_OK;
}

uint8_t rfm95_readReg(RFM95_t* rfm95, uint8_t reg, uint8_t* readVal)
{
    uint8_t ret;
    uint8_t writeVal = reg & 0x7f;
    rfm95->NSEL(0);
    rfm95->SPI_WriteRead(&writeVal, readVal, 2, SPI_TIMEOUT);
    rfm95->NSEL(1);
    ret = rfm95->SPI_CheckState();
    if(ret == rfm95->spi_ok) return RFM95_OK;
    else return RFM95_ERR_READ_REG;
}

uint8_t rfm95_writeReg(RFM95_t* rfm95, uint8_t reg, uint8_t* writeVal)
{
    uint8_t ret;
    uint8_t buff[] = {(reg | 0x80), *writeVal};
    rfm95->NSEL(0);
    rfm95->SPI_Write(buff, 2, SPI_TIMEOUT);
    ret = rfm95->SPI_CheckState();
    rfm95->NSEL(1);
    if(ret == rfm95->spi_ok) return RFM95_OK;
    else return RFM95_ERR_WRITE_REG;
}

uint8_t rfm95_burst_readReg(RFM95_t* rfm95, uint8_t reg, uint8_t* readVal, uint8_t length)
{
    uint8_t ret[2];
    uint8_t buff = reg & 0x7f;
    rfm95->NSEL(0);
    rfm95->SPI_Write(&buff, 1, SPI_TIMEOUT);
    ret[0] = rfm95->SPI_CheckState();
    rfm95->SPI_Read(readVal, length, SPI_TIMEOUT);
    ret[1] = rfm95->SPI_CheckState();
    rfm95->NSEL(1);
    if((ret[0] == rfm95->spi_ok) && (ret[1] == rfm95->spi_ok)) return RFM95_OK;
    else return RFM95_ERR_BURST_READ_REG;
}

uint8_t rfm95_burst_writeReg(RFM95_t* rfm95, uint8_t reg, uint8_t* writeVal, uint8_t length)
{
    uint8_t ret;
    uint8_t *buff = (uint8_t*)malloc((length + 1) * sizeof(uint8_t));
    buff[0] = reg | 0x80;
    for(int i = 1; i < (length + 1); i++)
        buff[i] = writeVal[i-1];
    rfm95->NSEL(0);
    rfm95->SPI_Write(buff, length + 1, SPI_TIMEOUT);
    ret = rfm95->SPI_CheckState();
    rfm95->NSEL(1);
    free(buff);
    if(ret == rfm95->spi_ok) return RFM95_OK;
    else return RFM95_ERR_BURST_WRITE_REG;
}

uint8_t rfm95_fifo_readReg(RFM95_t* rfm95, uint8_t* readVal, uint8_t length)
{
    return rfm95_burst_readReg(rfm95, REG_FIFO, readVal, length);
}

uint8_t rfm95_fifo_writeReg(RFM95_t* rfm95, uint8_t* writeVal, uint8_t length)
{
    return rfm95_burst_writeReg(rfm95, REG_FIFO, writeVal, length);
}
