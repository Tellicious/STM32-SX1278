/* BEGIN Header */
/**
 ******************************************************************************
 * @file    SX1278.c
 * @author  Andrea Vivani
 * @brief   Semtech SX1278 LoRa radio driver
 ******************************************************************************
 * @copyright
 *
 * Copyright 2023 Andrea Vivani
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 ******************************************************************************
 */
/* END Header */

/* Includes ------------------------------------------------------------------*/

#include "SX1278.h"
#include <string.h>
#include "gpio.h"
#include "spi.h"

/* Macros --------------------------------------------------------------------*/

//Internal registers Address
/********************LoRa mode***************************/
#define SX1278_LR_RegFifo                                  0x00
// Common settings
#define SX1278_LR_RegOpMode                                0x01
#define SX1278_LR_RegFrMsb                                 0x06
#define SX1278_LR_RegFrMid                                 0x07
#define SX1278_LR_RegFrLsb                                 0x08
// Tx settings
#define SX1278_LR_RegPaConfig                              0x09
#define SX1278_LR_RegPaRamp                                0x0A
#define SX1278_LR_RegOcp                                   0x0B
// Rx settings
#define SX1278_LR_RegLna                                   0x0C
// LoRa registers
#define SX1278_LR_RegFifoAddrPtr                           0x0D
#define SX1278_LR_RegFifoTxBaseAddr                        0x0E
#define SX1278_LR_RegFifoRxBaseAddr                        0x0F
#define SX1278_LR_RegFifoRxCurrentaddr                     0x10
#define SX1278_LR_RegIrqFlagsMask                          0x11
#define SX1278_LR_RegIrqFlags                              0x12
#define SX1278_LR_RegRxNbBytes                             0x13
#define SX1278_LR_RegRxHeaderCntValueMsb                   0x14
#define SX1278_LR_RegRxHeaderCntValueLsb                   0x15
#define SX1278_LR_RegRxPacketCntValueMsb                   0x16
#define SX1278_LR_RegRxPacketCntValueLsb                   0x17
#define SX1278_LR_RegModemStat                             0x18
#define SX1278_LR_RegPktSnrValue                           0x19
#define SX1278_LR_RegPktRssiValue                          0x1A
#define SX1278_LR_RegRssiValue                             0x1B
#define SX1278_LR_RegHopChannel                            0x1C
#define SX1278_LR_RegModemConfig1                          0x1D
#define SX1278_LR_RegModemConfig2                          0x1E
#define SX1278_LR_RegSymbTimeoutLsb                        0x1F
#define SX1278_LR_RegPreambleMsb                           0x20
#define SX1278_LR_RegPreambleLsb                           0x21
#define SX1278_LR_RegPayloadLength                         0x22
#define SX1278_LR_RegMaxPayloadLength                      0x23
#define SX1278_LR_RegHopPeriod                             0x24
#define SX1278_LR_RegFifoRxByteAddr                        0x25
#define SX1278_LR_RegModemConfig3                          0x26
#define SX1278_LR_RegSyncWord		                       	0x39
// I/O settings
#define SX1278_LR_RegDIOMAPPING1                          0x40
#define SX1278_LR_RegDIOMAPPING2                          0x41
// Version
#define SX1278_LR_RegVERSION                              0x42
// Additional settings
#define SX1278_LR_RegPLLHOP                               0x44
#define SX1278_LR_RegTCXO                                 0x4B
#define SX1278_LR_RegPADAC                                0x4D
#define SX1278_LR_RegFORMERTEMP                           0x5B
#define SX1278_LR_RegAGCREF                               0x61
#define SX1278_LR_RegAGCTHRESH1                           0x62
#define SX1278_LR_RegAGCTHRESH2                           0x63
#define SX1278_LR_RegAGCTHRESH3                           0x64

/********************FSK/ook mode***************************/
#define  SX1278_RegFIFO                0x00
#define  SX1278_RegOpMode              0x01
#define  SX1278_RegBitRateMsb      		0x02
#define  SX1278_RegBitRateLsb      		0x03
#define  SX1278_RegFdevMsb             0x04
#define  SX1278_RegFdevLsb             0x05
#define  SX1278_RegFreqMsb             0x06
#define  SX1278_RegFreqMid             0x07
#define  SX1278_RegFreqLsb         	    0x08
#define  SX1278_RegPaConfig            0x09
#define  SX1278_RegPaRamp              0x0a
#define  SX1278_RegOcp                 0x0b
#define  SX1278_RegLna                 0x0c
#define  SX1278_RegRxConfig            0x0d
#define  SX1278_RegRssiConfig      	    0x0e
#define  SX1278_RegRssiCollision 		0x0f
#define  SX1278_RegRssiThresh       	0x10
#define  SX1278_RegRssiValue           0x11
#define  SX1278_RegRxBw                0x12
#define  SX1278_RegAfcBw               0x13
#define  SX1278_RegOokPeak             0x14
#define  SX1278_RegOokFix              0x15
#define  SX1278_RegOokAvg              0x16
#define  SX1278_RegAfcFei              0x1a
#define  SX1278_RegAfcMsb              0x1b
#define  SX1278_RegAfcLsb              0x1c
#define  SX1278_RegFeiMsb              0x1d
#define  SX1278_RegFeiLsb              0x1e
#define  SX1278_RegPreambleDetect  	0x1f
#define  SX1278_RegRxTimeout1       	0x20
#define  SX1278_RegRxTimeout2       	0x21
#define  SX1278_RegRxTimeout3       	0x22
#define  SX1278_RegRxDelay             0x23
#define  SX1278_RegOsc                 0x24
#define  SX1278_RegPreambleMsb      	0x25
#define  SX1278_RegPreambleLsb      	0x26
#define  SX1278_RegSyncConfig       	0x27
#define  SX1278_RegSyncValue1       	0x28
#define  SX1278_RegSyncValue2       	0x29
#define  SX1278_RegSyncValue3       	0x2a
#define  SX1278_RegSyncValue4       	0x2b
#define  SX1278_RegSyncValue5       	0x2c
#define  SX1278_RegSyncValue6       	0x2d
#define  SX1278_RegSyncValue7       	0x2e
#define  SX1278_RegSyncValue8       	0x2f
#define  SX1278_RegPacketConfig1       0x30
#define  SX1278_RegPacketConfig2       0x31
#define  SX1278_RegPayloadLength       0x32
#define  SX1278_RegNodeAdrs            0x33
#define  SX1278_RegBroadcastAdrs       0x34
#define  SX1278_RegFifoThresh       	0x35
#define  SX1278_RegSeqConfig1       	0x36
#define  SX1278_RegSeqConfig2       	0x37
#define  SX1278_RegTimerResol       	0x38
#define  SX1278_RegTimer1Coef       	0x39
#define  SX1278_RegSyncWord			0x39
#define  SX1278_RegTimer2Coef      		0x3a
#define  SX1278_RegImageCal            0x3b
#define  SX1278_RegTemp                0x3c
#define  SX1278_RegLowBat              0x3d
#define  SX1278_RegIrqFlags1           0x3e
#define  SX1278_RegIrqFlags2           0x3f
#define  SX1278_RegDioMapping1			0x40
#define  SX1278_RegDioMapping2			0x41
#define  SX1278_RegVersion				0x42
#define  SX1278_RegPllHop				0x44
#define  SX1278_RegPaDac				0x4d
#define  SX1278_RegBitRateFrac			0x5d

/* Function prototypes --------------------------------------------------------*/

/* HW Interface  functions ----------------------------------------------------*/

#define SX1278_DELAY(x) 							HAL_Delay(x);

#define SX1278_WRITE_PIN(port, pin, status) 		HAL_GPIO_WritePin(port, pin, status)
#define SX1278_READ_PIN(port, pin) 				HAL_GPIO_ReadPin(port, pin)

#define SX1278_PIN_SET 							GPIO_PIN_SET
#define SX1278_PIN_RESET 						GPIO_PIN_RESET

static void SX1278_SPIBurstRead(SX1278_t *module, uint8_t addr, uint8_t *rxBuf, uint8_t length)
{
	SX1278_WRITE_PIN(module->cs.port, module->cs.pin, SX1278_PIN_RESET);
	while (HAL_SPI_GetState(module->spi) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(module->spi, &addr, 1, 1000);
	HAL_SPI_Receive(module->spi, rxBuf, length,1000);
	while (HAL_SPI_GetState(module->spi) != HAL_SPI_STATE_READY);
	SX1278_WRITE_PIN(module->cs.port, module->cs.pin, SX1278_PIN_SET);
}

static void SX1278_SPIBurstWrite(SX1278_t *module, uint8_t addr, uint8_t *txBuf, uint8_t length)
{
	SX1278_WRITE_PIN(module->cs.port, module->cs.pin, SX1278_PIN_RESET);
	while (HAL_SPI_GetState(module->spi) != HAL_SPI_STATE_READY);
	addr |= 0x80;
	HAL_SPI_Transmit(module->spi, &addr, 1, 1000);
	HAL_SPI_Transmit(module->spi, txBuf, length, 1000);
	while (HAL_SPI_GetState(module->spi) != HAL_SPI_STATE_READY);
	SX1278_WRITE_PIN(module->cs.port, module->cs.pin, SX1278_PIN_SET);
}

/* Static  functions ----------------------------------------------------------*/

static inline uint8_t SX1278_SPIRead(SX1278_t *module, uint8_t addr)
{
	uint8_t tmp;
	SX1278_SPIBurstRead(module, addr, &tmp, 1);
	return tmp;
}

static inline void SX1278_SPIWrite(SX1278_t *module, uint8_t addr, uint8_t cmd)
{
	SX1278_SPIBurstWrite(module, addr, &cmd, 1);
}

static void SX1278_hw_init(SX1278_t *module)
{
	SX1278_WRITE_PIN(module->cs.port, module->cs.pin, SX1278_PIN_SET);
	SX1278_WRITE_PIN(module->reset.port, module->reset.pin, SX1278_PIN_RESET);
	SX1278_DELAY(200);
	SX1278_WRITE_PIN(module->reset.port, module->reset.pin, SX1278_PIN_SET);
	SX1278_DELAY(200);
}

/* Private  functions ---------------------------------------------------------*/

void SX1278_config(SX1278_t *module)
{
	SX1278_hw_init(module);
	SX1278_sleep(module); //Change modem mode Must in Sleep mode
	SX1278_DELAY(15);

	SX1278_entryLoRa(module);
	//SX1278_SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note

	uint64_t freq = ((uint64_t) module->frequency << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	SX1278_SPIBurstWrite(module, SX1278_LR_RegFrMsb, (uint8_t*) freq_reg, 3); //setting  frequency parameter

	SX1278_SPIWrite(module, SX1278_RegSyncWord, module->syncWord);

	//setting base parameter
	SX1278_SPIWrite(module, SX1278_LR_RegPaConfig, module->power); //Setting output power parameter
	SX1278_SPIWrite(module, SX1278_LR_RegPADAC, 0x84);	//Normal and RX

	SX1278_SPIWrite(module, SX1278_LR_RegOcp, 0x3B);			//Ocp enabled @ 240mA
	SX1278_SPIWrite(module, SX1278_LR_RegLna, 0x23);		//RegLNA,High & LNA Enable
	if (module->LoRa_SF == SX1278_LORA_SF_6) //SFactor=6
	{
		uint8_t tmp;
		SX1278_SPIWrite(module, SX1278_LR_RegModemConfig1, (module->LoRa_BW + module->LoRa_CR + 0x01)); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module, SX1278_LR_RegModemConfig2, (module->LoRa_SF + module->LoRa_CRC_sum + 0x03));

		tmp = SX1278_SPIRead(module, 0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX1278_SPIWrite(module, 0x31, tmp);
		SX1278_SPIWrite(module, 0x37, 0x0C);
	}
	else
	{
		SX1278_SPIWrite(module,
		SX1278_LR_RegModemConfig1, (module->LoRa_BW + module->LoRa_CR + 0x00)); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module,
		SX1278_LR_RegModemConfig2, (module->LoRa_SF + module->LoRa_CRC_sum + 0x00)); //SFactor &  LNA gain set by the internal AGC loop
	}

	SX1278_SPIWrite(module, SX1278_LR_RegModemConfig3, 0x04);
	SX1278_SPIWrite(module, SX1278_LR_RegSymbTimeoutLsb, 0x08); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX1278_SPIWrite(module, SX1278_LR_RegPreambleMsb, 0x00); //RegPreambleMsb
	SX1278_SPIWrite(module, SX1278_LR_RegPreambleLsb, 0x08); //RegPreambleLsb 8+4=12byte Preamble
	SX1278_SPIWrite(module, SX1278_LR_RegDIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	module->readBytesNum = 0;
	SX1278_standby(module); //Entry standby mode
}

void SX1278_standby(SX1278_t *module)
{
	SX1278_SPIWrite(module, SX1278_LR_RegOpMode, 0x09);
	module->status = SX1278_STANDBY;
}

void SX1278_sleep(SX1278_t *module)
{
	SX1278_SPIWrite(module, SX1278_LR_RegOpMode, 0x08);
	module->status = SX1278_SLEEP;
}

void SX1278_entryLoRa(SX1278_t *module)
{
	SX1278_SPIWrite(module, SX1278_LR_RegOpMode, 0x88);
}

void SX1278_clearLoRaIrq(SX1278_t *module)
{
	SX1278_SPIWrite(module, SX1278_LR_RegIrqFlags, 0xFF);
}

SX1278_retStatus_t SX1278_LoRaEntryRx(SX1278_t *module, uint8_t length, uint32_t timeout)
{
	uint8_t addr;

	module->packetLength = length;

	//SX1278_config(module);		//Setting base parameter
	SX1278_standby(module); //Entry standby mode
	if(module->power < SX1278_POWER_17DBM)
	{
		SX1278_SPIWrite(module, SX1278_LR_RegPADAC, 0x84);	//Normal and RX
	}
//	SX1278_SPIWrite(module, SX1278_LR_RegHopPeriod, 0xFF);	//No FHSS
	SX1278_SPIWrite(module, SX1278_LR_RegDIOMAPPING1, 0x01);//DIO=00,DIO1=00,DIO2=00, DIO3=01
	SX1278_SPIWrite(module, SX1278_LR_RegIrqFlagsMask, 0x1F);//Open RxDone interrupt, Timeout, CRC Error
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, SX1278_LR_RegPayloadLength, length);//Payload Length 21byte(this register must define when the data long of one byte in SF is 6)
	addr = SX1278_SPIRead(module, SX1278_LR_RegFifoRxBaseAddr); //Read RxBaseAddr
	SX1278_SPIWrite(module, SX1278_LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX1278_SPIWrite(module, SX1278_LR_RegOpMode, 0x8d);	//Mode//Low Frequency Mode
	//SX1278_SPIWrite(module, SX1278_LR_RegOpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
	module->readBytesNum = 0;

	while (1)
	{
		if ((SX1278_SPIRead(module, SX1278_LR_RegModemStat) & 0x04) == 0x04)
		{	//Rx-on
			module->status = SX1278_RX;
			return SX1278_SUCCESS;
		}
		if (--timeout == 0)
		{
			SX1278_hw_init(module);
			SX1278_config(module);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
}

uint8_t SX1278_LoRaRxPacket(SX1278_t *module)
{
	uint8_t addr;
	uint8_t packet_size;
	module -> readBytesNum = 0;

	if (SX1278_READ_PIN(module->dio0.port, module->dio0.pin) == SX1278_PIN_RESET)
	{
		return 0;
	}

	if ((SX1278_SPIRead(module, SX1278_LR_RegIrqFlags) & 0x60) == 0x40) { //Check RX Done is set and CRC error is not
		memset(module->rxBuffer, 0x00, SX1278_MAX_PACKET);

		addr = SX1278_SPIRead(module, SX1278_LR_RegFifoRxCurrentaddr); //last packet addr
		SX1278_SPIWrite(module, SX1278_LR_RegFifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if (module->LoRa_SF == SX1278_LORA_SF_6) //When SpreadFactor is six,will used Implicit Header mode(Excluding internal packet length)
		{
			packet_size = module->packetLength;
		}
		else
		{
			packet_size = SX1278_SPIRead(module, SX1278_LR_RegRxNbBytes); //Number for received bytes
		}

		SX1278_SPIBurstRead(module, 0x00, module->rxBuffer, packet_size);
		module->readBytesNum = packet_size;
		SX1278_clearLoRaIrq(module);
		return module->readBytesNum;
	}
	else
	{
		SX1278_clearLoRaIrq(module);
		return 0;
	}
}

SX1278_retStatus_t SX1278_LoRaEntryTx(SX1278_t *module, uint8_t length, uint32_t timeout)
{
	uint8_t addr;
	uint8_t temp;

	module->packetLength = length;

	//SX1278_config(module); //setting base parameter
	SX1278_standby(module); //Entry standby mode
	if(module->power < SX1278_POWER_17DBM)
		SX1278_SPIWrite(module, SX1278_LR_RegPADAC, 0x87);	//Tx for 20dBm
//	SX1278_SPIWrite(module, SX1278_LR_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1278_SPIWrite(module, SX1278_LR_RegDIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, SX1278_LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1278_SPIWrite(module, SX1278_LR_RegPayloadLength, length); //RegPayloadLength 21byte
	addr = SX1278_SPIRead(module, SX1278_LR_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX1278_SPIWrite(module, SX1278_LR_RegFifoAddrPtr, addr); //RegFifoAddrPtr

	while (1)
	{
		temp = SX1278_SPIRead(module, SX1278_LR_RegPayloadLength);
		if (temp == length)
		{
			module->status = SX1278_TX;
			return SX1278_SUCCESS;
		}

		if (--timeout == 0)
		{
			SX1278_hw_init(module);
			SX1278_config(module);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
}

SX1278_retStatus_t SX1278_LoRaTxPacket(SX1278_t *module, uint8_t *txBuffer, uint8_t length, uint32_t timeout)
{
	SX1278_SPIBurstWrite(module, 0x00, txBuffer, length);
	SX1278_SPIWrite(module, SX1278_LR_RegOpMode, 0x8b);	//Tx Mode
	while (1)
	{
		if ((SX1278_READ_PIN(module->dio0.port, module->dio0.pin) == SX1278_PIN_SET) && (SX1278_SPIRead(module, SX1278_LR_RegIrqFlags) & 0x08)) //if(Get_NIRQ()) //Packet send over
		{
			SX1278_SPIRead(module, SX1278_LR_RegIrqFlags);
			SX1278_clearLoRaIrq(module); //Clear irq
			SX1278_standby(module); //Entry Standby mode
			return SX1278_SUCCESS;
		}

		if (--timeout == 0)
		{
			SX1278_hw_init(module);
			SX1278_config(module);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
}

SX1278_retStatus_t SX1278_transmit(SX1278_t *module, uint8_t *txBuf, uint8_t length, uint32_t timeout)
{
	uint8_t addr;

	module->packetLength = length;

	//SX1278_config(module); //setting base parameter
	SX1278_standby(module); //Entry standby mode
	if(module->power < SX1278_POWER_17DBM)
	{
		SX1278_SPIWrite(module, SX1278_LR_RegPADAC, 0x87);	//Tx for 20dBm
	}
//	SX1278_SPIWrite(module, SX1278_LR_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1278_SPIWrite(module, SX1278_LR_RegDIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, SX1278_LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1278_SPIWrite(module, SX1278_LR_RegPayloadLength, length); //RegPayloadLength 21byte
	addr = SX1278_SPIRead(module, SX1278_LR_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX1278_SPIWrite(module, SX1278_LR_RegFifoAddrPtr, addr); //RegFifoAddrPtr
	SX1278_SPIBurstWrite(module, 0x00, txBuf, length);
	SX1278_SPIWrite(module, SX1278_LR_RegOpMode, 0x8b);	//Tx Mode
	module->status = SX1278_TX;
	while (1)
	{
		if ((SX1278_READ_PIN(module->dio0.port, module->dio0.pin) == SX1278_PIN_SET) && (SX1278_SPIRead(module, SX1278_LR_RegIrqFlags) & 0x08)) //if(Get_NIRQ()) //Packet send over
		{
			SX1278_clearLoRaIrq(module); //Clear irq
			SX1278_standby(module); //Entry Standby mode
			return SX1278_SUCCESS;
		}

		if (--timeout == 0)
		{
			SX1278_hw_init(module);
			SX1278_config(module);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
	return SX1278_ERROR;
}

uint8_t SX1278_read(SX1278_t *module, uint8_t *rxBuf, uint8_t length)
{
	if (length != module->readBytesNum)
	{
		length = module->readBytesNum;
	}
	memcpy(rxBuf, module->rxBuffer, length);
	rxBuf[length] = '\0';
	module->readBytesNum = 0;
	return length;
}

uint8_t SX1278_RSSI_LoRa(SX1278_t *module)
{
	uint32_t temp = 10;
	temp = SX1278_SPIRead(module, SX1278_LR_RegRssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset
	return (uint8_t) temp;
}

uint8_t SX1278_RSSI(SX1278_t *module)
{
	uint8_t temp = 0xff;
	temp = SX1278_SPIRead(module, SX1278_RegRssiValue);
	temp = 127 - (temp >> 1);	//127:Max RSSI
	return temp;
}
