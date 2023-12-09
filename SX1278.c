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
#define LR_REG_Fifo                                 0x00
// Common settings
#define LR_REG_OpMode                               0x01
#define LR_REG_FrMsb                                0x06
#define LR_REG_FrMid                                0x07
#define LR_REG_FrLsb                                0x08
// Tx settings
#define LR_REG_PaConfig                             0x09
#define LR_REG_PaRamp                               0x0A
#define LR_REG_Ocp                                  0x0B
// Rx settings
#define LR_REG_Lna                                  0x0C
// LoRa registers
#define LR_REG_FifoAddrPtr                          0x0D
#define LR_REG_FifoTxBaseAddr                       0x0E
#define LR_REG_FifoRxBaseAddr                       0x0F
#define LR_REG_FifoRxCurrentaddr                    0x10
#define LR_REG_IrqFlagsMask                         0x11
#define LR_REG_IrqFlags                             0x12
#define LR_REG_RxNbBytes                            0x13
#define LR_REG_RxHeaderCntValueMsb                  0x14
#define LR_REG_RxHeaderCntValueLsb                  0x15
#define LR_REG_RxPacketCntValueMsb                  0x16
#define LR_REG_RxPacketCntValueLsb                  0x17
#define LR_REG_ModemStat                            0x18
#define LR_REG_PktSnrValue                          0x19
#define LR_REG_PktRssiValue                         0x1A
#define LR_REG_RssiValue                            0x1B
#define LR_REG_HopChannel                           0x1C
#define LR_REG_ModemConfig1                         0x1D
#define LR_REG_ModemConfig2                         0x1E
#define LR_REG_SymbTimeoutLsb                       0x1F
#define LR_REG_PreambleMsb                          0x20
#define LR_REG_PreambleLsb                          0x21
#define LR_REG_PayloadLength                        0x22
#define LR_REG_MaxPayloadLength                     0x23
#define LR_REG_HopPeriod                            0x24
#define LR_REG_FifoRxByteAddr                       0x25
#define LR_REG_ModemConfig3                         0x26
#define LR_REG_SyncWord		                       	0x39
// I/O settings
#define LR_REG_DIOMAPPING1                          0x40
#define LR_REG_DIOMAPPING2                          0x41
// Version 
#define LR_REG_VERSION                              0x42
// Additional settings 
#define LR_REG_PLLHOP                               0x44
#define LR_REG_TCXO                                 0x4B
#define LR_REG_PADAC                                0x4D
#define LR_REG_FORMERTEMP                           0x5B
#define LR_REG_AGCREF                               0x61
#define LR_REG_AGCTHRESH1                           0x62
#define LR_REG_AGCTHRESH2                           0x63
#define LR_REG_AGCTHRESH3                           0x64

/********************FSK/ook mode***************************/
#define  REG_FIFO                	0x00
#define  REG_OpMode              	0x01
#define  REG_BitRateMsb      		0x02
#define  REG_BitRateLsb      		0x03
#define  REG_FdevMsb             	0x04
#define  REG_FdevLsb             	0x05
#define  REG_FreqMsb             	0x06
#define  REG_FreqMid             	0x07
#define  REG_FreqLsb         	    0x08
#define  REG_PaConfig            	0x09
#define  REG_PaRamp              	0x0a
#define  REG_Ocp                 	0x0b
#define  REG_Lna                 	0x0c
#define  REG_RxConfig            	0x0d
#define  REG_RssiConfig      	    0x0e
#define  REG_RssiCollision 			0x0f
#define  REG_RssiThresh       		0x10
#define  REG_RssiValue           	0x11
#define  REG_RxBw                	0x12
#define  REG_AfcBw               	0x13
#define  REG_OokPeak             	0x14
#define  REG_OokFix              	0x15
#define  REG_OokAvg              	0x16
#define  REG_AfcFei              	0x1a
#define  REG_AfcMsb              	0x1b
#define  REG_AfcLsb              	0x1c
#define  REG_FeiMsb              	0x1d
#define  REG_FeiLsb              	0x1e
#define  REG_PreambleDetect  		0x1f
#define  REG_RxTimeout1       		0x20
#define  REG_RxTimeout2       		0x21
#define  REG_RxTimeout3       		0x22
#define  REG_RxDelay             	0x23
#define  REG_Osc                 	0x24
#define  REG_PreambleMsb      		0x25
#define  REG_PreambleLsb      		0x26
#define  REG_SyncConfig       		0x27
#define  REG_SyncValue1       		0x28
#define  REG_SyncValue2       		0x29
#define  REG_SyncValue3       		0x2a
#define  REG_SyncValue4       		0x2b
#define  REG_SyncValue5       		0x2c
#define  REG_SyncValue6       		0x2d
#define  REG_SyncValue7       		0x2e
#define  REG_SyncValue8       		0x2f
#define  REG_PacketConfig1       	0x30
#define  REG_PacketConfig2       	0x31
#define  REG_PayloadLength       	0x32
#define  REG_NodeAdrs            	0x33
#define  REG_BroadcastAdrs       	0x34
#define  REG_FifoThresh       		0x35
#define  REG_SeqConfig1       		0x36
#define  REG_SeqConfig2       		0x37
#define  REG_TimerResol       		0x38
#define  REG_Timer1Coef       		0x39
#define  REG_SyncWord				0x39
#define  REG_Timer2Coef      		0x3a
#define  REG_ImageCal            	0x3b
#define  REG_Temp                	0x3c
#define  REG_LowBat              	0x3d
#define  REG_IrqFlags1           	0x3e
#define  REG_IrqFlags2           	0x3f
#define  REG_DioMapping1			0x40
#define  REG_DioMapping2			0x41
#define  REG_Version				0x42
#define  REG_PllHop					0x44
#define  REG_PaDac					0x4d
#define  REG_BitRateFrac			0x5d

/* HW Interface  functions ----------------------------------------------------*/

#define SX1278_DELAY(x) 							HAL_Delay(x);

#define SX1278_WRITE_PIN(port, pin, status) 		HAL_GPIO_WritePin(port, pin, status)
#define SX1278_READ_PIN(port, pin) 					HAL_GPIO_ReadPin(port, pin)

#define SX1278_PIN_SET 								GPIO_PIN_SET
#define SX1278_PIN_RESET 							GPIO_PIN_RESET

static void SX1278_SPIBurstRead(SX1278_t *SX1278, uint8_t addr, uint8_t *rxBuf, uint8_t length)
{
	SX1278_WRITE_PIN(SX1278->cs.port, SX1278->cs.pin, SX1278_PIN_RESET);
	while (HAL_SPI_GetState(SX1278->spi) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(SX1278->spi, &addr, 1, 1000);
	HAL_SPI_Receive(SX1278->spi, rxBuf, length, 1000);
	while (HAL_SPI_GetState(SX1278->spi) != HAL_SPI_STATE_READY);
	SX1278_WRITE_PIN(SX1278->cs.port, SX1278->cs.pin, SX1278_PIN_SET);
}

static void SX1278_SPIBurstWrite(SX1278_t *SX1278, uint8_t addr, uint8_t *txBuf, uint8_t length)
{
	SX1278_WRITE_PIN(SX1278->cs.port, SX1278->cs.pin, SX1278_PIN_RESET);
	while (HAL_SPI_GetState(SX1278->spi) != HAL_SPI_STATE_READY);
	addr |= 0x80;
	HAL_SPI_Transmit(SX1278->spi, &addr, 1, 1000);
	HAL_SPI_Transmit(SX1278->spi, txBuf, length, 1000);
	while (HAL_SPI_GetState(SX1278->spi) != HAL_SPI_STATE_READY);
	SX1278_WRITE_PIN(SX1278->cs.port, SX1278->cs.pin, SX1278_PIN_SET);
}

/* Static  functions ----------------------------------------------------------*/

static inline uint8_t SX1278_SPIRead(SX1278_t *SX1278, uint8_t addr)
{
	uint8_t tmp;
	SX1278_SPIBurstRead(SX1278, addr, &tmp, 1);
	return tmp;
}

static inline void SX1278_SPIWrite(SX1278_t *SX1278, uint8_t addr, uint8_t cmd)
{
	SX1278_SPIBurstWrite(SX1278, addr, &cmd, 1);
}

static void SX1278_hw_init(SX1278_t *SX1278)
{
	SX1278_WRITE_PIN(SX1278->cs.port, SX1278->cs.pin, SX1278_PIN_SET);
	SX1278_WRITE_PIN(SX1278->reset.port, SX1278->reset.pin, SX1278_PIN_RESET);
	SX1278_DELAY(200);
	SX1278_WRITE_PIN(SX1278->reset.port, SX1278->reset.pin, SX1278_PIN_SET);
	SX1278_DELAY(200);
}

/* Private  functions ---------------------------------------------------------*/

void SX1278_config(SX1278_t *SX1278)
{
	SX1278_hw_init(SX1278);
	SX1278_sleep(SX1278); //Change modem mode Must in Sleep mode
	SX1278_DELAY(15);

	SX1278_entryLoRa(SX1278);
	//SX1278_SPIWrite(SX1278, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note

	uint64_t freq = ((uint64_t) SX1278->frequency << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	SX1278_SPIBurstWrite(SX1278, LR_REG_FrMsb, (uint8_t*) freq_reg, 3); //setting  frequency parameter

	SX1278_SPIWrite(SX1278, REG_SyncWord, SX1278->syncWord);

	//setting base parameter
	SX1278_SPIWrite(SX1278, LR_REG_PaConfig, SX1278->power); //Setting output power parameter
	SX1278_SPIWrite(SX1278, LR_REG_PADAC, 0x84);	//Normal and RX

	SX1278_SPIWrite(SX1278, LR_REG_Ocp, 0x3B);			//Ocp enabled @ 240mA
	SX1278_SPIWrite(SX1278, LR_REG_Lna, 0x23);		//RegLNA,High & LNA Enable
	if (SX1278->LoRa_SF == SX1278_LORA_SF_6) //SFactor=6
	{
		uint8_t tmp;
		SX1278_SPIWrite(SX1278, LR_REG_ModemConfig1, (SX1278->LoRa_BW + SX1278->LoRa_CR + 0x01)); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(SX1278, LR_REG_ModemConfig2, (SX1278->LoRa_SF + SX1278->LoRa_CRC_sum + 0x03));

		tmp = SX1278_SPIRead(SX1278, 0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX1278_SPIWrite(SX1278, 0x31, tmp);
		SX1278_SPIWrite(SX1278, 0x37, 0x0C);
	}
	else
	{
		SX1278_SPIWrite(SX1278,
		LR_REG_ModemConfig1, (SX1278->LoRa_BW + SX1278->LoRa_CR + 0x00)); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(SX1278,
		LR_REG_ModemConfig2, (SX1278->LoRa_SF + SX1278->LoRa_CRC_sum + 0x00)); //SFactor &  LNA gain set by the internal AGC loop
	}

	SX1278_SPIWrite(SX1278, LR_REG_ModemConfig3, 0x04);
	SX1278_SPIWrite(SX1278, LR_REG_SymbTimeoutLsb, 0x08); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX1278_SPIWrite(SX1278, LR_REG_PreambleMsb, 0x00); //RegPreambleMsb
	SX1278_SPIWrite(SX1278, LR_REG_PreambleLsb, 0x08); //RegPreambleLsb 8+4=12byte Preamble
	SX1278_SPIWrite(SX1278, LR_REG_DIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	SX1278->readBytesNum = 0;
	SX1278_standby(SX1278); //Entry standby mode
}

void SX1278_standby(SX1278_t *SX1278)
{
	SX1278_SPIWrite(SX1278, LR_REG_OpMode, 0x09);
	SX1278->status = SX1278_STANDBY;
}

void SX1278_sleep(SX1278_t *SX1278)
{
	SX1278_SPIWrite(SX1278, LR_REG_OpMode, 0x08);
	SX1278->status = SX1278_SLEEP;
}

void SX1278_entryLoRa(SX1278_t *SX1278)
{
	SX1278_SPIWrite(SX1278, LR_REG_OpMode, 0x88);
}

void SX1278_clearLoRaIrq(SX1278_t *SX1278)
{
	SX1278_SPIWrite(SX1278, LR_REG_IrqFlags, 0xFF);
}

SX1278_retStatus_t SX1278_LoRaEntryRx(SX1278_t *SX1278, uint8_t length, uint32_t timeout)
{
	uint8_t addr;

	SX1278->packetLength = length;

	//SX1278_config(SX1278);		//Setting base parameter
	SX1278_standby(SX1278); //Entry standby mode
	if(SX1278->power < SX1278_POWER_17DBM)
	{
		SX1278_SPIWrite(SX1278, LR_REG_PADAC, 0x84);	//Normal and RX
	}
//	SX1278_SPIWrite(SX1278, LR_REG_HopPeriod, 0xFF);	//No FHSS
	SX1278_SPIWrite(SX1278, LR_REG_DIOMAPPING1, 0x01);//DIO=00,DIO1=00,DIO2=00, DIO3=01
	SX1278_SPIWrite(SX1278, LR_REG_IrqFlagsMask, 0x1F);//Open RxDone interrupt, Timeout, CRC Error
	SX1278_clearLoRaIrq(SX1278);
	SX1278_SPIWrite(SX1278, LR_REG_PayloadLength, length);//Payload Length 21byte(this register must define when the data long of one byte in SF is 6)
	addr = SX1278_SPIRead(SX1278, LR_REG_FifoRxBaseAddr); //Read RxBaseAddr
	SX1278_SPIWrite(SX1278, LR_REG_FifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX1278_SPIWrite(SX1278, LR_REG_OpMode, 0x8d);	//Mode//Low Frequency Mode
	//SX1278_SPIWrite(SX1278, LR_REG_OpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
	SX1278->readBytesNum = 0;

	while (1)
	{
		if ((SX1278_SPIRead(SX1278, LR_REG_ModemStat) & 0x04) == 0x04)
		{	//Rx-on
			SX1278->status = SX1278_RX;
			return SX1278_SUCCESS;
		}
		if (--timeout == 0)
		{
			SX1278_hw_init(SX1278);
			SX1278_config(SX1278);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
}

uint8_t SX1278_LoRaRxPacket(SX1278_t *SX1278)
{
	uint8_t addr;
	uint8_t packet_size;
	SX1278 -> readBytesNum = 0;

	if (SX1278_READ_PIN(SX1278->dio0.port, SX1278->dio0.pin) == SX1278_PIN_RESET)
	{
		return 0;
	}

	if ((SX1278_SPIRead(SX1278, LR_REG_IrqFlags) & 0x60) == 0x40) { //Check RX Done is set and CRC error is not
		memset(SX1278->rxBuffer, 0x00, SX1278_MAX_PACKET);

		addr = SX1278_SPIRead(SX1278, LR_REG_FifoRxCurrentaddr); //last packet addr
		SX1278_SPIWrite(SX1278, LR_REG_FifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if (SX1278->LoRa_SF == SX1278_LORA_SF_6) //When SpreadFactor is six,will used Implicit Header mode(Excluding internal packet length)
		{
			packet_size = SX1278->packetLength;
		}
		else
		{
			packet_size = SX1278_SPIRead(SX1278, LR_REG_RxNbBytes); //Number for received bytes
		}

		SX1278_SPIBurstRead(SX1278, 0x00, SX1278->rxBuffer, packet_size);
		SX1278->readBytesNum = packet_size;
		SX1278_clearLoRaIrq(SX1278);
		return SX1278->readBytesNum;
	}
	else
	{
		SX1278_clearLoRaIrq(SX1278);
		return 0;
	}
}

SX1278_retStatus_t SX1278_LoRaEntryTx(SX1278_t *SX1278, uint8_t length, uint32_t timeout)
{
	uint8_t addr;
	uint8_t temp;

	SX1278->packetLength = length;

	//SX1278_config(SX1278); //setting base parameter
	SX1278_standby(SX1278); //Entry standby mode
	if(SX1278->power < SX1278_POWER_17DBM)
		SX1278_SPIWrite(SX1278, LR_REG_PADAC, 0x87);	//Tx for 20dBm
//	SX1278_SPIWrite(SX1278, LR_REG_HopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1278_SPIWrite(SX1278, LR_REG_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX1278_clearLoRaIrq(SX1278);
	SX1278_SPIWrite(SX1278, LR_REG_IrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1278_SPIWrite(SX1278, LR_REG_PayloadLength, length); //RegPayloadLength 21byte
	addr = SX1278_SPIRead(SX1278, LR_REG_FifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX1278_SPIWrite(SX1278, LR_REG_FifoAddrPtr, addr); //RegFifoAddrPtr

	while (1)
	{
		temp = SX1278_SPIRead(SX1278, LR_REG_PayloadLength);
		if (temp == length)
		{
			SX1278->status = SX1278_TX;
			return SX1278_SUCCESS;
		}

		if (--timeout == 0)
		{
			SX1278_hw_init(SX1278);
			SX1278_config(SX1278);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
}

SX1278_retStatus_t SX1278_LoRaTxPacket(SX1278_t *SX1278, uint8_t *txBuffer, uint8_t length, uint32_t timeout)
{
	SX1278_SPIBurstWrite(SX1278, 0x00, txBuffer, length);
	SX1278_SPIWrite(SX1278, LR_REG_OpMode, 0x8b);	//Tx Mode
	while (1)
	{
		if ((SX1278_READ_PIN(SX1278->dio0.port, SX1278->dio0.pin) == SX1278_PIN_SET) && (SX1278_SPIRead(SX1278, LR_REG_IrqFlags) & 0x08)) //if(Get_NIRQ()) //Packet send over
		{
			SX1278_SPIRead(SX1278, LR_REG_IrqFlags);
			SX1278_clearLoRaIrq(SX1278); //Clear irq
			SX1278_standby(SX1278); //Entry Standby mode
			return SX1278_SUCCESS;
		}

		if (--timeout == 0)
		{
			SX1278_hw_init(SX1278);
			SX1278_config(SX1278);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
}

SX1278_retStatus_t SX1278_transmit(SX1278_t *SX1278, uint8_t *txBuf, uint8_t length, uint32_t timeout)
{
	uint8_t addr;

	SX1278->packetLength = length;

	//SX1278_config(SX1278); //setting base parameter
	SX1278_standby(SX1278); //Entry standby mode
	if(SX1278->power < SX1278_POWER_17DBM)
	{
		SX1278_SPIWrite(SX1278, LR_REG_PADAC, 0x87);	//Tx for 20dBm
	}
//	SX1278_SPIWrite(SX1278, LR_REG_HopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1278_SPIWrite(SX1278, LR_REG_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX1278_clearLoRaIrq(SX1278);
	SX1278_SPIWrite(SX1278, LR_REG_IrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1278_SPIWrite(SX1278, LR_REG_PayloadLength, length); //RegPayloadLength 21byte
	addr = SX1278_SPIRead(SX1278, LR_REG_FifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX1278_SPIWrite(SX1278, LR_REG_FifoAddrPtr, addr); //RegFifoAddrPtr
	SX1278_SPIBurstWrite(SX1278, 0x00, txBuf, length);
	SX1278_SPIWrite(SX1278, LR_REG_OpMode, 0x8b);	//Tx Mode
	SX1278->status = SX1278_TX;
	while (1)
	{
		if ((SX1278_READ_PIN(SX1278->dio0.port, SX1278->dio0.pin) == SX1278_PIN_SET) && (SX1278_SPIRead(SX1278, LR_REG_IrqFlags) & 0x08)) //if(Get_NIRQ()) //Packet send over
		{
			SX1278_clearLoRaIrq(SX1278); //Clear irq
			SX1278_standby(SX1278); //Entry Standby mode
			return SX1278_SUCCESS;
		}

		if (--timeout == 0)
		{
			SX1278_hw_init(SX1278);
			SX1278_config(SX1278);
			return SX1278_TIMEOUT;
		}
		SX1278_DELAY(1);
	}
	return SX1278_ERROR;
}

uint8_t SX1278_read(SX1278_t *SX1278, uint8_t *rxBuf, uint8_t length)
{
	if (length != SX1278->readBytesNum)
	{
		length = SX1278->readBytesNum;
	}
	memcpy(rxBuf, SX1278->rxBuffer, length);
	rxBuf[length] = '\0';
	SX1278->readBytesNum = 0;
	return length;
}

uint8_t SX1278_RSSI_LoRa(SX1278_t *SX1278)
{
	uint32_t temp = 10;
	temp = SX1278_SPIRead(SX1278, LR_REG_RssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset
	return (uint8_t) temp;
}

uint8_t SX1278_RSSI(SX1278_t *SX1278)
{
	uint8_t temp = 0xff;
	temp = SX1278_SPIRead(SX1278, REG_RssiValue);
	temp = 127 - (temp >> 1);	//127:Max RSSI
	return temp;
}
