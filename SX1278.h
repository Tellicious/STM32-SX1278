/* BEGIN Header */
/*!
 ******************************************************************************
 * @file    SX1278.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SX1278_H__
#define __SX1278_H__

#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Macros --------------------------------------------------------------------*/

#define SX1278_MAX_PACKET			256
#define SX1278_DEFAULT_TIMEOUT		2000

#define SX1278_POWER_20DBM			0xFF
#define SX1278_POWER_17DBM			0xFC
#define SX1278_POWER_14DBM			0xF9
#define SX1278_POWER_11DBM			0xF6

#define SX1278_LORA_SF_6			0x60
#define SX1278_LORA_SF_7			0x70
#define SX1278_LORA_SF_8			0x80
#define SX1278_LORA_SF_9			0x90
#define SX1278_LORA_SF_10			0x100
#define SX1278_LORA_SF_11			0x110
#define SX1278_LORA_SF_12			0x120

#define	SX1278_LORA_BW_7_8KHZ		0x00
#define	SX1278_LORA_BW_10_4KHZ		0x10
#define	SX1278_LORA_BW_15_6KHZ		0x20
#define	SX1278_LORA_BW_20_8KHZ		0x30
#define	SX1278_LORA_BW_31_2KHZ		0x40
#define	SX1278_LORA_BW_41_7KHZ		0x50
#define	SX1278_LORA_BW_62_5KHZ		0x60
#define	SX1278_LORA_BW_125KHZ		0x70
#define	SX1278_LORA_BW_250KHZ		0x80
#define	SX1278_LORA_BW_500KHZ		0x90

//Coding rate
#define SX1278_LORA_CR_4_5    		0x02
#define SX1278_LORA_CR_4_6   		0x03
#define SX1278_LORA_CR_4_7    		0x06
#define SX1278_LORA_CR_4_8    		0x08

//CRC Enable
#define SX1278_LORA_CRC_EN          0x04
#define SX1278_LORA_CRC_DIS         0x00


/* Typedefs ------------------------------------------------------------------*/

/*
* SX1278 module status
*/
typedef enum {
	SX1278_SLEEP = 0,
	SX1278_STANDBY = 1,
	SX1278_TX = 2,
	SX1278_RX = 3
} SX1278_status_t;

/*
* SX1278 return status
*/
typedef enum {
	SX1278_SUCCESS = 0,
	SX1278_ERROR = 1,
	SX1278_TIMEOUT = 2
} SX1278_retStatus_t;

/*
* SX1278 HW pins
*/
typedef struct {
	int pin;
	void *port;
} SX1278_HWPin_t;

/*
* SX1278 struct
*/
typedef struct {
	SX1278_HWPin_t reset;
	SX1278_HWPin_t dio0;
	SX1278_HWPin_t cs;
	void *spi;
	uint64_t frequency;
	uint8_t power, LoRa_SF, LoRa_BW, LoRa_CR, LoRa_CRC_sum, payloadSize, syncWord, availableBytes;
	SX1278_status_t status;
} SX1278_t;


/* Function prototypes --------------------------------------------------------*/

/*!
 * @brief Configure LoRa module according to parameters stored in module structure
 *
 * @param[in] SX1278	pointer to SX1278 structure
 */
void SX1278_init(SX1278_t *SX1278);

/*!
 * @brief Entry LoRa mode
 *
 * @param[in] SX1278	pointer to SX1278 structure
 */
void SX1278_entryLoRa(SX1278_t *SX1278);

/*!
 * @brief Clear LoRa interrupt flags
 *
 * @param[in] SX1278	pointer to SX1278 structure
 */
void SX1278_clearLoRaIrq(SX1278_t *SX1278);

/*!
 * @brief Entry reception mode
 *
 * @param[in] SX1278	pointer to SX1278 structure
 * @param[in] length    length of message to be received
 * @param[in] timeout   timeout in [ms]
 *
 * @return SX1278_SUCCESS if entry into RX mode is successful, SX1278_TIMEOUT if timeout was reached
 */
SX1278_retStatus_t SX1278_LoRaStartReceiver(SX1278_t *SX1278, uint8_t length, uint32_t timeout);

/*!
 * @brief Read data
 *
 * Read data and return it via buffer pointer
 *
 * @param[in] SX1278	pointer to SX1278 structure
 * @param[out] buffer   pointer to data to be read
 * @param[in] length    length of data to be read
 *
 * @return returns number of read bytes
 */
uint8_t SX1278_read(SX1278_t *SX1278, uint8_t *buffer, uint8_t length);

/*!
 * @brief Entry transmitter mode
 *
 * Entry transmitter mode
 *
 * @param[in] SX1278	pointer to SX1278 structure
 * @param[in] length    length of message to be sent
 * @param[in] timeout   timeout in [ms]
 *
 * @return SX1278_SUCCESS if entry into TX mode is successful, SX1278_TIMEOUT if timeout was reached
 */
SX1278_retStatus_t SX1278_LoRaStartTransmitter(SX1278_t *SX1278, uint8_t length, uint32_t timeout);

/*!
 * @brief Transmit data
 *
 * @param[in] SX1278	pointer to SX1278 structure
 * @param[in] buffer    pointer to data to be sent
 * @param[in] length    length of data to be sent
 * @param[in] timeout   timeout in [ms]
 *
 * @return SX1278_SUCCESS if data is successfully sent, SX1278_TIMEOUT if timeout was reached
 */
SX1278_retStatus_t SX1278_LoRaTxPacket(SX1278_t *SX1278, uint8_t *buffer, uint8_t length, uint32_t timeout);

/*!
 * @brief Entry TX mode and send data, combination of SX1278_LoRaStartTransmitter() and SX1278_LoRaTxPacket()
 *
 * @param[in] SX1278	pointer to SX1278 structure
 * @param[in] buffer     pointer to data to be sent
 * @param[in] length    length of data to be sent
 * @param[in] timeout   timeout in [ms]
 *
 * @return SX1278_SUCCESS if data is successfully sent, SX1278_TIMEOUT if timeout was reached
 */
SX1278_retStatus_t SX1278_write(SX1278_t *SX1278, uint8_t *buffer, uint8_t length, uint32_t timeout);

/*!
 * @brief Returns RSSI (LoRa mode)
 *
 * @param[in] SX1278	pointer to SX1278 structure
 *
 * @return RSSI value
 */
uint8_t SX1278_RSSI_LoRa(SX1278_t *SX1278);

/*!
 * @brief Returns RSSI (general mode)
 *
 * @param[in] SX1278	pointer to SX1278 structure
 *
 * @return RSSI value
 */
uint8_t SX1278_RSSI(SX1278_t *SX1278);

/*!
 * @brief Enter standby mode
 *
 * @param[in] SX1278	Pointer to LoRa structure
 */
void SX1278_standby(SX1278_t *SX1278);

/*!
 * @brief Enter sleep mode
 *
 * @param[in] SX1278	Pointer to LoRa structure
 */
void SX1278_sleep(SX1278_t *SX1278);

#ifdef __cplusplus
}
#endif

#endif // __SX1278_H__
