# Semtech SX1278 driver for STM32

Example usage:
- Initialization:
    ```cpp
    /* initialize LoRa module */
        SX1278.dio0.port = LoRaInt_GPIO_Port;
        SX1278.dio0.pin = LoRaInt_Pin;
        SX1278.cs.port = LoRaCS_GPIO_Port;
        SX1278.cs.pin = LoRaCS_Pin;
        SX1278.reset.port = LoRaReset_GPIO_Port;
        SX1278.reset.pin = LoRaReset_Pin;
        SX1278.spi = &hspi1;
        SX1278.frequency = 433000000;
        SX1278.power = SX1278_POWER_17DBM;
        SX1278.LoRa_SF = SX1278_LORA_SF_9;
        SX1278.LoRa_BW = SX1278_LORA_BW_250KHZ;
        SX1278.LoRa_CR = SX1278_LORA_CR_4_6;
        SX1278.LoRa_CRC_sum = SX1278_LORA_CRC_EN;
        SX1278.syncWord = cfgSYNC_WORD;
        SX1278.payloadSize = 5;

        SX1278_init(&SX1278);

        /* Always start in receiver mode */
        if (SX1278_LoRaStartReceiver(&SX1278, 1, 2000) != SX1278_SUCCESS)
        {
            while(1);
        }
    ```
- Receive data:
    ```cpp
    uint8_t rxBuffer[SX1278.payloadSize];

    toBeReadCounter = SX1278_read(SX1278, rxBuffer, SX1278.payloadSize);
    if (toBeReadCounter)
    {
        /* do something with received data */
    }
    ```
- Transmit data:
    ```cpp
    uint8_t txBuffer[SX1278.payloadSize];

    if (SX1278_write(&SX1278, txBuffer, SX1278.payloadSize, 500) == SX1278_SUCCESS)
    {
        /* data sent succesfully */
    }

    /* re-enter in RX mode */
    while(SX1278_LoRaStartReceiver(&SX1278, 1, 500) != SX1278_SUCCESS);
    /* always remember to clear interrupts to reset errors */
    SX1278_clearLoRaIrq(&SX1278);
    ```