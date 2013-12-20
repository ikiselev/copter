//
// Created by Alabay on 11.12.13.
//

#include <avr/eeprom.h>
#include "SDLogger.h"


/** Set SCK to max rate of F_CPU/2. See Sd2Card::setSckRate(). */
uint8_t const SPI_FULL_SPEED = 0;
/** Set SCK rate to F_CPU/4. See Sd2Card::setSckRate(). */
uint8_t const SPI_HALF_SPEED = 1;
/** Set SCK rate to F_CPU/8. Sd2Card::setSckRate(). */
uint8_t const SPI_QUARTER_SPEED = 2;

bool errorCaused = false;
const char * UNIQUE_DELIMITER = "$$\0";
const char * END_DELIMITER = "\n\0";
const char * VALUE_SEPARATOR = ",";


SDLogger Logger;



static void spiSend(uint8_t b) {
    SPDR = b;
    while (!(SPSR & (1 << SPIF)));
}
/** Receive a byte from the card */
static  uint8_t spiRec(void) {
    spiSend(0XFF);
    return SPDR;
}


boolean SDLogger::begin(uint8_t loggerType)
{
    this->loggerType = loggerType;
    /**
     * TODO: autodetect?
     */
    if(loggerType == LOGGER_SD_CARD)
    {
        int address = 1;
        this->logUniqueNumber = eeprom_read_byte((unsigned char *) address);
        eeprom_write_byte((unsigned char *) address, (this->logUniqueNumber > 250) ? (uint8_t)0x0 : this->logUniqueNumber + 1);
        sdCardInited = initCard();
    }
    else if(loggerType == LOGGER_SERIAL)
    {
        sdCardInited = true;
    }
    return sdCardInited;
}

boolean SDLogger::initCard()
{
    errorCode_ = type_ = 0;
    chipSelectPin_ = SS_PIN;


    uint16_t t0 = (uint16_t)millis();
    uint32_t arg;

    initSpi();

    // command to go idle in SPI mode
    while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
        if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
            error(SD_CARD_ERROR_CMD0);
            goto fail;
        }
    }
    // check SD version
    if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
        type(SD_CARD_TYPE_SD1);
    } else {
        // only need last byte of r7 response
        for (uint8_t i = 0; i < 4; i++) status_ = spiRec();
        if (status_ != 0XAA) {
            error(SD_CARD_ERROR_CMD8);
            goto fail;
        }
        type(SD_CARD_TYPE_SD2);
    }
    // initialize card and send host supports SDHC if SD2
    arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;

    while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
        // check for timeout
        if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
            error(SD_CARD_ERROR_ACMD41);
            goto fail;
        }
    }
    // if SD2 read OCR register to check for SDHC card
    if (type() == SD_CARD_TYPE_SD2) {
        if (cardCommand(CMD58, 0)) {
            error(SD_CARD_ERROR_CMD58);
            goto fail;
        }
        if ((spiRec() & 0XC0) == 0XC0) type(SD_CARD_TYPE_SDHC);
        // discard rest of ocr - contains allowed voltage range
        for (uint8_t i = 0; i < 3; i++) spiRec();
    }
    chipSelectHigh();


    return setSckRate(SPI_HALF_SPEED);

    fail:
    chipSelectHigh();
    return false;
}


uint8_t SDLogger::waitNotBusy(uint16_t timeoutMillis) {
    uint16_t t0 = millis();
    do {
        if (spiRec() == 0XFF) return true;
    }
    while (((uint16_t)millis() - t0) < timeoutMillis);
    return false;
}

uint8_t SDLogger::cardCommand(uint8_t cmd, uint32_t arg) {

    // select card
    chipSelectLow();

    // wait up to 300 ms if busy
    waitNotBusy(300);

    // send command
    spiSend(cmd | 0x40);

    // send argument
    for (int8_t s = 24; s >= 0; s -= 8) spiSend(arg >> s);

    // send CRC
    uint8_t crc = 0XFF;
    if (cmd == CMD0) crc = 0X95;  // correct crc for CMD0 with arg 0
    if (cmd == CMD8) crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
    spiSend(crc);

    // wait for response
    for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++);
    return status_;
}


uint8_t SDLogger::setSckRate(uint8_t sckRateID) {
    // see avr processor datasheet for SPI register bit definitions
    if ((sckRateID & 1) || sckRateID == 6) {
        SPSR &= ~(1 << SPI2X);
    } else {
        SPSR |= (1 << SPI2X);
    }
    SPCR &= ~((1 <<SPR1) | (1 << SPR0));
    SPCR |= (sckRateID & 4 ? (1 << SPR1) : 0)
            | (sckRateID & 2 ? (1 << SPR0) : 0);
    return true;
}

void SDLogger::initSpi(void)
{
    // set pin modes
    pinMode(chipSelectPin_, OUTPUT);
    chipSelectHigh();
    pinMode(MISO_PIN, INPUT);
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(SCK_PIN, OUTPUT);


    // SS must be in output mode even it is not chip select
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH); // disable any SPI device using hardware SS pin
    // Enable SPI, Master, clock rate f_osc/128
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
    // clear double speed
    SPSR &= ~(1 << SPI2X);


    // must supply min of 74 clock cycles with CS high.
    for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);

    chipSelectLow();
}


//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t SDLogger::writeBlock(uint32_t blockNumber, const uint8_t* src, uint8_t significantBytes) {
#if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
#endif  // SD_PROTECT_BLOCK_ZERO

    // use address if not SDHC card
    if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
    if (cardCommand(CMD24, blockNumber)) {
        error(SD_CARD_ERROR_CMD24);
        goto fail;
    }
    if (!writeData(DATA_START_BLOCK, src, significantBytes)) goto fail;

    // wait for flash programming to complete
    if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
        error(SD_CARD_ERROR_WRITE_TIMEOUT);
        goto fail;
    }
    // response is r2 so get and check two bytes for nonzero
    if (cardCommand(CMD13, 0) || spiRec()) {
        error(SD_CARD_ERROR_WRITE_PROGRAMMING);
        goto fail;
    }
    chipSelectHigh();
    return true;

    fail:
    chipSelectHigh();
    return false;
}

//------------------------------------------------------------------------------
// send one block of data for write block or write multiple blocks
uint8_t SDLogger::writeData(uint8_t token, const uint8_t* src, uint8_t significantBytes) {

    // send data - optimized loop
    SPDR = token;

    // send two byte per iteration
    for (uint16_t i = 0; i < significantBytes; i++) {
        while (!(SPSR & (1 << SPIF)));
        SPDR = src[i];
        //while (!(SPSR & (1 << SPIF)));
        //SPDR = src[i+1];
    }

    /**
     * 512 bytes must be written
     */
    for (uint16_t i = 0; i < 512 - significantBytes; i++) {
        while (!(SPSR & (1 << SPIF)));
        SPDR = 0;
        //while (!(SPSR & (1 << SPIF)));
        //SPDR = 0;
    }

    // wait for last data byte
    while (!(SPSR & (1 << SPIF)));

    spiSend(0xff);  // dummy crc
    spiSend(0xff);  // dummy crc

    status_ = spiRec();
    if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
        error(SD_CARD_ERROR_WRITE);
        chipSelectHigh();
        return false;
    }
    return true;
}

void SDLogger::chipSelectHigh(void) {
    digitalWrite(chipSelectPin_, HIGH);
}
//------------------------------------------------------------------------------
void SDLogger::chipSelectLow(void) {
    digitalWrite(chipSelectPin_, LOW);
}


void SDLogger::log(String str, bool endOfLine)
{
    if(!sdCardInited)
    {

        return ;
    }

    if(startWithNumber)
    {
        /**
         * Уникальный номер записи, по которому можно определить конец данных
         */
        logUniqueNumber++;

        /**
         * Перевод числа в строку
         */
        char * number = NULL;
        int numberLen = 5 + 1; //65535 has 5 chars + 1 null-terminated
        number = (char *) malloc(sizeof(char) * numberLen);
        sprintf(number, "%d", logUniqueNumber);

        /**
        * Объединение строк в одну
        */
        strcat(buffer, number);
        free( number );
        strcat(buffer, UNIQUE_DELIMITER);
        startWithNumber = false;
    }

    strcat(buffer, str.c_str());

    if(endOfLine)
    {
        strcat(buffer, END_DELIMITER);

        //TODO: need refactoring
        startWithNumber = (loggerType == LOGGER_SD_CARD);


        /**
         * Flush only if end of line
         */
        if(loggerType == LOGGER_SD_CARD)
        {
            messagesCounter++;
            if(messagesCounter >= MESSAGES_COUNT_FLUSH)
            {
                flush(buffer);
                messagesCounter = 0;
            }
        }
        else
        {
            flush(buffer);
        }
    }


}

void SDLogger::flush(char * source)
{
    if(loggerType == LOGGER_SD_CARD)
    {
        if(writeBlock(currentBlock, (uint8_t *)source, (uint8_t)strlen(source)))
        {
            //Serial.print(".");
            currentBlock++;
        }
        else
        {
            //Serial.print("Error: ");
            //Serial.println(errorCode_, HEX);
        }
    }
    else if(loggerType == LOGGER_SERIAL)
    {
        Serial.print(source);
    }

    source[0] = '\0';
}


void SDLogger::log(String columnName, double value, bool endOfLine)
{
    if(!sdCardInited)
    {

        return ;
    }

    if(!columnNamesInited)
    {
        if(isFirstColumn)
        {
            isFirstColumn = false;
            String header = String("Columns:");
            header += columnName;
            columnName = header;
        }

        headerColumns.concat(columnName);

        if(endOfLine)
        {
            columnNamesInited = true;

            headerColumns.concat(END_DELIMITER);

            /**
             * Flush headers manually
             */
            flush((char *)headerColumns.c_str());

            headerColumns = NULL;
        }
        else
        {
            /**
             * Если строка не последняя, то добавляем запятую
             */
            headerColumns.concat(VALUE_SEPARATOR);
        }
    }
    else
    {
        String tempValue = String((long)value);


        if(!endOfLine)
        {
            tempValue.concat(VALUE_SEPARATOR);
        }
        else
        {
            tempValue.concat("|");
            tempValue.concat(millis());
        }

        log(tempValue, endOfLine);
    }
}
