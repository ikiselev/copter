//
// Created by Alabay on 11.12.13.
//

#include "SDLogger.h"


/** Set SCK to max rate of F_CPU/2. See Sd2Card::setSckRate(). */
uint8_t const SPI_FULL_SPEED = 0;
/** Set SCK rate to F_CPU/4. See Sd2Card::setSckRate(). */
uint8_t const SPI_HALF_SPEED = 1;
/** Set SCK rate to F_CPU/8. Sd2Card::setSckRate(). */
uint8_t const SPI_QUARTER_SPEED = 2;

bool errorCaused = false;
const int SECTOR_SIZE = 512;
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


boolean SDLogger::begin(uint8_t loggerType, uint16_t logUniqueNumber)
{
    this->loggerType = loggerType;
    this->logUniqueNumber = logUniqueNumber;
    if(loggerType == LOGGER_SD_CARD)
    {
        sdCardInited = initCard();
    }
    else if(loggerType == LOGGER_SERIAL)
    {
        Serial.begin(115200);
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
uint8_t SDLogger::writeBlock(uint32_t blockNumber, const uint8_t* src) {
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
    if (!writeData(DATA_START_BLOCK, src)) goto fail;

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
uint8_t SDLogger::writeData(uint8_t token, const uint8_t* src) {

    // send data - optimized loop
    SPDR = token;

    // send two byte per iteration
    for (uint16_t i = 0; i < 512; i += 2) {
        while (!(SPSR & (1 << SPIF)));
        SPDR = src[i];
        while (!(SPSR & (1 << SPIF)));
        SPDR = src[i+1];
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

    if(errorCaused)
    {
        Serial.print("MAEC"); //Memory allocation error caused
        return;
    }

    /**
     * Подсчет кол-ва места необходимого для выделения лог-строки
     */
    char const * temp = str.c_str();
    int buffer_length = strlen(temp);

    char * number = NULL;
    if(startWithNumber)
    {
        /**
         * Уникальный номер записи, по которому можно определить конец данных
         */
        logUniqueNumber++;

        /**
         * Перевод числа в строку
         */
        int numberLen = 5 + 1; //65535 has 5 chars + 1 null-terminated
        number = (char *) malloc(sizeof(char) * numberLen);
        sprintf(number,"%d",logUniqueNumber);

        buffer_length += strlen(number) + strlen(UNIQUE_DELIMITER);
    }


    buffer_length += 1; //null termination
    if(endOfLine)
    {
        buffer_length += strlen(END_DELIMITER);
    }
    int lastChar = 0;

    if(buffer == NULL)
    {
        /**
        * Первый раз выделяем память
        */
        buffer = (char *) malloc(sizeof(char) * buffer_length );
        if(buffer == NULL)
        {
            errorCaused = true;
            return;
        }
    }
    else
    {
        /**
        * Перераспределяем пространство под новую длину буфера
        */
        buffer_length += strlen(buffer);
        buffer = (char *) realloc(buffer, sizeof(char) * buffer_length );
        lastChar = buffer_length;
        if(buffer == NULL)
        {
            errorCaused = true;
            return;
        }
    }

    buffer[lastChar] = '\0';


    /**
    * Объединение строк в одну
    */
    if(startWithNumber)
    {
        strcat(buffer, number);
        free( number );
        strcat(buffer, UNIQUE_DELIMITER);
        startWithNumber = false;
    }
    strcat(buffer, temp);

    if(endOfLine)
    {
        strcat(buffer, END_DELIMITER);

        //TODO: need refactoring
        startWithNumber = (loggerType == LOGGER_SD_CARD);
    }

    if(loggerType == LOGGER_SD_CARD)
    {
        flushSdCard(buffer_length);
    }
    else if(loggerType == LOGGER_SERIAL)
    {
        if(endOfLine)
        {
            /**
             * Flush only if string is ready
             */
            flushSerial();
        }
    }
}

void SDLogger::flushSdCard(int buffer_length)
{
    //TODO: maybe buffer_length instead of strlen?
    while(strlen(buffer) >= SECTOR_SIZE)
    {
        /**
        * writeBlock не запишет больше 512 байт (блок) из buffer. Это нам и нужно
        */
        if(writeBlock(currentBlock, (unsigned char*)buffer))
        {
            //Serial.print(".");
            currentBlock++;
        }
        else
        {
            //Serial.print(":( ");
        }

        int offsetLen = buffer_length - SECTOR_SIZE;


        buffer = (char *) memmove(buffer, buffer + SECTOR_SIZE, offsetLen);
        buffer = (char *) realloc(buffer, sizeof(char)*(offsetLen));
        if(buffer == NULL)
        {
            errorCaused = true;
            return;
        }

        buffer[offsetLen] = '\0';
    }
}

void SDLogger::flushSerial()
{
    Serial.print(buffer);

    /**
     * Очищаем буфер для следующих строк
     */
    free(buffer);
    buffer = NULL;
}


void SDLogger::log(String columnName, double value, bool endOfLine)
{
    if(!sdCardInited)
    {

        return ;
    }

    String tempValue = String((long)value);

    int millisLen = 0;

    String millisBetweenPack;
    if(endOfLine)
    {
        millisBetweenPack = "|";
        millisBetweenPack.concat(millis());

        millisLen = millisBetweenPack.length();
    }


    if(!columnNamesInited)
    {
        int lastChar = 0;
        int comaLen = (endOfLine) ? 0 : 1;


        if(firstDataLineBuffer == NULL)
        {
            firstDataLineBuffer = (char *) malloc(sizeof(char) * tempValue.length() + 1 + comaLen + millisLen); //null-term + ","
        }
        else
        {
            int len = strlen(firstDataLineBuffer) + tempValue.length() + 1 + comaLen + millisLen;
            lastChar = len;
            firstDataLineBuffer = (char *) realloc(firstDataLineBuffer, sizeof(char)*(len));
        }
        firstDataLineBuffer[lastChar] = '\0';

        char const * temp = tempValue.c_str();
        strcat(firstDataLineBuffer, temp);


        if(isFirstColumn)
        {
            isFirstColumn = false;
            String header = String("Columns:");
            header += columnName;
            columnName = header;
        }

        if(!endOfLine)
        {
            /**
             * Если строка не последняя, то добавляем запятую
             */
            strcat(firstDataLineBuffer, VALUE_SEPARATOR);


            columnName.concat(VALUE_SEPARATOR);
        }

        /**
         * Накапливаем колонки
         */
        log(columnName, endOfLine);
        if(endOfLine)
        {
            columnNamesInited = true;

            char const * millistemp = millisBetweenPack.c_str();
            strcat(firstDataLineBuffer, millistemp);

            log(firstDataLineBuffer, endOfLine);
            free(firstDataLineBuffer);
        }
    }
    else
    {
        if(!endOfLine)
        {
            tempValue.concat(VALUE_SEPARATOR);
        }
        else
        {
            tempValue.concat(millisBetweenPack);
        }

        log(tempValue, endOfLine);
    }
}
