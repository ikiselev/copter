#ifndef __CONSTANTS_
#define __CONSTANTS_

/**
 * Какой из типов включен
 */
#define LOGGER_SD_CARD 1
#define LOGGER_SERIAL 0


#define DEBUG_ENABLE 1
#define DEBUG_SENSORS_ENABLE 0

#ifdef MAIN_FILE
char temp_buffer[40];
#else
extern char temp_buffer[40];
#endif

#define P(str) (strcpy_P(temp_buffer, PSTR(str)), temp_buffer)


#if DEBUG_ENABLE
    #include <avr/pgmspace.h>

    inline void debug(const char *s) { Serial.println(s); }
    inline void debug(const char *s, char *t) { Serial.print(s);Serial.println(t); }
    inline void debug(const char *s, float t) { Serial.print(s);Serial.println(t); }
#else
    inline void debug(const char *s) {  }
    inline void debug(const char *s, const char *t) { }
    inline void debug(const char *s, float t) { }
#endif


#endif