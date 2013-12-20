/**
 * Константы для класса Logger
 */
#define LOGGER_NONE 0
#define LOGGER_SD_CARD 1
#define LOGGER_SERIAL 2


#define DEBUG_ENABLE 0



#if DEBUG_ENABLE
    inline void debug(const char *s) { Serial.println(s); }
    inline void debug(const char *s, char *t) { Serial.print(s);Serial.println(t); }
#else
inline void debug(const char *s) {  }
inline void debug(const char *s, const char *t) { }
#endif