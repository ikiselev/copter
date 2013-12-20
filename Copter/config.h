#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "constants.h"

struct Config
{
    /**
     * Тип логгера
     */
    static const uint8_t loggerType = LOGGER_SERIAL;
    /**
     * SD Card Start block
     */
    static const uint32_t sdCardStartBlock_config = 1025;

    /**
     * Возможность задать время полета
     * В миллисекундах
     * 0 - не выключаться
     */
    static const int flightTime = 8000;


    /**
     * Ожидание перед началом стабилизации.
     * После того как подключено питание стабилизация
     * начнется не сразу, а после этого времени.
     */
    static const int heatUpTime = 3000;


    /**
     * Возможность задать силу тяги
     * От 0 до 255
     */
    double xSpeed = 40;
    double ySpeed = 40;

    /**
     * ПИД максимальные значения (+-)
     */
    static const double pidOutputLimits = 90;


    /**
     * Fail-safe
     * Отключение моторов, если угл наклона превысил допустимый предел
     */
    static const int failsafeAngle = 90;


    /**
     * Калибровка угла плоскости припаянной инерциальной сборки на коптере
     * по отношению к плоскости коптера.
     * На ровной поверхности
     * X: 181
     * Y:177
     */
    static const int xOffsetIMU = -1;
    static const int yOffsetIMU = +3;


    /**
     * Цель для ПИД-регулятора стремиться к этим значениям
     * чтобы сохранить свое положение в пространстве.
     */
    double targetAngleX = 180.0;
    double targetAngleY = 180.0;


    /**
     * Настройка выводов моторов
     */
    static const uint8_t esc_x1_pin = 3;
    static const uint8_t esc_x2_pin = 6;
    //Y-axis
    static const uint8_t esc_y1_pin = 5;
    static const uint8_t esc_y2_pin = 9; //Pin with led

    /**
     * Вывод МК на светодиод.
     * К сожалению, в данном коптере он подключен к мотору
     */
    static const int ledPin = esc_y2_pin;
};

extern Config config;

#endif