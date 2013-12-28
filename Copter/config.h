#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "Arduino.h"
#include "constants.h"

struct Config
{
    /**
     * SD Card Start block
     */
    static const uint32_t sdCardStartBlock_config = 1025;

    /**
     * Возможность задать время полета
     * В миллисекундах
     * 0 - не выключаться
     */
    static const uint32_t flightTime = 2000;
    /**
     * Время постепенного взлета.
     * Время набирания оборотов с xSpeedStart до xSpeed
     */
    static const uint32_t takeoffTime = 1500;

    /**
     * Проверять ли напряжение на батарейке.
     * Полезно при отладке только по Serial ставить в false
     */
    static const bool checkBatteryAtStartup = true;

    /**
     * Ожидание перед началом стабилизации.
     * После того как подключено питание стабилизация
     * начнется не сразу, а после этого времени.
     */
    static const uint32_t heatUpTime = 3000;


    /**
     * Возможность задать силу тяги
     * От 0 до 255
     */
    double xSpeed = 190;
    double ySpeed = 190;

    double xSpeedStart = 55;
    double ySpeedStart = 55;

    /**
     * ПИД максимальные значения (+-)
     */
    static const float pidOutputLimits = 90;


    /**
     * TODO: Fail-safe
     * Отключение моторов, если угл наклона превысил допустимый предел
     */
    static const int failsafeAngle = 90;


    /**
     * Калибровка угла плоскости припаянной инерциальной сборки на коптере
     * по отношению к плоскости коптера.
     */
    static const int xOffsetIMU = 0;
    static const int yOffsetIMU = 0;


    /**
     * Цель для ПИД-регулятора стремиться к этим значениям
     * чтобы сохранить свое положение в пространстве.
     */
    float targetAngleX = 180.0;
    float targetAngleY = 180.0;


    /**
     * Настройка выводов моторов
     */
    static const uint8_t esc_x1_pin = 5;
    static const uint8_t esc_x2_pin = 9;
    //Y-axis
    static const uint8_t esc_y1_pin = 3;
    static const uint8_t esc_y2_pin = 6; //Pin with led

    /**
     * Вывод МК на светодиод.
     * К сожалению, в данном коптере он подключен к мотору
     */
    static const int ledPin = 9;

    static const uint8_t batteryAnalogPing = 1;
    static const uint16_t batteryFull = 690; //4.2V - 0.8v drop-down (170)
    static const uint16_t batteryLow = 608; // 3.8V - 0.8v drop-down
    //static const uint8_t batteryLow = 500; // 3.3V - 0.8v drop-down
};

extern Config config;

#endif