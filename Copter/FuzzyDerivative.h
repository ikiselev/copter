//
// Created by Alabay on 26.12.13.
//


#ifndef __FuzzyDerivative_H_
#define __FuzzyDerivative_H_
#include "Arduino.h"

/**
 * TODO: hardcode magic numbers
 * Соглашение: каждое правило состоит из двух значений термов переменных
 * Первый терм: Ошибка рассогласования
 * Второй: угловая скорость
 */
typedef struct condition { uint8_t set[2]; } condition;


typedef struct rule
{
    int conditionCount;
    condition * conditions;
} Rule;

class FuzzyDerivative
{
    /**
     * Таблица правил
     */
    const uint8_t inputVariablesCount = 2;

    //4 переменных для задания трапеции
    #define TRAPEZODIAL 4

    #define VAR_ERROR 1
    #define VAR_OMEGA 2

    /**
     * Ошибка рассогласования
     * {{трапеция "низкое"}, {трапеция "среднее"}, {трапеция "высокое"}, {трапеция "огромное"}}
     */
    static const int input_error_term_count = 4;

    static const uint8_t ERROR_LOW = 0;
    static const uint8_t ERROR_MID = 1;
    static const uint8_t ERROR_HIGH = 2;
    static const uint8_t ERROR_VHIGH = 3;

    //int input_error[input_error_term_count][TRAPEZODIAL];
    float input_error[input_error_term_count][TRAPEZODIAL] = {
        {0.0, 0.0, 2.0, 9.0}, //ERROR_LOW
        {8.0, 12.0, 14.0, 15.0}, //ERROR_MID
        {14.0, 16.0, 17.0, 22.0}, //ERROR_HIGH
        {20.0, 40.0, 90.0, 90.0}, //ERROR_VHIGH
    };


    /**
     * Угловая скорость
     */
    static const int input_omega_term_count = 5;

    static const uint8_t OMEGA_VERY_LOW = 0;
    static const uint8_t OMEGA_LOW = 1;
    static const uint8_t OMEGA_MID = 2;
    static const uint8_t OMEGA_HIGH = 3;
    static const uint8_t OMEGA_VHIGH = 4;

    //int input_omega[input_omega_term_count][TRAPEZODIAL];
    float input_omega[input_omega_term_count][TRAPEZODIAL] = {
        {0.0, 0.0, 50.0, 190.0}, //OMEGA_VERY_LOW
        {160.0, 220.0, 260.0, 290.0}, //OMEGA_LOW
        {280.0, 350.0, 500.0, 570.0}, //OMEGA_MID
        {550.0, 580.0, 800.0, 900.0}, //OMEGA_HIGH
        {800.0, 1000.0, 2020.0, 2020.0} //OMEGA_VHIGH
    };


    /**
     * Описание термов выходной переменной
     */
    static const int output_term_count = 4;

    static const uint8_t OUTPUT_LOW = 0;
    static const uint8_t OUTPUT_SMALL = 1;
    static const uint8_t OUTPUT_MID = 2;
    static const uint8_t OUTPUT_HIGH = 3;

    //float output[output_term_count][TRAPEZODIAL];
    float output[output_term_count][TRAPEZODIAL] = {
            {0.3, 0.3, 0.35, 0.4}, //OUTPUT_LOW
            {0.35, 0.4, 0.55, 0.6}, //OUTPUT_SMALL
            {0.55, 0.6, 0.8, 0.85}, //OUTPUT_MID
            {0.8, 0.85, 1, 2} //OUTPUT_HIGH
    };



    float noRuleReturnValue = 1;

    const Rule *rules[output_term_count];
    float blockCenter[output_term_count] = {0};

public:
    FuzzyDerivative()
    {

        // Кол-во точек численного интегрирования
        int IntCount = 100;
        for(int i=0; i < output_term_count; i++)
        {

            float a = output[i][2] - output[i][1];
            float b = output[i][3] - output[i][0];
            float c = output[i][1] - output[i][0];

            blockCenter[i] = (2 * a * c  +  a * 2  +  c * b  +  a * b  +  b * b) / 3 * (a + b);
        }




        /*
         * RULE_OUTPUT_LOW
         *
         **/
        static condition conditions_low[] = {{ERROR_LOW, OMEGA_VHIGH}, {ERROR_LOW, OMEGA_VERY_LOW}, {ERROR_VHIGH, OMEGA_VERY_LOW}, {ERROR_LOW, OMEGA_MID}, {ERROR_LOW, OMEGA_LOW}, {ERROR_HIGH, OMEGA_LOW}, {ERROR_VHIGH, OMEGA_LOW}, {ERROR_VHIGH, OMEGA_VHIGH}};
        static const Rule rule_output_low = {.conditionCount = sizeof(conditions_low) / sizeof(condition), .conditions = conditions_low};

        /*
         * RULE_OUTPUT_SMALL
         *
         **/
        static condition conditions_small[] = {{ERROR_HIGH, OMEGA_MID}, {ERROR_VHIGH, OMEGA_MID}};
        static const Rule rule_output_small = {.conditionCount = sizeof(conditions_small) / sizeof(condition), .conditions = conditions_small};

        /*
         * RULE_OUTPUT_MID
         *
         **/
        static condition conditions_mid[] = {{ERROR_MID, OMEGA_LOW}, {ERROR_MID, OMEGA_MID}, {ERROR_LOW, OMEGA_HIGH}, {ERROR_MID, OMEGA_HIGH}, {ERROR_HIGH, OMEGA_HIGH}, {ERROR_VHIGH, OMEGA_HIGH}};
        static const Rule rule_output_mid = {.conditionCount = sizeof(conditions_mid) / sizeof(condition), .conditions = conditions_mid};

        /*
         * RULE_OUTPUT_HIGH
         *
         **/
        static condition conditions_high[] = {{ERROR_MID, OMEGA_VHIGH}, {ERROR_HIGH, OMEGA_VHIGH}};
        static const Rule rule_output_high = {.conditionCount = sizeof(conditions_high) / sizeof(condition), .conditions = conditions_high};


        rules[OUTPUT_LOW] = &rule_output_low;
        rules[OUTPUT_SMALL] = &rule_output_small;
        rules[OUTPUT_MID] = &rule_output_mid;
        rules[OUTPUT_HIGH] = &rule_output_high;
    }



    float TrapezoidalMF(float x, float a, float b, float c, float d);

    float execute(float error, float omega);
};

extern FuzzyDerivative fuzzyDerivative;


#endif //__FuzzyDerivative_H_
