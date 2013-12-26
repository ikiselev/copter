//
// Created by Alabay on 26.12.13.
//


#ifndef __FuzzyDerivative_H_
#define __FuzzyDerivative_H_
#include "Arduino.h"

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
    int input_error[input_error_term_count][TRAPEZODIAL] = {
        {0, 0, 4, 9}, //ERROR_LOW
        {4, 6, 10, 13}, //ERROR_MID
        {10, 12, 40, 45}, //ERROR_HIGH
        {40, 45, 90, 90}, //ERROR_VHIGH
    };


    /**
     * Угловая скорость
     */
    static const int input_omega_term_count = 4;

    static const uint8_t OMEGA_LOW = 0;
    static const uint8_t OMEGA_MID = 1;
    static const uint8_t OMEGA_HIGH = 2;
    static const uint8_t OMEGA_VHIGH = 3;

    //int input_omega[input_omega_term_count][TRAPEZODIAL];
    int input_omega[input_omega_term_count][TRAPEZODIAL] = {
        {0, 0, 10, 12}, //OMEGA_LOW
        {10, 12, 16, 20}, //OMEGA_MID
        {15, 20, 35, 40}, //OMEGA_HIGH
        {35, 40, 120, 120} //OMEGA_VHIGH
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
            {0.8, 0.85, 1, 1} //OUTPUT_HIGH
    };

    /**
     * TODO: hardcode magic numbers
     * На данном микроконтроллере с 2 кб ОЗУ захардкожено
     * Соглашение: каждое правило состоит из двух значений термов переменных
     * Первый терм: Ошибка рассогласования
     * Второй: угловая скорость
     *
     * В каждом правиле может быть не обязательно 4 элемента, но мы считаем, что это сейчас нормально
     */
    static const int combinationsPerOutTerm = 4;


    uint8_t rules[output_term_count][combinationsPerOutTerm][2] = {
        /*
         * RULE_OUTPUT_LOW
         *
         **/
        {{ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}},

        /*
         * RULE_OUTPUT_SMALL
         *
         **/
        {{ERROR_MID, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}},

        /*
         * RULE_OUTPUT_MID
         *
         **/
        {{ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}},

        /*
         * RULE_OUTPUT_HIGH
         *
         **/
        {{ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}, {ERROR_LOW, OMEGA_LOW}}
    };

    float blockCenter[output_term_count] = {0};

public:
    FuzzyDerivative()
    {

        // Кол-во точек численного интегрирования
        int IntCount = 100;
        for(int i=0; i < output_term_count; i++)
        {
            /*float dx = (output[i][3] - output[i][0]) / IntCount;
            float x = output[i][0];
            float sum_RI = 0;
            float sum_i = 0;
            for(int j = 0; j < IntCount; j++)
            {
                x = x + dx * j;
                float m_i = TrapezoidalMF(x, output[i][0], output[i][1], output[i][2], output[i][3]);
                sum_RI += x * m_i;
                sum_i += m_i;
            }

            blockCenter[i] = sum_RI / sum_i;*/

            float a = output[i][2] - output[i][1];
            float b = output[i][3] - output[i][0];
            float c = output[i][1] - output[i][0];

            blockCenter[i] = (2 * a * c  +  a * 2  +  c * b  +  a * b  +  b * b) / 3 * (a + b);
        }
    }



    float TrapezoidalMF(float x, float a, float b, float c, float d);

    float execute(float error, float omega);
};


#endif //__FuzzyDerivative_H_
