//
// Created by Alabay on 26.12.13.
//

#include "FuzzyDerivative.h"


#define DOT_A 0
#define DOT_B 1
#define DOT_C 2
#define DOT_D 3

float FuzzyDerivative::TrapezoidalMF(float x, float a, float b, float c, float d)
{
    if(x < a || x > d) return 0;

    if(x == b && a == b) return 1;
    if(x == d && c == d) return 1;

    if(x >= b && x <= c) return 1;

    if(x < b) return (x - a) / (b - a);

    if(x > c) return (d - x) / (d - c);
}

float FuzzyDerivative::execute(float error, float omega)
{
    /**
     * Расчет функции инстинности для ошибки рассогласования
     */
    float thruth_error[input_error_term_count] = {0};

    for(int i=0; i < input_error_term_count; i++)
    {
        thruth_error[i] = TrapezoidalMF(error, input_error[i][DOT_A], input_error[i][DOT_B], input_error[i][DOT_C], input_error[i][DOT_D]);
    }


    /**
    * TODO: копипаста
     * Расчет функции инстинности для угловой скорости
     */
    float thruth_omega[input_omega_term_count] = {0};

    for(int i=0; i < input_omega_term_count; i++)
    {
        thruth_omega[i] = TrapezoidalMF(omega, input_omega[i][DOT_A], input_omega[i][DOT_B], input_omega[i][DOT_C], input_omega[i][DOT_D]);
    }


    /**
     * Расчет функции принадлежности
     */

    float allWeight = 0;
    float Accumulation[output_term_count] = {0};

    float SumRI = 0;

    for (int i=0; i < output_term_count; i++)
    {
        const Rule *rule = rules[i];


        for(int j=0; j < rule->conditionCount; j++)
        {
            /**
             * Выбираем минимальное, т.к. логическое И
             *
             * MAGIC NUMBERS:
             *  0 - error
             *  1 - omega
             *  (В порядке добавления в массив rules)
             */

            int thruth_e_index = rule->conditions[j].set[0];
            int thruth_o_index = rule->conditions[j].set[1];
            float val = (float)fmin(thruth_error[thruth_e_index], thruth_omega[thruth_o_index]);


            if(val != 0 && val > Accumulation[i])
            {
                Accumulation[i] = val;
            }
        }

        /**
         * Получаем координаты начала блока + центр тяжести
         */

        SumRI += (output[i][0] + blockCenter[i]) * Accumulation[i];
        //Вычисление центра масс трапецевидного блока функции вывода
        allWeight += Accumulation[i];
    }


    if(allWeight == 0)
    {

        return noRuleReturnValue;
    }

    return SumRI / allWeight;
}

FuzzyDerivative fuzzyDerivative;