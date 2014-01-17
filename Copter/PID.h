//
// Created by Alabay on 21.12.13.
//


#ifndef __PID_H_
#define __PID_H_


class PID {
public:
    PID(float *inputValue, float *outputValue, float *targetValue, float kp, float ki, float kd)
    {
        input = inputValue;
        output = outputValue;
        target = targetValue;

        _imax = 100;

        _integrator = 0;
        _last_derivative = NAN;
        _last_square = 0;

        setCoefficients(kp, ki, kd);
        setLimits(-100, 100);

        p = 0;
        i = 0;
        d = 0;
        lastMillis = 0;

        der_ch = 1;
    }

    float get_p(float error);
    float get_d(float input, float dt);
    float get_i(float error, float dt);
    float get_integrator(){
        return _integrator;
    };

    float p,i,d,s;

    void Compute(int id, int gyro);
    void setLimits(float min, float max);


private:
    float _kp;
    float _ki;
    float _kd;

    unsigned long lastMillis;

    float *input;
    float *output;
    float *target;

    float limitMin;
    float limitMax;

    float _integrator;
    int _imax;
    float _last_derivative;
    float _last_square;
    float _last_input;
    static float const _filter;

    float der_ch;

    float lastDimult = 1;

    void setCoefficients(float Kp, float Ki, float Kd);

    void reset_I();

    float get_s(float error);
};


#endif //__PID_H_
