#include "Arduino.h"
#include "PID.h"


const float  PID::_filter = 25.0f;


float PID::get_p(float error)
{
    return error * _kp;
}

float PID::get_i(float error, float dt)
{
    if ( ( dt != 0 ) && ( _ki != 0 ) ) {

        _integrator += (error * _ki) * dt;

        if (_integrator < -_imax)
        {
            _integrator = -_imax;
        }
        else if (_integrator > _imax)
        {
            _integrator = _imax;
        }


        return _integrator;
    }
    return 0;
}



float PID::get_d(float input, float dt)
{
    if ( ( dt > 0 ) && ( _kd != 0 ) ) {

        float derivative;

        if (isnan(_last_derivative))
        {
            derivative = 0;
            _last_derivative = 0;
        }
        else
        {
            derivative = (input - _last_input) / dt;
        }

        float RC = 1 / (2 * (float)PI * _filter);
        derivative = _last_derivative + ( ( dt / (RC + dt ) ) * (derivative - _last_derivative));

        _last_derivative = derivative;
        _last_input = input;

        return _kd * derivative;
    }

    return 0;
}

float PID::get_s(float error)
{
    float p = get_p(error);
    /**
     * Сглаживает D-term
     */
    float sign = p > 0 ? -1 : 1;
    p = abs(p);
    if(p != 0)
    {
        float s = 0.2 * _last_square + 0.8 * (float)constrain(sign * sqrt(p) * 10 * _kd, -16, 16);
        _last_square = s;
        return s;
    }

    return 0;
}

void PID::setCoefficients(float Kp, float Ki, float Kd)
{
    _kp = Kp;
    _ki = Ki;
    _kd = Kd;
}

void PID::setLimits(float min, float max)
{
    if(min >= max) return;
    limitMin = min;
    limitMax = max;


    //*output = constrain(*output, limitMin, limitMax);
}

void PID::Compute()
{


    unsigned long now = millis();
    unsigned long dt = (now - lastMillis);
    float delta_time;

    if (lastMillis == 0 || dt > 1000) {
        dt = 0;
        reset_I();
    }
    lastMillis = now;

    delta_time = dt / 1000.0f;

    float error = *target - *input;


    p = get_p(error);
    i = get_i(error, delta_time);
    d = get_d(error, delta_time);
    s = get_s(error);

    *output = p + i + d + s;

    *output = constrain(*output, limitMin, limitMax);
}


void PID::reset_I()
{
    _integrator = 0;
    _last_derivative = NAN;
}
