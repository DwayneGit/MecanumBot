#ifndef BIMOTOR_HPP
#define BIMOTOR_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define BIMOTOR_FORWARD false
#define BIMOTOR_BACKWARD true

class BiMotor
{
private:
    void set_motor_direction(bool direction);
public:
    uint gpioForward;
    uint gpioBackward;
    uint gpioEnabled;
    uint slice;
    uint ENchan;
    uint direction;
    uint speed;
    uint freq;
    uint resolution;
    bool on;
    BiMotor(uint gpioEn, uint gpioFor, uint gpioBack, uint frequency);
    ~BiMotor();
    void set_motor_speed(int s, bool direction);
    void set_motor_on();
    void set_motor_off();
};

#endif