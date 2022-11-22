#ifndef BIMOTOR_HPP
#define BIMOTOR_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define BIMOTOR_FORWARD false
#define BIMOTOR_BACKWARD true

#define ENCODER_MIN -32768
#define ENCODER_MAX 32768

class BiMotor
{
private:
    void set_motor_direction(bool direction);
public:
    uint gpioForward;
    uint gpioBackward;
    uint gpioEnabled;
    uint gpioEncOut;
    uint slice;
    uint ENchan;
    uint direction;
    uint speed;
    uint freq;
    uint resolution;
    uint encoderTickCount;
    int motorNum;
    bool on;
    BiMotor();
    BiMotor(int motNum, uint gpioEn, uint gpioFor, uint gpioBack, uint gpioEncoderOut, uint frequency);
    ~BiMotor();
    void set_motor_speed(int s, bool direction);
    void set_motor_on();
    void set_motor_off();
    void get_encoder_data();
};

#endif