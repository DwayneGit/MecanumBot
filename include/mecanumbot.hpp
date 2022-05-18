#ifndef MECANUMBOT_HPP
#define MECANUMBOT_HPP

#include <stdio.h>
#include "bimotor.hpp"

#define MECANUMBOT_DIRECTION_FORWARD 0
#define MECANUMBOT_DIRECTION_BACKWARD 1
#define MECANUMBOT_DIRECTION_RIGHT 2
#define MECANUMBOT_DIRECTION_LEFT 3
#define MECANUMBOT_DIRECTION_FORWARD_RIGHT 4
#define MECANUMBOT_DIRECTION_FORWARD_LEFT 5
#define MECANUMBOT_DIRECTION_BACKWARD_RIGHT 6 
#define MECANUMBOT_DIRECTION_BACKWARD_LEFT 7
#define MECANUMBOT_DIRECTION_CLOCKWISE 8
#define MECANUMBOT_DIRECTION_COUNTER_CLOCKWISE 9

class Mecanumbot
{
public:
    uint direction;
    bool on;
    BiMotor* motor1;
    BiMotor* motor2;
    BiMotor* motor3;
    BiMotor* motor4;
    Mecanumbot(BiMotor* m1, BiMotor* m2, BiMotor* m3, BiMotor* m4);
    ~Mecanumbot();
    void set_on();
    void set_off();
    void set_direction(int m1_speed, int m2_speed, int m3_speed, int m4_speed, uint direction);
};

#endif