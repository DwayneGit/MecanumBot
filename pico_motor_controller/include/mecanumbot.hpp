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

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define BRIDGE1_ENA 8
#define BRIDGE1_IN1 7
#define BRIDGE1_IN2 6
#define BRIDGE1_ENCA 18

#define BRIDGE1_ENB 2
#define BRIDGE1_IN3 4
#define BRIDGE1_IN4 3
#define BRIDGE1_ENCB 19

#define BRIDGE2_ENA 10
#define BRIDGE2_IN1 11
#define BRIDGE2_IN2 12
#define BRIDGE2_ENCA 20

#define BRIDGE2_ENB 15
#define BRIDGE2_IN3 13
#define BRIDGE2_IN4 14
#define BRIDGE2_ENCB 21

static int chars_rxed = 0;

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
    void get_encoder_data(uint gpio);
};

#endif