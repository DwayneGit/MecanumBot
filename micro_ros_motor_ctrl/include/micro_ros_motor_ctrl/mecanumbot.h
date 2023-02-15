#ifndef MECANUMBOT_HPP
#define MECANUMBOT_HPP

#include <stdio.h>
#include "bimotor.h"

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

#define MAX_VEL 100.0
#define MIN_VEL 45.0

static int chars_rxed = 0;

struct Mecanumbot
{
    uint direction;
    bool on;
    struct BiMotor* motor1;
    struct BiMotor* motor2;
    struct BiMotor* motor3;
    struct BiMotor* motor4;
};

void set_on(struct Mecanumbot *this);
void set_off(struct Mecanumbot *this);
void set_direction(struct Mecanumbot *this, double linear_x, double angular_z);

void mecanumbot_new(
    struct Mecanumbot ** _mbot,
    struct BiMotor *m1, 
    struct BiMotor *m2, 
    struct BiMotor *m3, 
    struct BiMotor *m4
);

#endif