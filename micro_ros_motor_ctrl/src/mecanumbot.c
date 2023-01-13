#include "../include/mecanumbot.h"

struct Mecanumbot * mecanumbot_new(struct BiMotor *m1, struct BiMotor *m2, struct BiMotor *m3, struct BiMotor *m4)
{
    struct Mecanumbot * mbot;
    mbot->motor1 = m1;
    mbot->motor2 = m2;
    mbot->motor3 = m3;
    mbot->motor4 = m4;
}

void set_off(struct Mecanumbot *this)
{
    this->motor1->set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
    this->motor2->set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
    this->motor3->set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
    this->motor4->set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
    // this->motor1->set_motor_off();
    // this->motor2->set_motor_off();
    // this->motor3->set_motor_off();
    // this->motor4->set_motor_off();
}

void set_on(struct Mecanumbot *this)
{
    this->motor1->set_motor_on(this->motor1);
    this->motor2->set_motor_on(this->motor2);
    this->motor3->set_motor_on(this->motor3);
    this->motor4->set_motor_on(this->motor4);
}

void get_encoder_data(struct Mecanumbot *this, uint gpio){
    if( gpio == this->motor1->gpioEncOut)
        this->motor1->get_encoder_data(this->motor1);
    else if (gpio ==  this->motor2->gpioEncOut)
        this->motor2->get_encoder_data(this->motor2);
    else if (gpio ==  this->motor3->gpioEncOut)
        this->motor3->get_encoder_data(this->motor3);
    else if (gpio ==  this->motor4->gpioEncOut)
        this->motor4->get_encoder_data(this->motor4);
    // this->motor1->get_encoder_data();
    // this->motor2->get_encoder_data();
    // this->motor3->get_encoder_data();
    // this->motor4->get_encoder_data();
}

void set_direction(struct Mecanumbot *this, int m1_speed, int m2_speed, int m3_speed, int m4_speed, uint direction)
{
    switch (direction)
    {
    case MECANUMBOT_DIRECTION_FORWARD:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_RIGHT:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_LEFT:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_FORWARD_RIGHT:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_FORWARD_LEFT:
        this->motor1->set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD_RIGHT:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD_LEFT:
        this->motor1->set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_CLOCKWISE:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_COUNTER_CLOCKWISE:
        this->motor1->set_motor_speed(this->motor1, m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(this->motor2, m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(this->motor3, m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(this->motor4, m4_speed, BIMOTOR_BACKWARD);
        break;
    default:
        break;
    }
}