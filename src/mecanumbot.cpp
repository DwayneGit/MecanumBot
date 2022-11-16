#include "../include/mecanumbot.hpp"

Mecanumbot::Mecanumbot(BiMotor *m1, BiMotor *m2, BiMotor *m3, BiMotor *m4)
{
    motor1 = m1;
    motor2 = m2;
    motor3 = m3;
    motor4 = m4;
}

void Mecanumbot::set_off()
{
    this->motor1->set_motor_speed(0, BIMOTOR_FORWARD);
    this->motor2->set_motor_speed(0, BIMOTOR_FORWARD);
    this->motor3->set_motor_speed(0, BIMOTOR_FORWARD);
    this->motor4->set_motor_speed(0, BIMOTOR_FORWARD);
    // this->motor1->set_motor_off();
    // this->motor2->set_motor_off();
    // this->motor3->set_motor_off();
    // this->motor4->set_motor_off();
}

void Mecanumbot::set_on()
{
    this->motor1->set_motor_on();
    this->motor2->set_motor_on();
    this->motor3->set_motor_on();
    this->motor4->set_motor_on();
}

void Mecanumbot::get_encoder_data(uint gpio){
    if( gpio == this->motor1->gpioEncOut)
        this->motor1->get_encoder_data();
    else if (gpio ==  this->motor2->gpioEncOut)
        this->motor2->get_encoder_data();
    else if (gpio ==  this->motor3->gpioEncOut)
        this->motor3->get_encoder_data();
    else if (gpio ==  this->motor4->gpioEncOut)
        this->motor4->get_encoder_data();
    // this->motor1->get_encoder_data();
    // this->motor2->get_encoder_data();
    // this->motor3->get_encoder_data();
    // this->motor4->get_encoder_data();
}

void Mecanumbot::set_direction(int m1_speed, int m2_speed, int m3_speed, int m4_speed, uint direction)
{
    switch (direction)
    {
    case MECANUMBOT_DIRECTION_FORWARD:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_RIGHT:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_LEFT:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_FORWARD_RIGHT:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(0, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(0, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_FORWARD_LEFT:
        this->motor1->set_motor_speed(0, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(0, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD_RIGHT:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(0, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(0, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD_LEFT:
        this->motor1->set_motor_speed(0, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(0, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_CLOCKWISE:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_FORWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_BACKWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_BACKWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_COUNTER_CLOCKWISE:
        this->motor1->set_motor_speed(m1_speed, BIMOTOR_BACKWARD);
        this->motor2->set_motor_speed(m2_speed, BIMOTOR_FORWARD);
        this->motor3->set_motor_speed(m3_speed, BIMOTOR_FORWARD);
        this->motor4->set_motor_speed(m4_speed, BIMOTOR_BACKWARD);
        break;
    default:
        break;
    }
}