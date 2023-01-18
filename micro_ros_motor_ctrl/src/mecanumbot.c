#include "../include/mecanumbot.h"
#include <stdlib.h>
#include <math.h>

void mecanumbot_new(struct Mecanumbot ** _mbot, struct BiMotor *m1, struct BiMotor *m2, struct BiMotor *m3, struct BiMotor *m4)
{
    struct Mecanumbot * mbot = malloc(sizeof(*_mbot));
    if (mbot) {
        mbot->direction = MECANUMBOT_DIRECTION_FORWARD;
        mbot->on = false;
        mbot->motor1 = m1;
        mbot->motor2 = m2;
        mbot->motor3 = m3;
        mbot->motor4 = m4;
    }
    *_mbot = mbot;
}

void set_off(struct Mecanumbot *this)
{
    set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
    set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
    set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
    set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
    // set_motor_off();
    // set_motor_off();
    // set_motor_off();
    // set_motor_off();
}

void set_on(struct Mecanumbot *this)
{
    set_motor_on(this->motor1);
    set_motor_on(this->motor2);
    set_motor_on(this->motor3);
    set_motor_on(this->motor4);
}

void get_encoder_robot_data(struct Mecanumbot *this, uint gpio){
    if(gpio == this->motor1->gpioEncOut)
        get_encoder_data(this->motor1);
    else if (gpio == this->motor2->gpioEncOut)
        get_encoder_data(this->motor2);
    else if (gpio == this->motor3->gpioEncOut)
        get_encoder_data(this->motor3);
    else if (gpio == this->motor4->gpioEncOut)
        get_encoder_data(this->motor4);
    // get_encoder_data();
    // get_encoder_data();
    // get_encoder_data();
    // get_encoder_data();
}

void set_direction(struct Mecanumbot *this, double linear_x, double linear_y, double angular_z)
{
    int direction = MECANUMBOT_DIRECTION_FORWARD;

    if(linear_x > 0 && linear_y == 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_FORWARD;
    else if(linear_x < 0 && linear_y == 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_BACKWARD;
    else if(linear_x == 0 && linear_y > 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_RIGHT;
    else if(linear_x == 0 && linear_y < 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_LEFT;
    else if(linear_x > 0 && linear_y > 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_FORWARD_RIGHT;
    else if(linear_x > 0 && linear_y < 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_FORWARD_LEFT;
    else if(linear_x < 0 && linear_y > 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_BACKWARD_RIGHT;
    else if(linear_x < 0 && linear_y < 0 && angular_z == 0)
        direction = MECANUMBOT_DIRECTION_BACKWARD_LEFT;
        
    double x_vel = fabs(linear_x);
    double y_vel = fabs(linear_y);

    printf("x: %.2f, y: %.2f, a: %.2f\n", x_vel, y_vel, angular_z);

    switch (direction)
    {
    case MECANUMBOT_DIRECTION_FORWARD:
        set_motor_speed(this->motor1, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor2, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor3, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor4, x_vel, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD:
        set_motor_speed(this->motor1, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor2, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor3, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor4, x_vel, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_RIGHT:
        set_motor_speed(this->motor1, y_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor2, y_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor3, y_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor4, y_vel, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_LEFT:
        set_motor_speed(this->motor1, y_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor2, y_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor3, y_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor4, y_vel, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_FORWARD_RIGHT:
        set_motor_speed(this->motor1, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
        set_motor_speed(this->motor3, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_FORWARD_LEFT:
        set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
        set_motor_speed(this->motor2, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
        set_motor_speed(this->motor4, x_vel, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD_RIGHT:
        set_motor_speed(this->motor1, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
        set_motor_speed(this->motor3, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_BACKWARD_LEFT:
        set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
        set_motor_speed(this->motor2, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
        set_motor_speed(this->motor4, x_vel, BIMOTOR_BACKWARD);
        break;
    case MECANUMBOT_DIRECTION_CLOCKWISE:
        set_motor_speed(this->motor1, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor2, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor3, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor4, x_vel, BIMOTOR_FORWARD);
        break;
    case MECANUMBOT_DIRECTION_COUNTER_CLOCKWISE:
        set_motor_speed(this->motor1, x_vel, BIMOTOR_BACKWARD);
        set_motor_speed(this->motor2, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor3, x_vel, BIMOTOR_FORWARD);
        set_motor_speed(this->motor4, x_vel, BIMOTOR_BACKWARD);
        break;
    default:
        break;
    }
}