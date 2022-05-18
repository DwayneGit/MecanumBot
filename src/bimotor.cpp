#include "../include/bimotor.hpp"
#include "../include/pwm.hpp"

BiMotor::BiMotor(uint gpioEn, uint gpioFor, uint gpioBack, uint frequency)
{
    gpio_set_function(gpioEn, GPIO_FUNC_PWM);
    gpioEnabled = gpioEn;
    slice = pwm_gpio_to_slice_num(gpioEn);
    ENchan = pwm_gpio_to_channel(gpioEn);

    gpio_init(gpioFor);
    gpio_set_dir(gpioFor, GPIO_OUT);
    gpioForward = gpioFor;

    gpio_init(gpioBack);
    gpio_set_dir(gpioBack, GPIO_OUT);
    gpioBackward = gpioBack;

    freq = frequency;
    speed = 0;
    direction = BIMOTOR_FORWARD;
    resolution = pwm_set_freq_duty(slice, ENchan, freq, 0);
    pwm_set_duty(slice, ENchan, 0);
    on = false;
}

void BiMotor::set_motor_speed(int s, bool direction)
{
    this->set_motor_direction(direction);
    pwm_set_duty(this->slice, this->ENchan, s);
    this->speed = s;
}

void BiMotor::set_motor_on()
{
    this->on = true;
    pwm_set_enabled(this->slice, this->on);
}

void BiMotor::set_motor_off()
{
    this->on = false;
    pwm_set_enabled(this->slice, this->on);
}

void BiMotor::set_motor_direction(bool direction)
{
    if (direction == BIMOTOR_FORWARD)
    {
        gpio_put(this->gpioBackward, 0);
        gpio_put(this->gpioForward, 1);
    }
    else
    {
        gpio_put(this->gpioBackward, 1);
        gpio_put(this->gpioForward, 0);
    }
    this->direction = direction;
}
