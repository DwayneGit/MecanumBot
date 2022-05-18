#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d)
{
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
    if (divider16 / 16 == 0)
        divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / f - 1;
    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);
    return wrap;
}

uint32_t pwm_get_wrap(uint slice_num)
{
    valid_params_if(PWM, slice_num >= 0 && slice_num < NUM_PWM_SLICES);
    return pwm_hw->slice[slice_num].top;
}

void pwm_set_duty(uint slice_num, uint chan, int d)
{
    pwm_set_chan_level(slice_num, chan, pwm_get_wrap(slice_num) * d / 100);
}

typedef struct
{
    uint gpioForward;
    uint gpioBackward;
    uint gpioEnabled;
    uint slice;
    uint Fchan;
    uint ENchan;
    bool forward;
    uint speed;
    uint freq;
    uint resolution;
    bool on;
} BiMotor;

void BiMotorInit(BiMotor *m, uint gpioEnabled, uint gpioForward, uint gpioBackward, uint freq)
{
    gpio_set_function(gpioEnabled, GPIO_FUNC_PWM);
    m->gpioEnabled = gpioEnabled;
    m->slice = pwm_gpio_to_slice_num(gpioEnabled);
    m->ENchan = pwm_gpio_to_channel(gpioEnabled);

    gpio_init(gpioForward);
    gpio_set_dir(gpioForward, GPIO_OUT);
    m->gpioForward = gpioForward;

    gpio_init(gpioBackward);
    gpio_set_dir(gpioBackward, GPIO_OUT);
    m->gpioBackward = gpioBackward;

    m->freq = freq;
    m->speed = 0;
    m->forward = true;
    m->resolution = pwm_set_freq_duty(m->slice, m->ENchan, m->freq, 0);
    pwm_set_duty(m->slice, m->ENchan, 0);
    m->on = false;
}

void BiMotorspeed(BiMotor *m, int s, bool forward)
{
    if (forward)
    {
        gpio_put(m->gpioBackward, 0);
        gpio_put(m->gpioForward, 1);
        m->forward = true;
    }
    else
    {
        gpio_put(m->gpioForward, 0);
        gpio_put(m->gpioBackward, 1);
        m->forward = false;
    }
    pwm_set_duty(m->slice, m->ENchan, s);
}

void BiMotorOn(BiMotor *m)
{
    pwm_set_enabled(m->slice, true);
    m->on = true;
}

void BiMotorOff(BiMotor *m)
{
    pwm_set_enabled(m->slice, false);
    m->on = false;
}

int main()
{
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    BiMotor mot1;
    BiMotor mot2;
    BiMotor mot3;
    BiMotor mot4;
    BiMotorInit(&mot1, 20, 4, 5, 2000);
    BiMotorInit(&mot2, 21, 6, 7, 2000);
    BiMotorInit(&mot3, 18, 8, 9, 2000);
    BiMotorInit(&mot4, 19, 10, 11, 2000);

    BiMotorOn(&mot1);
    BiMotorOn(&mot2);
    BiMotorOn(&mot3);
    BiMotorOn(&mot4);
    while (true)
    {
        BiMotorspeed(&mot1, 95, true);
        BiMotorspeed(&mot2, 95, true);
        BiMotorspeed(&mot3, 95, true);
        BiMotorspeed(&mot4, 95, true);
        gpio_put(25, 1);
        sleep_ms(2000);
        BiMotorspeed(&mot1, 95, false);
        BiMotorspeed(&mot2, 95, false);
        BiMotorspeed(&mot3, 95, false);
        BiMotorspeed(&mot4, 95, false);
        gpio_put(25, 0);
        sleep_ms(2000);
    }

    return 0;
}
