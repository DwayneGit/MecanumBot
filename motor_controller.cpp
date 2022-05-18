#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "include/mecanumbot.hpp"

using namespace std;

int main()
{
    stdio_init_all();
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    BiMotor mot1 = BiMotor(20, 4, 5, 2000);
    BiMotor mot2 = BiMotor(18, 2, 3, 2000);
    BiMotor mot3 = BiMotor(28, 12, 13, 2000);
    BiMotor mot4 = BiMotor(22, 6, 7, 2000);
    Mecanumbot mecanumbot = Mecanumbot(&mot1, &mot2, &mot3, &mot4);
    // mecanumbot.set_on();
    while (true)
    {
        mecanumbot.set_on();
        mecanumbot.set_direction(95, 95, 95, 95, MECANUMBOT_DIRECTION_FORWARD);
        cout << "Moving Forward..." << endl;
        gpio_put(25, 1);
        sleep_ms(2000);
        mecanumbot.set_off();
        gpio_put(25, 0);
        sleep_ms(2000);
        mecanumbot.set_on();
        mecanumbot.set_direction(95, 95, 95, 95, MECANUMBOT_DIRECTION_RIGHT);
        cout << "Moving Right..." << endl;
        gpio_put(25, 1);
        sleep_ms(2000);
        mecanumbot.set_off();
        gpio_put(25, 0);
        sleep_ms(2000);
        mecanumbot.set_on();
        mecanumbot.set_direction(95, 95, 95, 95, MECANUMBOT_DIRECTION_BACKWARD);
        cout << "Moving Backward..." << endl;
        gpio_put(25, 1);
        sleep_ms(2000);
        mecanumbot.set_off();
        gpio_put(25, 0);
        sleep_ms(2000);
        mecanumbot.set_on();
        mecanumbot.set_direction(95, 95, 95, 95, MECANUMBOT_DIRECTION_LEFT);
        cout << "Moving Left..." << endl;
        gpio_put(25, 1);
        sleep_ms(2000);
        mecanumbot.set_off();
        gpio_put(25, 0);
        sleep_ms(2000);
    }

    return 0;
}
