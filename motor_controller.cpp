#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "include/mecanumbot.hpp"

#define MOTOR_SPEED 45

using namespace std;

Mecanumbot* mecanumbot;

void encoder_pass_irq_handler(uint gpio, uint32_t events){
    mecanumbot->get_encoder_data(gpio);
}

int main()
{
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    // irq_set_enabled(UART0_IRQ, true);
    // uart_set_irq_enables(UART_ID, true, false);

    BiMotor mot1 = BiMotor(1, BRIDGE1_ENB, BRIDGE1_IN3, BRIDGE1_IN4, BRIDGE1_ENCA, 2000);
    BiMotor mot2 = BiMotor(2, BRIDGE1_ENA, BRIDGE1_IN1, BRIDGE1_IN2, BRIDGE1_ENCB, 2000);
    BiMotor mot3 = BiMotor(3, BRIDGE2_ENA, BRIDGE2_IN1, BRIDGE2_IN2, BRIDGE2_ENCA, 2000);
    BiMotor mot4 = BiMotor(4, BRIDGE2_ENB, BRIDGE2_IN3, BRIDGE2_IN4, BRIDGE2_ENCB, 2000);
    mecanumbot = new Mecanumbot(&mot1, &mot2, &mot3, &mot4);

    gpio_set_irq_enabled_with_callback(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);
    gpio_set_irq_enabled_with_callback(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);
    gpio_set_irq_enabled_with_callback(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);
    gpio_set_irq_enabled_with_callback(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);

    // mecanumbot->set_on();
    uint8_t buffer[1];
    printf("Motor Controller active \n");
    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        if(uart_is_readable(UART_ID)){
            buffer[0] = uart_getc(UART_ID);
            switch(buffer[0]){
                case 'w':
                    mecanumbot->set_on();
                    mecanumbot->set_direction(MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MECANUMBOT_DIRECTION_FORWARD);
                    printf("Moving Forward ... \n");
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                break;
                case 'a':
                    mecanumbot->set_on();
                    mecanumbot->set_direction(MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MECANUMBOT_DIRECTION_LEFT);
                    printf("Moving Left ... \n");
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                break;
                case 's':
                    mecanumbot->set_on();
                    mecanumbot->set_direction(MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MECANUMBOT_DIRECTION_BACKWARD);
                    printf("Moving Backward ... \n");
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                break;
                case 'd':
                    mecanumbot->set_on();
                    mecanumbot->set_direction(MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MOTOR_SPEED, MECANUMBOT_DIRECTION_RIGHT);
                    printf("Moving Right ... \n");
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                break;
                case '0':
                    mecanumbot->set_off();
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                break;
                default:
                    printf("%c", buffer[0]);
                break;
            }
        }
        // mecanumbot->get_encoder_data();
    }
    return 0;
}
