#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include <mecanumbot_msgs/msg/wheel_ticks_message.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "include/micro_ros_motor_ctrl/mecanumbot.h"
#include "include/micro_ros_motor_ctrl/pico_uart_transports.h"

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

#define PUBLISHER_NUMBER 4

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

struct Mecanumbot *mecanumbot;

rcl_subscription_t subscriber;

rcl_publisher_t wheel_ticks_publisher;

void subscription_callback(const void * msgin)
{
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
    set_on(mecanumbot);
    set_direction(mecanumbot, msg->linear.x, msg->angular.z);
}

void wheel1_enc_tick(uint gpio, uint32_t events){
    get_encoder_data(mecanumbot->motor1, gpio);
    get_encoder_data(mecanumbot->motor2, gpio);
    get_encoder_data(mecanumbot->motor3, gpio);
    get_encoder_data(mecanumbot->motor4, gpio);
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    rcl_node_t node;
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_allocator_t allocator;

    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // // Turn off FIFO's - we want to do this character by character
    // uart_set_fifo_enabled(UART_ID, false);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    gpio_set_irq_enabled(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true);

    gpio_set_irq_callback(&wheel1_enc_tick);
    
    irq_set_enabled(IO_IRQ_BANK0, true);

    geometry_msgs__msg__Twist twist_msg;

    allocator = rcl_get_default_allocator();
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "motor_controller", "", &support));

    struct BiMotor mots[4] = {
        bimotor_new(1, BRIDGE1_ENB, BRIDGE1_IN3, BRIDGE1_IN4, BRIDGE1_ENCA, 2000),
        bimotor_new(2, BRIDGE1_ENA, BRIDGE1_IN1, BRIDGE1_IN2, BRIDGE1_ENCB, 2000),
        bimotor_new(3, BRIDGE2_ENA, BRIDGE2_IN1, BRIDGE2_IN2, BRIDGE2_ENCA, 2000),
        bimotor_new(4, BRIDGE2_ENB, BRIDGE2_IN3, BRIDGE2_IN4, BRIDGE2_ENCB, 2000),
    };

    mecanumbot_new(&mecanumbot, &mots[0], &mots[1], &mots[2], &mots[3]);

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    RCCHECK(rclc_publisher_init_default(
        &wheel_ticks_publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(mecanumbot_msgs, msg, WheelTicksMessage), 
        "wheel_ticks"
    ));

	RCCHECK(rclc_executor_init(
        &executor,
        &support.context, 
        1, 
        &allocator
    ));

    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &twist_msg,
        &subscription_callback, 
        ON_NEW_DATA
    ));
    // printf("%i %i\n\r", ENCODER_MIN, ENCODER_MAX);
    // rclc_executor_spin(&executor)
    mecanumbot_msgs__msg__WheelTicksMessage msg;
    while (true){
        msg.wheel1_tick = mecanumbot->motor1->encoderTickCount;
        msg.wheel2_tick = mecanumbot->motor2->encoderTickCount;
        msg.wheel3_tick = mecanumbot->motor3->encoderTickCount;
        msg.wheel4_tick = mecanumbot->motor4->encoderTickCount;
        RCSOFTCHECK(rcl_publish(&wheel_ticks_publisher, &msg, NULL));
        rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
    }

	RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&wheel_ticks_publisher, &node));    
	RCCHECK(rcl_node_fini(&node));

    return 0;
}
