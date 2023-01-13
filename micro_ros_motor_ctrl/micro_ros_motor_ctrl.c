#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"

#include "include/mecanumbot.h"
#include "include/pico_uart_transports.h"

struct Mecanumbot *mecanumbot;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  printf("a: %d, l: %d\n", msg->angular, msg->linear);
}

void encoder_pass_irq_handler(uint gpio, uint32_t events){
    // std_msgs__msg__Int32 msg;

    printf("this is a meme\n");
    // // Set message value
    // msg.data = 0;
    // mecanumbot->get_encoder_data(mecanumbot, gpio);
    // rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
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

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);
    
    geometry_msgs__msg__Twist twist_msg;

    allocator = rcl_get_default_allocator();
    
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motor_controller", "", &support);

    struct BiMotor * mot1 = bimotor_new(1, BRIDGE1_ENB, BRIDGE1_IN3, BRIDGE1_IN4, BRIDGE1_ENCA, 2000);
    struct BiMotor * mot2 = bimotor_new(2, BRIDGE1_ENA, BRIDGE1_IN1, BRIDGE1_IN2, BRIDGE1_ENCB, 2000);
    struct BiMotor * mot3 = bimotor_new(3, BRIDGE2_ENA, BRIDGE2_IN1, BRIDGE2_IN2, BRIDGE2_ENCA, 2000);
    struct BiMotor * mot4 = bimotor_new(4, BRIDGE2_ENB, BRIDGE2_IN3, BRIDGE2_IN4, BRIDGE2_ENCB, 2000);
    mecanumbot = mecanumbot_new(mot1, mot2, mot3, mot4);

    gpio_set_irq_enabled_with_callback(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);
    gpio_set_irq_enabled_with_callback(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);
    gpio_set_irq_enabled_with_callback(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);
    gpio_set_irq_enabled_with_callback(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true, &encoder_pass_irq_handler);

    if (rc != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return -1;
    }

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "mecanumbot/wheel_rot"
    );

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "mecanumbot/cmd_vel"
    );

	rclc_executor_init(
        &executor,
        &support.context, 
        1, 
        &allocator
    );

    rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &twist_msg,
        &subscription_callback, 
        ON_NEW_DATA
    );

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // msg.linear = [0,0,0];
    // msg.angular = [0,0,0];
    while (true){
        rclc_executor_spin(&executor);
    }

	// rcl_subscription_fini(&subscriber, &node);
    // rcl_publisher_fini(&publisher, &node);
	// rcl_node_fini(&node);

    return 0;
}
