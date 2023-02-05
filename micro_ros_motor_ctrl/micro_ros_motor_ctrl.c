#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"

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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n\r",__LINE__,(int)temp_rc);}}

struct Mecanumbot *mecanumbot;

rcl_subscription_t subscriber;

rcl_publisher_t wheel1_pub;
rcl_publisher_t wheel2_pub;
rcl_publisher_t wheel3_pub;
rcl_publisher_t wheel4_pub;

void subscription_callback(const void * msgin)
{
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
    // if(msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0)
    //     set_off(mecanumbot);
    // else {
        set_on(mecanumbot);
        set_direction(mecanumbot, msg->linear.x, msg->angular.z);
    // }
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

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    gpio_set_irq_enabled(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true);

    gpio_set_irq_callback(&wheel1_enc_tick);
    
    irq_set_enabled(IO_IRQ_BANK0, true);

    geometry_msgs__msg__Twist twist_msg;

    allocator = rcl_get_default_allocator();
    
    rcl_ret_t rc;
    rc = rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motor_controller", "", &support);

    struct BiMotor mot1 = bimotor_new(1, BRIDGE1_ENB, BRIDGE1_IN3, BRIDGE1_IN4, BRIDGE1_ENCA, 2000);
    struct BiMotor mot2 = bimotor_new(2, BRIDGE1_ENA, BRIDGE1_IN1, BRIDGE1_IN2, BRIDGE1_ENCB, 2000);
    struct BiMotor mot3 = bimotor_new(3, BRIDGE2_ENA, BRIDGE2_IN1, BRIDGE2_IN2, BRIDGE2_ENCA, 2000);
    struct BiMotor mot4 = bimotor_new(4, BRIDGE2_ENB, BRIDGE2_IN3, BRIDGE2_IN4, BRIDGE2_ENCB, 2000);

    mecanumbot_new(&mecanumbot, &mot1, &mot2, &mot3, &mot4);

    if (rc != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return -1;
    }

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    const rosidl_message_type_support_t * int16_type_support = 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16);

    rclc_publisher_init_default(&wheel1_pub, &node, int16_type_support, "wheel1_ticks");
    rclc_publisher_init_default(&wheel2_pub, &node, int16_type_support, "wheel2_ticks");
    rclc_publisher_init_default(&wheel3_pub, &node, int16_type_support, "wheel3_ticks");
    rclc_publisher_init_default(&wheel4_pub, &node, int16_type_support, "wheel4_ticks");

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

    // rclc_executor_spin(&executor)
    while (true){
        // rc = rcl_publish(&wheel1_pub, &mecanumbot->motor1->encoderTickCount, NULL);
        // rc = rcl_publish(&wheel2_pub, &mecanumbot->motor2->encoderTickCount, NULL);
        // rc = rcl_publish(&wheel3_pub, &mecanumbot->motor3->encoderTickCount, NULL);
        // rc = rcl_publish(&wheel4_pub, &mecanumbot->motor4->encoderTickCount, NULL);
        rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
    }

	RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&wheel1_pub, &node));
    RCCHECK(rcl_publisher_fini(&wheel2_pub, &node));
    RCCHECK(rcl_publisher_fini(&wheel3_pub, &node));
    RCCHECK(rcl_publisher_fini(&wheel4_pub, &node));
	RCCHECK(rcl_node_fini(&node));

    return 0;
}
