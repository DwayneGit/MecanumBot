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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

struct Mecanumbot *mecanumbot;

rcl_subscription_t subscriber;

rcl_publisher_t pubFrontRightTicks;
rcl_publisher_t pubFrontLeftTicks;
rcl_publisher_t pubRearRightTicks;
rcl_publisher_t pubRearLeftTicks;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	printf("Last callback time: %ld, Motor Ticks: \n\r", 
        last_call_time
    );
	printf("%d\n\r",
        mecanumbot->motor1->encoderTickCount
    );
	printf("%d\n\r", 
        mecanumbot->motor2->encoderTickCount
    );
	printf("%d\n\r", 
        mecanumbot->motor3->encoderTickCount
    );
	printf("%d\n\r", 
        mecanumbot->motor4->encoderTickCount
    );

	if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&pubFrontRightTicks, &mecanumbot->motor1->encoderTickCount, NULL));
        RCSOFTCHECK(rcl_publish(&pubFrontLeftTicks, &mecanumbot->motor2->encoderTickCount, NULL));
        RCSOFTCHECK(rcl_publish(&pubRearRightTicks, &mecanumbot->motor3->encoderTickCount, NULL));
        RCSOFTCHECK(rcl_publish(&pubRearLeftTicks,  &mecanumbot->motor4->encoderTickCount, NULL));
	}
}

void subscription_callback(const void * msgin)
{
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
    set_on(mecanumbot);
    set_direction(mecanumbot, msg->linear.x, msg->angular.z);
}

void wheel1_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE1_ENCA) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor1);
    }
}

void wheel2_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE1_ENCB) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor2);
    }
}

void wheel3_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE2_ENCA) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor3);
    }
}

void wheel4_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE2_ENCB) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor4);
    }
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
    rcl_timer_t timer;
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_allocator_t allocator;

    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // // Turn off FIFO's - we want to do this character by character
    // uart_set_fifo_enabled(UART_ID, false);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    gpio_set_irq_enabled(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true);

    gpio_add_raw_irq_handler(BRIDGE1_ENCA, wheel1_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE1_ENCB, wheel2_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE2_ENCA, wheel3_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE2_ENCB, wheel4_enc_tick);
    
    irq_set_enabled(IO_IRQ_BANK0, true);

    struct BiMotor mot1 = bimotor_new(1, BRIDGE1_ENB, BRIDGE1_IN3, BRIDGE1_IN4, BRIDGE1_ENCA, 2000);
    struct BiMotor mot2 = bimotor_new(2, BRIDGE1_ENA, BRIDGE1_IN1, BRIDGE1_IN2, BRIDGE1_ENCB, 2000);
    struct BiMotor mot3 = bimotor_new(3, BRIDGE2_ENA, BRIDGE2_IN1, BRIDGE2_IN2, BRIDGE2_ENCA, 2000);
    struct BiMotor mot4 = bimotor_new(4, BRIDGE2_ENB, BRIDGE2_IN3, BRIDGE2_IN4, BRIDGE2_ENCB, 2000);
   
    mecanumbot_new(&mecanumbot, &mot1, &mot2, &mot3, &mot4);

	printf("%d %d %d %d\n\r",
        mecanumbot->motor1->encoderTickCount,
        mecanumbot->motor2->encoderTickCount,
        mecanumbot->motor3->encoderTickCount,
        mecanumbot->motor4->encoderTickCount
    );
    allocator = rcl_get_default_allocator();
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "motor_controller", "", &support));
	RCCHECK(rclc_executor_init(
        &executor,
        &support.context, 
        1, 
        &allocator
    ));

    RCCHECK(rclc_publisher_init_default(&pubFrontRightTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_right_ticks"));
    RCCHECK(rclc_publisher_init_default(&pubFrontLeftTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_left_ticks"));
    RCCHECK(rclc_publisher_init_default(&pubRearRightTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_right_ticks"));
    RCCHECK(rclc_publisher_init_default(&pubRearLeftTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_left_ticks"));

    // RCCHECK(rclc_subscription_init_default(
    //     &subscriber,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    //     "cmd_vel"
    // ));

    // geometry_msgs__msg__Twist twist_msg;
    // RCCHECK(rclc_executor_add_subscription(
    //     &executor, 
    //     &subscriber, 
    //     &twist_msg,
    //     &subscription_callback, 
    //     ON_NEW_DATA
    // ));
    
    const unsigned int timer_period = RCL_MS_TO_NS(1000);
    RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_callback));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // printf("%i %i\n\r", ENCODER_MIN, ENCODER_MAX);
    rclc_executor_spin(&executor);

    RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&pubFrontRightTicks, &node));   
    RCCHECK(rcl_publisher_fini(&pubFrontLeftTicks, &node));   
    RCCHECK(rcl_publisher_fini(&pubRearRightTicks, &node));   
    RCCHECK(rcl_publisher_fini(&pubRearLeftTicks,  &node));    
	RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rclc_support_fini(&support))

    return 0;
}
