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
#include "pico/util/queue.h"

#include "include/micro_ros_motor_ctrl/mecanumbot.h"
#include "include/micro_ros_motor_ctrl/pico_uart_transports.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n\r",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n\r",__LINE__,(int)temp_rc);}}

struct Mecanumbot *mecanumbot;

rcl_publisher_t pubFrontRightTicks;
rcl_publisher_t pubFrontLeftTicks;
rcl_publisher_t pubRearRightTicks;
rcl_publisher_t pubRearLeftTicks;

typedef struct {
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
} queue_node_req_t;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	printf("Last callback time: %ld, Motor Ticks: \n\r", 
        last_call_time
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

#define FLAG_VALUE 123
queue_t node1_queue;

void core1_entry() {
    queue_node_req_t node1;
    rcl_timer_t timer;
    rclc_executor_t executor;
    
    multicore_fifo_push_blocking(FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();
    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n\r");
    else{
        queue_remove_blocking(&node1_queue, &node1);

        gpio_set_irq_enabled(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true);

        gpio_add_raw_irq_handler(BRIDGE1_ENCA, wheel1_enc_tick);
        gpio_add_raw_irq_handler(BRIDGE1_ENCB, wheel2_enc_tick);
        gpio_add_raw_irq_handler(BRIDGE2_ENCA, wheel3_enc_tick);
        gpio_add_raw_irq_handler(BRIDGE2_ENCB, wheel4_enc_tick);
        
        irq_set_enabled(IO_IRQ_BANK0, true);

        RCSOFTCHECK(rclc_executor_init(
            &executor,
            &node1.support.context, 
            1, 
            &node1.allocator
        ));

        RCSOFTCHECK(rclc_publisher_init_default(&pubFrontRightTicks, &node1.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_right_ticks"));
        // RCSOFTCHECK(rclc_publisher_init_default(&pubFrontLeftTicks, &node1.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_left_ticks"));
        // RCSOFTCHECK(rclc_publisher_init_default(&pubRearRightTicks, &node1.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_right_ticks"));
        // RCSOFTCHECK(rclc_publisher_init_default(&pubRearLeftTicks, &node1.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_left_ticks"));
        
        const unsigned int timer_period = RCL_MS_TO_NS(100);
        RCSOFTCHECK(rclc_timer_init_default(&timer, &node1.support, timer_period, timer_callback));
        RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
        printf("Its all gone well on core 1!\n\r");

        // rclc_executor_spin(&executor);
        while (true) {
            // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }

        RCSOFTCHECK(rclc_executor_fini(&executor));
        RCSOFTCHECK(rcl_publisher_fini(&pubFrontRightTicks, &node1.node));   
        RCSOFTCHECK(rcl_publisher_fini(&pubFrontLeftTicks, &node1.node));   
        RCSOFTCHECK(rcl_publisher_fini(&pubRearRightTicks, &node1.node));   
        RCSOFTCHECK(rcl_publisher_fini(&pubRearLeftTicks,  &node1.node));    
        RCSOFTCHECK(rcl_node_fini(&node1.node));
        RCSOFTCHECK(rcl_timer_fini(&timer));
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


    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    queue_init(&node1_queue, sizeof(queue_node_req_t), 2);

    // // Turn off FIFO's - we want to do this character by character
    // uart_set_fifo_enabled(UART_ID, false);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    multicore_launch_core1(core1_entry);
    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n\r");
    else {
        mecanumbot_init(&mecanumbot);

        rcl_node_t node0;
        RCCHECK(rclc_node_init_default(&node0, "motor_controller", "", &support));
        rcl_node_t node1;
        RCCHECK(rclc_node_init_default(&node1, "encoder_node", "", &support));
        
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("It's all gone well on core 0!\n\r");
        
        queue_node_req_t node_req = {node1, allocator, support};
        queue_add_blocking(&node1_queue, &node_req);
        
        rclc_executor_t executor;
        RCCHECK(rclc_executor_init(
            &executor,
            &support.context, 
            1, 
            &allocator
        ));

        rcl_subscription_t subscriber;
        geometry_msgs__msg__Twist twist_msg;
        RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node0,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"
        ));
        RCCHECK(rclc_executor_add_subscription(
            &executor, 
            &subscriber, 
            &twist_msg,
            &subscription_callback, 
            ON_NEW_DATA
        ));

        set_on(mecanumbot);

        rclc_executor_spin(&executor);

        mecanumbot_destroy(mecanumbot);
        RCCHECK(rcl_node_fini(&node0));
        RCCHECK(rclc_executor_fini(&executor));
        RCCHECK(rcl_subscription_fini(&subscriber, &node0));
        RCCHECK(rcl_publisher_fini(&pubFrontRightTicks, &node0));   
        RCCHECK(rclc_support_fini(&support))
    }

    // printf("%i %i\n\r", ENCODER_MIN, ENCODER_MAX);
    // while (true)
    // {
    //     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    // }

    return 0;
}
