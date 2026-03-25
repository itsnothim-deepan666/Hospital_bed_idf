#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Micro-ROS objects
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// --- CALLBACK FUNCTION ---
// This runs every time a message is received
void subscription_callback(const void * msvin) {  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
  
  // Logic: If we receive 1, turn LED ON. If 0, turn LED OFF.
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize support, node, and subscriber
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_subscriber_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "led_control"));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // The executor handles the "spinning" to check for new messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}