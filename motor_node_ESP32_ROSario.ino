// Include Libraries to be used
#include <micro_ros_arduino.h>    //micro-ros-arduino library
#include <rcl/rcl.h>              //Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>   //Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>            //Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>        //Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/float32.h>   //Predefined ROS 2 message type
#include <stdio.h>                //Standard I/O library for debugging.

//Declare nodes to be used
rcl_node_t node;            //Represents a ROS 2 Node running on the microcontroller.

//Instantiate executor and its support classes
rclc_executor_t executor;   //Manages task execution (timers, callbacks, etc.).
rclc_support_t support;     //Data structure that holds the execution context of Micro-ROS, including its communication state, memory management, and initialization data.
rcl_allocator_t allocator;  //Manages memory allocation.

//Declare Subscribers to be used
rcl_subscription_t subscriber;

//Declare Messages to be used
std_msgs__msg__Float32 msg;  //Defines a message of type float32.

//Define Macros to be used
//Executes fn and goes to error_loop() function if fn fails.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Specifies GPIO pin 12 for controlling an LED
#define LED_PIN 12 // Define LED_PIN

#define PWM_PIN 26 // DEFINE PWM_PIN
#define In1 14  // Input 1 (In3 pin in L298N H-bridge) 
#define In2 27  // Input 2 (In4 pin in L298N H-bridge)

#define PWM_FRQ 5000 // Define PWM Frequency
#define PWM_RES 8  // Define PWM Resolution
#define PWM_CHNL 0    // Define Channel

#define MSG_MIN_VAL -1 // Define min input value
#define MSG_MAX_VAL 1 // Define max input value


//Variables to be used
float pwm_set_point = 0.0;

int pwm_value,  // Store data convertion to 8 bits scale
    prev_sign = 0,  // Store previous value sign
    sign = 0, // Store actual value sign
    prev_val = 0, // Store previous value
    time_delay = 10; // Set delay time (ms)

bool state = false; // State change flag

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED state
    delay(100); // Wait 100 milliseconds
  }
}

// Define callbacks
void subscription_callback(const void * msgin)
{  
  // Get the message received and store it on the message msg
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  pwm_set_point = constrain(msg->data, MSG_MIN_VAL, MSG_MAX_VAL);
  
  if (pwm_set_point == 0){
    sign = 0;
  }else if (pwm_set_point < 0){
    sign = -1;
  }else if (pwm_set_point > 0){
    sign = 1;
  }
  
  if(abs(sign - prev_sign) == 2){
    state = true;
  }else{
    state = false;
  }

  // Compare the input value to define de spin direction
  if (sign > 0){
    if(state){
      for(int i = prev_val; i >= 0; i--){
        ledcWrite(PWM_CHNL, i);
        delay(time_delay);
      }

      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      delay(time_delay);
    }

    // Right spin
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  }else if (sign < 0) {
    if(state){
      for(int i = prev_val; i >= 0; i--){
        ledcWrite(PWM_CHNL, i);
        delay(time_delay);
      }

      digitalWrite(In1, LOW);
      digitalWrite(In2, LOW);
      delay(time_delay);
    }
    
    // Left spin
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    
  }

  pwm_value = (int)(fabs(pwm_set_point) * 255);  

  if(state){
    for(int i = 0; i <= pwm_value; i++){
        ledcWrite(PWM_CHNL, i);
        delay(time_delay);
    }
    state = false;
  }else{
    if(pwm_value > prev_val){
      for(int i = prev_val; i <= pwm_value; i++){
        ledcWrite(PWM_CHNL, i);
        delay(time_delay);
      }
    }else{
      for(int i = prev_val; i >= pwm_value; i--){
        ledcWrite(PWM_CHNL, i);
        delay(time_delay);
      }
    }
  }
  prev_sign = sign;
  prev_val = pwm_value;
}

void setup() {
  set_microros_transports(); // Initializes communication between ESP32 and the ROS 2 agent (Serial).
  
  // Setup Microcontroller Pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  pinMode(In1, OUTPUT); // Setup In1
  pinMode(In2, OUTPUT); // Setup In2
  
  // Set no direction
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);

  pinMode(PWM_PIN, OUTPUT);
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);  // Setup the PWM
  ledcAttachPin(PWM_PIN, PWM_CHNL);       // Setup Attach the Pin to the Channel   
  
  // Connection delay
  delay(2000);
  
  // Initializes memory allocation for Micro-ROS operations.
  allocator = rcl_get_default_allocator();

  // Creates a ROS 2 support structure to manage the execution context.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_sub_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm_ROSario")); // Tópico

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Register suscription with executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Executor Spin
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  // Executor Spin
}
