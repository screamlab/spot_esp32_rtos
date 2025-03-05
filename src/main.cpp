#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

#include <armDriver.hpp>

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// publisher
rclc_executor_t executor_pub;
rcl_publisher_t publisher;
rcl_timer_t timer;

// subscriber
rclc_executor_t executor_sub;
rcl_subscription_t subscriber;

const unsigned int SPOT_MOTOR_ANGLES_SIZE = 12;
trajectory_msgs__msg__JointTrajectoryPoint spot_motor_angles;
double spot_motor_angles_data[SPOT_MOTOR_ANGLES_SIZE] = { 90.0, 90.0, 90.0, 90.0, 90.0, 90.0,
                                                          90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

bool micro_ros_init_successful;

#define SPOT_MOTOR_ANGLES_SIZE 12
const char* spotMotorAnglesTopic = "spot_motor_angles";
const char* spotActionTopic = "spot_actions";
states state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    for (int i = 0; i < SPOT_MOTOR_ANGLES_SIZE; i++) {
      spot_motor_angles.positions.data[i] = spot_motor_angles_data[i];
    }
    RCSOFTCHECK(rcl_publish(&publisher, &spot_motor_angles, NULL));
  }
}

void subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const trajectory_msgs__msg__JointTrajectoryPoint * msg = (const trajectory_msgs__msg__JointTrajectoryPoint *)msgin;

  // Process message
  for (int i = 0; i < SPOT_MOTOR_ANGLES_SIZE; i++) {
    spot_motor_angles_data[i] = degrees(msg->positions.data[i]);
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "spot_publisher_rclc", "", &support));


  // === Initialize Publisher ===
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
    spotMotorAnglesTopic));

  // Initialize Timer
  const unsigned int timer_timeout = 1000; // 1-second interval
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Add timer to executor
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  // === Initialize Subscriber ===
  executor_sub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
    spotActionTopic));

  // Add subscriber to executor
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &spot_motor_angles, &subscription_callback, ON_NEW_DATA));
  
  // Allocate memory for message (Motor Angles)
  spot_motor_angles.positions.capacity = SPOT_MOTOR_ANGLES_SIZE;
  spot_motor_angles.positions.size = SPOT_MOTOR_ANGLES_SIZE;
  spot_motor_angles.positions.data = (double*)calloc(SPOT_MOTOR_ANGLES_SIZE, sizeof(double));
  
  // Check if memory allocation was successful
  if (spot_motor_angles.positions.data == NULL) {
    return false;
  }

  return true; // successfully created entities
}

void destroy_entities()
{
  // Set session destroy timeout to 0
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Free publisher resources
  RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rclc_executor_fini(&executor_pub));
  RCSOFTCHECK(rclc_executor_fini(&executor_sub));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  // Free allocated memory for message
  if (spot_motor_angles.positions.data != NULL) {
    free(spot_motor_angles.positions.data);
    spot_motor_angles.positions.data = NULL;  // Prevent dangling pointer
  }
}

void microROSTaskFunction(void *parameter) {
  while (true) {
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));
          rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100));
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
    }

    if (state == AGENT_CONNECTED) {
      digitalWrite(LED_PIN, 1);
    } else {
      digitalWrite(LED_PIN, 0);
    } 
  }
}

void armControlTaskFunction(void *parameter) {
  ArmManager armManager(uint8_t(SPOT_MOTOR_ANGLES_SIZE), servoMinAngles, servoMaxAngles, servoInitAngles);

  while (true) {
      for (size_t i = 0; i < SPOT_MOTOR_ANGLES_SIZE; ++i) {
          armManager.setServoTargetAngle(i, uint8_t(spot_motor_angles_data[i]));
      }
      armManager.moveArm();

      // Wait for some time before the next iteration
      vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(1000);
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;


  Serial.println("Starting micro-ROS...");

  xTaskCreate(
    microROSTaskFunction,   // Function to implement the task
    "microROSTaskFunction", // Name of the task
    8192,                   // Stack size in words
    NULL,                   // Task input parameter
    5,                      // Priority of the task
    NULL                    // Task handle.
  );
  delay(100);
  xTaskCreate(
    armControlTaskFunction, // Function to implement the task
    "armControlTaskFunction", // Name of the task
    4096,                   // Stack size in words
    NULL,                   // Task input parameter
    2,                      // Priority of the task
    NULL                    // Task handle.
  );
}

void loop() {
  // Empty loop
}
