#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include <M5Unified.h>

// ROS entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_timer_t timer;
rcl_publisher_t imu_publisher;
rcl_subscription_t cmd_vel_subscriber;

// messages
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// IMU raw data
float accx = 0, accy = 0, accz = 0;
float gyrox = 0, gyroy = 0, gyroz = 0;

// Error macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
  M5.Lcd.setCursor(0, 100); M5.Lcd.printf("ERROR: %s\n", #fn); error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("!!! ERROR !!!");
  while (1) delay(100);
}

// Timer callback: IMU publisher
void imu_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    M5.Imu.getAccelData(&accx, &accy, &accz);
    M5.Imu.getGyroData(&gyrox, &gyroy, &gyroz);

    imu_msg.linear_acceleration.x = accx;
    imu_msg.linear_acceleration.y = accy;
    imu_msg.linear_acceleration.z = accz;

    imu_msg.angular_velocity.x = gyrox;
    imu_msg.angular_velocity.y = gyroy;
    imu_msg.angular_velocity.z = gyroz;

    // optional orientation
    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 1;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

// cmd_vel callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;
  String command = "#M0";

  M5.Lcd.setCursor(0, 80);
  M5.Lcd.fillRect(0, 80, 320, 100, BLACK);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.printf("lin.x: %.2f\n", twist->linear.x);
  M5.Lcd.printf("ang.z: %.2f\n", twist->angular.z);

  if (twist->linear.x == 0.0 && twist->angular.z == 0.0) {
    command = "#M0";
  } else if (twist->linear.x > 0.0 && twist->angular.z == 0.0) {
    command = "#M1";
  } else if (twist->linear.x < 0.0 && twist->angular.z == 0.0) {
    command = "#M2";
  } else if (twist->linear.x == 0.0 && twist->angular.z < 0.0) {
    command = "#M3";
  } else if (twist->linear.x == 0.0 && twist->angular.z > 0.0) {
    command = "#M4";
  } else {
    command = "#M0";
  }

  M5.Lcd.setCursor(0, 160);
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.printf("command: %s\n", command.c_str());

  Serial2.println(command);
}

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Imu.init();
  M5.Lcd.begin();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.clear();
  M5.Lcd.println("Micro-ROS Init...");

  Serial.begin(115200);    // USB debug
  Serial2.begin(57600);    // Motor command out

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "uros_combined_node", "", &support));

  // Publisher /imu
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));

  // Subscriber /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // Timer for IMU publishing
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(timer_timeout), imu_timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  memset(&imu_msg, 0, sizeof(sensor_msgs__msg__Imu));
  M5.Lcd.println("All initialized!");
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
