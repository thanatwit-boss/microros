#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

// -------------------- WiFi Config --------------------
// เปลี่ยนจาก const char* เป็น char[] เพื่อให้ตรงกับพารามิเตอร์ char*
char WIFI_SSID[]     = "EMB5325";
char WIFI_PASSWORD[] = "cdti12345";
char AGENT_IP[]      = "192.168.1.119";
uint32_t    AGENT_PORT    = 8888;

// ---------------- Micro ROS ----------------
rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t tf_publisher;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped transform_stamped;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ----------- Encoder pin -----------  
#define LEFT_A 14
#define LEFT_B 12
#define RIGHT_A 34
#define RIGHT_B 35

volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
volatile long lastLeft = 0;
volatile long lastRight = 0;

// ----------- Motor pins -----------  
int PWMA = 25; // M1 PWM
int M1_A = 26;
int M1_B = 27;

int PWMB = 4;  // M2 PWM
int M2_A = 16;
int M2_B = 17;

// ----------- Robot parameters -----------  
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

const float WHEEL_BASE = 63.85;    
const float CM_PER_COUNT = 0.002274;
const float CM_TO_M = 0.01;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { fn; }

// ---------------- ERROR ----------------
void error_loop() {
  pinMode(2, OUTPUT);
  while (1) {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
}

// Quaternion
geometry_msgs__msg__Quaternion yaw_to_quaternion(float yaw) {
  geometry_msgs__msg__Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(yaw / 2.0);
  q.w = cos(yaw / 2.0);
  return q;
}

// ---------------- Motor Control ----------------
void move(int rightSpeed, int leftSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT MOTOR
  if (leftSpeed > 0) {
    digitalWrite(M1_A, HIGH);
    digitalWrite(M1_B, LOW);
    analogWrite(PWMA, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(M1_A, LOW);
    digitalWrite(M1_B, HIGH);
    analogWrite(PWMA, -leftSpeed);
  } else {
    digitalWrite(M1_A, LOW);
    digitalWrite(M1_B, LOW);
    analogWrite(PWMA, 0);
  }

  // RIGHT MOTOR
  if (rightSpeed > 0) {
    digitalWrite(M2_A, LOW);
    digitalWrite(M2_B, HIGH);
    analogWrite(PWMB, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(M2_A, HIGH);
    digitalWrite(M2_B, LOW);
    analogWrite(PWMB, -rightSpeed);
  } else {
    digitalWrite(M2_A, LOW);
    digitalWrite(M2_B, LOW);
    analogWrite(PWMB, 0);
  }
}

// ---------------- Encoder ISR ----------------
void leftEncoderISR() {
  if (digitalRead(LEFT_A) == digitalRead(LEFT_B)) leftEncoder--;
  else leftEncoder++;
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_A) == digitalRead(RIGHT_B)) rightEncoder++;
  else rightEncoder--;
}

// ---------------- Odometry ----------------
void updateOdometry() {
  long L = leftEncoder - lastLeft;
  long R = rightEncoder - lastRight;

  float Ld = L * CM_PER_COUNT;
  float Rd = R * CM_PER_COUNT;

  float dCenter = (Ld + Rd) / 2.0;
  float dTheta = (Rd - Ld) / WHEEL_BASE;

  robotX += dCenter * cos(robotTheta + dTheta / 2.0);
  robotY += dCenter * sin(robotTheta + dTheta / 2.0);
  robotTheta += dTheta;

  while (robotTheta > PI) robotTheta -= 2 * PI;
  while (robotTheta < -PI) robotTheta += 2 * PI;

  lastLeft = leftEncoder;
  lastRight = rightEncoder;
}

// ---------------- cmd_vel callback ----------------
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist*)msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  bool stop = (fabs(linear) < 0.0001 && fabs(angular) < 0.0001);


  // ---------------- Motor Control ----------------
  int left = (linear - angular) * 255;
  int right = (linear + angular) * 255;
  move(right, left);
}

// ---------------- Timer callback ----------------
void timer_callback(rcl_timer_t *timer, int64_t last_call) {
  (void)last_call;
  updateOdometry();

  odom_msg.header.stamp.sec = millis() / 1000;
  odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

  // หมายเหตุ: การกำหนด data ของ string แบบนี้เป็นการชอร์ตคัต อาจใช้ได้
  // แต่ถ้าต้องการปลอดภัย ให้ใช้ฟังก์ชันของ rosidl_runtime_c เพื่อ assign string
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.child_frame_id.data = (char*)"base_link";

  odom_msg.pose.pose.position.x = robotX * CM_TO_M;
  odom_msg.pose.pose.position.y = robotY * CM_TO_M;
  odom_msg.pose.pose.orientation = yaw_to_quaternion(robotTheta);

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  transform_stamped.header = odom_msg.header;
  transform_stamped.child_frame_id.data = (char*)"base_link";

  transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
  transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
  transform_stamped.transform.translation.z = 0;

  transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;

  tf_msg.transforms.size = 1;
  tf_msg.transforms.data = &transform_stamped;

  RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));
}

void setupEncoders() {
  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoderISR, CHANGE);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  // ---- micro-ROS WiFi transport ----
  set_microros_wifi_transports(
    WIFI_SSID,
    WIFI_PASSWORD,
    AGENT_IP,
    AGENT_PORT
  );

  delay(2000);
  Serial.println(WiFi.localIP());

  setupEncoders();

  // motor pins  
  pinMode(PWMA, OUTPUT);
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);

  // ROS init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_robot", "", &support);

  rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");

  rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom");

  rclc_publisher_init_default(
      &tf_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
      "/tf");

  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(100),
      timer_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  rclc_executor_add_subscription(
      &executor,
      &twist_subscriber,
      &twist_msg,
      &cmd_vel_callback,
      ON_NEW_DATA);

  Serial.println("Robot Ready with WiFi micro-ROS!");
}

// ---------------- LOOP ----------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}