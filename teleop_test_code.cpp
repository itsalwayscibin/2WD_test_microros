#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>  // NEW: For Wi-Fi transport

#define LED_BUILTIN 2

// Motor driver pins
#define IN1 5   // Left Motor A
#define IN2 18
#define IN3 19  // Right Motor B
#define IN4 21
#define ENA 23  // Left PWM
#define ENB 22  // Right PWM

// Wi-Fi credentials and agent info
const char* ssid = "younglord";              // 🟡 Replace with your Wi-Fi name
const char* password = "cristiano7"; // 🟡 Replace with your password
char agent_ip[] = "192.168.99.103";           // 🟡 Replace with IP of your PC running micro_ros_agent
uint agent_port = 8888;                       // Same as agent port

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

const int MAX_PWM = 255;

void error_loop() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setMotorSpeeds(int left_speed, int right_speed) {
  if (left_speed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, min(abs(left_speed), MAX_PWM));

  if (right_speed >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, min(abs(right_speed), MAX_PWM));

  Serial.print("Left Motor: "); Serial.print(left_speed);
  Serial.print(" | Right Motor: "); Serial.println(right_speed);
}

void stop_motors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  Serial.println("Motors stopped");
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  float scale = 255.0;
  float wheel_base = 0.15;

  int right_pwm  = (linear - angular * wheel_base / 2.0) * scale;
  int left_pwm = (linear + angular * wheel_base / 2.0) * scale;

  if (linear == 0.0 && angular == 0.0) {
    stop_motors();
  } else {
    setMotorSpeeds(left_pwm, right_pwm);
  }
}

void setup() {
  Serial.begin(115200);

  // NEW: Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  // NEW: Set Wi-Fi transport for micro-ROS
  set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip, agent_port);

  // Motor pin setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  stop_motors();

  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    error_loop();
  }

  if (rclc_node_init_default(&node, "esp32_motor_node", "", &support) != RCL_RET_OK) {
    error_loop();
  }

  if (rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) {
    error_loop();
  }

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    error_loop();
  }

  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) {
    error_loop();
  }
}

void loop() {
  delay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
