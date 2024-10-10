// Include necessary libraries
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "DualVNH5019MotorShield.h"

// Include FspTimer class
#include <FspTimer.h>


const int8_t encoder_lookup_table[16] = {
  0,  // 0b0000: no movement
  -1, // 0b0001: -1
  1,  // 0b0010: +1
  0,  // 0b0011: invalid
  1,  // 0b0100: +1
  0,  // 0b0101: invalid
  0,  // 0b0110: invalid
  -1, // 0b0111: -1
  -1, // 0b1000: -1
  0,  // 0b1001: invalid
  0,  // 0b1010: invalid
  1,  // 0b1011: +1
  0,  // 0b1100: invalid
  1,  // 0b1101: +1
  -1, // 0b1110: -1
  0   // 0b1111: no movement
};

// Define motor and encoder parameters
#define MIN_VAL -400
#define MAX_VAL 400

double speed_ang = 0.0, speed_lin = 0.0;
double w_r = 0.0, w_l = 0.0;
double wheel_rad = 0.034; // Wheel radius in meters
double wheel_sep = 0.14;  // Distance between wheel centers in meters

// Encoder pins (adjust according to your setup)
#define ENCODER_LEFT_A_PIN A2
#define ENCODER_LEFT_B_PIN A3
#define ENCODER_RIGHT_A_PIN A4
#define ENCODER_RIGHT_B_PIN A5

// Encoder counts
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

// Previous state variables
volatile bool prevLeftA = false;
volatile bool prevLeftB = false;
volatile bool prevRightA = false;
volatile bool prevRightB = false;

// Create an instance of FspTimer
FspTimer encoderTimer;

// ROS node handle
ros::NodeHandle nh;

// Publishers for motor currents
std_msgs::Float32 current_msg_left;
std_msgs::Float32 current_msg_right;
ros::Publisher motor_current_pub_left("motor/current/left", &current_msg_left);
ros::Publisher motor_current_pub_right("motor/current/right", &current_msg_right);

// Publishers for encoder counts
std_msgs::Int32 encoder_count_left_msg;
std_msgs::Int32 encoder_count_right_msg;
ros::Publisher encoder_pub_left("encoder/left", &encoder_count_left_msg);
ros::Publisher encoder_pub_right("encoder/right", &encoder_count_right_msg);

// Publisher for odometry
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

// Motor shield
DualVNH5019MotorShield motor;

// Variables for odometry calculation
long prev_count_left = 0, prev_count_right = 0;
double pos_x = 0.0, pos_y = 0.0, theta = 0.0;
unsigned long prev_time = 0;

// Function prototypes
void messageCb(const geometry_msgs::Twist& msg);
void timerCallback();
void publishCurrent();
void publishEncoders();
void publishOdometry();

// Subscriber for cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

// Callback function for cmd_vel messages
void messageCb(const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}

// Timer callback function for encoder polling
void timer_callback(timer_callback_args_t __attribute((unused)) *p_args) {
  // Read current states
  uint8_t leftA = digitalRead(ENCODER_LEFT_A_PIN);
  uint8_t leftB = digitalRead(ENCODER_LEFT_B_PIN);
  uint8_t rightA = digitalRead(ENCODER_RIGHT_A_PIN);
  uint8_t rightB = digitalRead(ENCODER_RIGHT_B_PIN);

  // Combine previous and current states for left encoder
  uint8_t left_state = (prevLeftA << 3) | (prevLeftB << 2) | (leftA << 1) | leftB;
  int8_t left_increment = encoder_lookup_table[left_state & 0x0F];
  encoderCountLeft += left_increment;

  // Update previous states for left encoder
  prevLeftA = leftA;
  prevLeftB = leftB;

  // Combine previous and current states for right encoder
  uint8_t right_state = (prevRightA << 3) | (prevRightB << 2) | (rightA << 1) | rightB;
  int8_t right_increment = encoder_lookup_table[right_state & 0x0F];
  encoderCountRight += right_increment;

  // Update previous states for right encoder
  prevRightA = rightA;
  prevRightB = rightB;
}

bool beginTimer(float rate) {
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0){
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0){
    return false;
  }

  FspTimer::force_use_of_pwm_reserved_timer();

  if(!encoderTimer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, timer_callback)){
    return false;
  }

  if (!encoderTimer.setup_overflow_irq()){
    return false;
  }

  if (!encoderTimer.open()){
    return false;
  }

  if (!encoderTimer.start()){
    return false;
  }
  return true;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize encoder pins
  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);

  // Read initial states
  prevLeftA = digitalRead(ENCODER_LEFT_A_PIN);
  prevLeftB = digitalRead(ENCODER_LEFT_B_PIN);
  prevRightA = digitalRead(ENCODER_RIGHT_A_PIN);
  prevRightB = digitalRead(ENCODER_RIGHT_B_PIN);

  // Initialize FspTimer
  // Set timer to call timerCallback every 1 millisecond (1000 microseconds)
    // Initialize FspTimer
  // Parameters:
  // - mode: TIMER_MODE_PERIODIC
  // - type: GPT_TIMER
  // - channel: 0 (timer channel)
  // - freq_hz: 1000.0f (frequency in Hz for 1 ms interval)
  // - duty_perc: 0.0f (not used in periodic mode)
  // - cbk: timerCallback (the callback function)
  // - ctx: nullptr (no context needed)
  beginTimer(10000);

  // Initialize ROS node and publishers/subscribers
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(motor_current_pub_left);
  nh.advertise(motor_current_pub_right);
  nh.advertise(encoder_pub_left);
  nh.advertise(encoder_pub_right);
  nh.advertise(odom_pub);

  // Initialize motor shield
  motor.init();

  // Initialize previous time
  prev_time = millis();
}

void loop() {
  // Publish motor currents
  publishCurrent();

  // Publish encoder counts
  publishEncoders();

  // Publish odometry
  publishOdometry();

  // Set motor speeds
  motor.setSpeeds(constrain(w_l * 10, MIN_VAL, MAX_VAL), constrain(w_r * 10, MIN_VAL, MAX_VAL));

  // Spin ROS node
  nh.spinOnce();
}

// Function to publish motor currents
void publishCurrent() {
  current_msg_left.data = motor.getM1CurrentMilliamps();
  current_msg_right.data = motor.getM2CurrentMilliamps();
  motor_current_pub_left.publish(&current_msg_left);
  motor_current_pub_right.publish(&current_msg_right);
}

// Function to publish encoder counts
void publishEncoders() {
  encoder_count_left_msg.data = encoderCountLeft;
  encoder_count_right_msg.data = encoderCountRight;
  encoder_pub_left.publish(&encoder_count_left_msg);
  encoder_pub_right.publish(&encoder_count_right_msg);
}

// Function to publish odometry
void publishOdometry() {
  unsigned long current_time = millis();
  double dt = (current_time - prev_time) / 1000.0; // Convert milliseconds to seconds
  prev_time = current_time;

  // Copy encoder counts atomically
  noInterrupts();
  long count_left = encoderCountLeft;
  long count_right = encoderCountRight;
  interrupts();

  // Calculate delta counts
  long delta_left = count_left - prev_count_left;
  long delta_right = count_right - prev_count_right;

  prev_count_left = count_left;
  prev_count_right = count_right;

  // Encoder counts per wheel revolution (replace with your encoder's CPR)
  double counts_per_rev = 1000.0;  // Update with your encoder's counts per revolution

  // Compute wheel distances
  double dist_per_count = (2 * PI * wheel_rad) / counts_per_rev;
  double dist_left = delta_left * dist_per_count;
  double dist_right = delta_right * dist_per_count;

  // Compute robot's movement
  double delta_s = (dist_right + dist_left) / 2.0;
  double delta_theta = (dist_right - dist_left) / wheel_sep;

  // Update pose
  double delta_x = delta_s * cos(theta + delta_theta / 2.0);
  double delta_y = delta_s * sin(theta + delta_theta / 2.0);

  pos_x += delta_x;
  pos_y += delta_y;
  theta += delta_theta;

  // Normalize theta
  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;

  // Prepare odometry message
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // Set position
  odom_msg.pose.pose.position.x = pos_x;
  odom_msg.pose.pose.position.y = pos_y;
  odom_msg.pose.pose.position.z = 0.0;

  // Convert theta to quaternion
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

  // Set velocities
  odom_msg.twist.twist.linear.x = delta_s / dt;
  odom_msg.twist.twist.angular.z = delta_theta / dt;

  // Publish odometry
  odom_pub.publish(&odom_msg);
}
