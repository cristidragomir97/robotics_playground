// Define ROS buffer sizes before including ros.h
#define ROS_RX_BUFFER_SIZE 256
#define ROS_TX_BUFFER_SIZE 256

// Include system headers
#include <Arduino.h>
#include <Wire.h>

// Include the NewPing library
#include <NewPing.h>

// Include ros.h before other ROS headers
#include <ros.h>

// Include ROS message headers
#include <sensor_msgs/Range.h>

// Ultrasonic sensor definitions
#define MAX_DISTANCE 450 // Maximum distance (in cm) to ping

// Pin assignments (keeping the same pins)
#define CENTER_TRIG_PIN 4
#define CENTER_ECHO_PIN 3

#define LEFT_TRIG_PIN 8
#define LEFT_ECHO_PIN 7

#define RIGHT_TRIG_PIN 5
#define RIGHT_ECHO_PIN 6

// Create NewPing objects
NewPing SonarCenter(CENTER_TRIG_PIN, CENTER_ECHO_PIN, MAX_DISTANCE);
NewPing SonarLeft(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing SonarRight(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

ros::NodeHandle nh;

// Create range messages and publishers for each sensor
sensor_msgs::Range center_range_msg;
ros::Publisher center_range_pub("/sonar/center", &center_range_msg);

sensor_msgs::Range left_range_msg;
ros::Publisher left_range_pub("/sonar/left", &left_range_msg);

sensor_msgs::Range right_range_msg;
ros::Publisher right_range_pub("/sonar/right", &right_range_msg);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize ROS node
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  // Initialize range messages
  initializeRangeMessage(center_range_msg, "sonar_center");
  initializeRangeMessage(left_range_msg, "sonar_left");
  initializeRangeMessage(right_range_msg, "sonar_right");

  // Advertise the publishers
  nh.advertise(center_range_pub);
  nh.advertise(left_range_pub);
  nh.advertise(right_range_pub);

  // Wait until ROS node is connected
  while (!nh.connected()) {
    nh.spinOnce();
    delay(10);
  }
}

void loop() {
  if (nh.connected()) {
    readAndPublishSonarData();
  }

  nh.spinOnce();

  // Control loop rate
  delay(20); // Approximately 20 Hz
}

void initializeRangeMessage(sensor_msgs::Range& range_msg, const char* frame_id) {
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.261799; // Approx 15 degrees in radians
  range_msg.min_range = 0.04; // Minimum distance in meters
  range_msg.max_range = (float)MAX_DISTANCE / 100.0; // Convert cm to meters
  range_msg.header.frame_id = frame_id;
}

void readAndPublishSonarData() {
  // Read distance from center sonar
  unsigned int center_distance_cm = SonarCenter.ping_cm();
  float center_distance_m = (center_distance_cm > 0) ? (float)center_distance_cm / 100.0 : center_range_msg.max_range;

  center_range_msg.header.stamp = nh.now();
  center_range_msg.range = center_distance_m;
  center_range_pub.publish(&center_range_msg);

  // Read distance from left sonar
  unsigned int left_distance_cm = SonarLeft.ping_cm();
  float left_distance_m = (left_distance_cm > 0) ? (float)left_distance_cm / 100.0 : left_range_msg.max_range;

  left_range_msg.header.stamp = nh.now();
  left_range_msg.range = left_distance_m;
  left_range_pub.publish(&left_range_msg);

  // Read distance from right sonar
  unsigned int right_distance_cm = SonarRight.ping_cm();
  float right_distance_m = (right_distance_cm > 0) ? (float)right_distance_cm / 100.0 : right_range_msg.max_range;

  right_range_msg.header.stamp = nh.now();
  right_range_msg.range = right_distance_m;
  right_range_pub.publish(&right_range_msg);
}
