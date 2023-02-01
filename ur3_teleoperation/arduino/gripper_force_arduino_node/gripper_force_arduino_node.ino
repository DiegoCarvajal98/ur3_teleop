/*
Arduino ROS node for reading force measures on the Robotiq 2F-85
gripper coupled to the UR3 robot
*/

// Include libraries
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <std_msgs/Float32.h>

// Define force sensors input pins
#define FSR1 A0
#define FSR2 A1

// Kalman filter constants
const float a = 98.5202077002541;
float b = 98.5202077002541;
const float c = 0.5;

// Define ROS Node Handle
ros::NodeHandle nd;

// Define kalman filters
SimpleKalmanFilter r1Filter = SimpleKalmanFilter(a, b, c);
SimpleKalmanFilter r2Filter = SimpleKalmanFilter(a, b, c);

// Define variables
float read_1;
float read_2;
float filtRead1;
float filtRead2;
float force_1;
float force_2;
std_msgs::Float32 force;

// Define force publisher
ros::Publisher force_publisher("gripper_force", &force);

void setup() {
  /*
  Initialize ROS node and advertise force message
  */
  nd.initNode();
  nd.advertise(force_publisher);
}

void loop() {
  read_1 = analogRead(FSR1);
  read_2 = analogRead(FSR2);

  filtRead1 = r1Filter.updateEstimate(read_1);
  filtRead2 = r2Filter.updateEstimate(read_2);

  force_1 = fmap(filtRead1, 0, 1023, 0, 2000);
  force_2 = fmap(filtRead2, 0, 1023, 0, 2000);

  // force.data = (force_1 + force_2)/2.0;
  force.data = force_1;

  force_publisher.publish(&force);
  nd.spinOnce();

  delay(50);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  /*
  Mapping function for float numbers
  */
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
