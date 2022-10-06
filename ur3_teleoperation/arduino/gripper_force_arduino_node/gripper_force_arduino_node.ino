// Lectura de fuerza 2Kg - unit test
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define FSR1 A0
#define FSR2 A1

ros::NodeHandle nd;

SimpleKalmanFilter r1Filter = SimpleKalmanFilter(2, 2, 0.1);
SimpleKalmanFilter r2Filter = SimpleKalmanFilter(2, 2, 0.1);

float read_1;
float read_2;
float filtRead1;
float filtRead2;
float force_1;
float force_2;
std_msgs::Float32 force;

ros::Publisher force_publisher("gripper_force", &force);

void setup() {
  Serial.begin(115200);
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

  force.data = (force_1 + force_2)/2.0;

  force_publisher.publish(&force);
  nd.spinOnce();

  delay(50);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
