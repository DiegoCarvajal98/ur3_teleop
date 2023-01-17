/*
Arduino ROS Node to read the quadrature encoder and 
control the DC motor
*/

// Include libraries
#include <ros.h>
#include <util/atomic.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// Define motor and encoder pins
#define motor       10
#define ENCODER_A   2
#define ENCODER_B   3

// Define ROS Node Handle
ros::NodeHandle nh;

// Define variables
float pwm;

// Global position variable shared with the interruption
volatile int theta = 0;

// Global pulses variable shared with the interruption
volatile int pulsos = 0;
unsigned long timeold;
float resolution = 374.22;

// Global position variable
int ang = 0;

// Motor activation variable
bool aMotor = false;

std_msgs::Float32 angle;

void SubscriberCallback(const std_msgs::Bool& act){
  /*
  Subscriber callback function for the motor activation message
  */
  if(act.data){
    aMotor = true;
  }else{
    aMotor = false;
  }
}

// Define publisher for the encoder and subscriber for the motor activation message
ros::Publisher angle_publisher("tool_angle", &angle);
ros::Subscriber<std_msgs::Bool> motor_subscriber("motor_activation", &SubscriberCallback);

void setup() {
  // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz
  // TCCR1B = TCCR1B & B11111000 | B00000101;

  // Initialize ROS node, publisher and subscriber
  nh.initNode();
  nh.advertise(angle_publisher);
  nh.subscribe(motor_subscriber);
  
  // Configure encoder and motor pins
  pinMode(motor, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  
  timeold = 0;

  // Configure interruption
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), leerEncoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), leerEncoderB,RISING);

}

void loop() {
  float posicion;
  
  // Set the motor PWM according to the activation variable
  if(aMotor){
    pwm = 3*255/5;
    analogWrite(motor, pwm);
  }else{
    pwm = 0;
    analogWrite(motor, pwm);
  }
  
  // Modify the interruption variables atomically
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    posicion = (float(theta * 360.0 /resolution));
  }

  // Publish encoder angle message
  angle.data = posicion;

  angle_publisher.publish(&angle);
  
  nh.spinOnce();
}

void leerEncoderA(){
  /*
  Interruption function for the angle increment
  */
  int b = digitalRead(ENCODER_B);
  if(digitalRead(ENCODER_B == LOW)){
    // Global variable increment
    theta++;
  }
}

void leerEncoderB(){
  /*
  Interruption function for the angle decrement
  */
  int a = digitalRead(ENCODER_A);
  if(digitalRead(ENCODER_A == LOW)){
    // Global variable decrement
    theta--;
  }
}
