// Motor control and encoder position - Unit test

#include <ros.h>
#include <util/atomic.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#define motor       10
#define ENCODER_A   2
#define ENCODER_B   3

ros::NodeHandle nh;

float pwm;

// Variable global de posición compartida con la interrupción
volatile int theta = 0;

// Variable global de pulsos compartida con la interrupción
volatile int pulsos = 0;
unsigned long timeold;
float resolution = 374.22;

// Variable global de posición
int ang = 0;

// Variable de activación del motor
bool aMotor = false;

std_msgs::Float32 angle;

void SubscriberCallback(const std_msgs::Bool& act){
  if(act.data){
    aMotor = true;
    ROSINFO("Motor activated");
  }else{
    aMotor = false;
    ROSINFO("Motor deactivated");
  }
}

ros::Publisher angle_publisher("tool_angle", &angle);
ros::Subscriber<std_msgs::Bool> motor_subscriber("motor_activation", &SubscriberCallback);

void setup() {
  // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;

  nh.initNode();
  nh.advertise(angle_publisher);
  nh.subscribe(motor_subscriber);
  
  pinMode(motor, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // Configurar interrupción
  timeold = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), leerEncoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), leerEncoderB,RISING);

}

void loop() {
  float posicion;
  
  if(aMotor){
    pwm = 2*255/5;
    analogWrite(motor, pwm);
  }else{
    pwm = 0;
    analogWrite(motor, pwm);
  }
  
  //Modifica las variables de la interrupción forma atómica
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    posicion = (float(theta * 360.0 /resolution));
  }

  angle.data = posicion;

  angle_publisher.publish(&angle);
  nh.spinOnce();
  
}

void leerEncoderA(){
  int b = digitalRead(ENCODER_B);
  if(digitalRead(ENCODER_B == LOW)){
    //Decremento variable global
    theta++;
  }
}

void leerEncoderB(){
  int a = digitalRead(ENCODER_A);
  if(digitalRead(ENCODER_A == LOW)){
    //Incremento variable global
    theta--;
  }
}
