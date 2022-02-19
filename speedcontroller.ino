#include <ros.h>
#include <std_msgs/Int16.h>

#include <std_msgs/Float32.h>
#include "Arduino.h"

#include <util/atomic.h>


ros::NodeHandle nh;


// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float kp = 5;
float ki = 10;
float eintegral = 0;
float vt = 0;


void setspeed(const std_msgs::Int16 data) 
{
  vt = data.data;
}

void setkp(const std_msgs::Float32 data) 
{
  kp = data.data;
}


void setki(const std_msgs::Float32 data) 
{
  ki = data.data;
}

std_msgs::Float32 a;



std_msgs::Int16 actual_speed;
ros::Publisher actual_speed_pub("actual_speed", &actual_speed);

ros::Publisher posss("lwheel", &a);
ros::Subscriber<std_msgs::Int16> rpm("speed", &setspeed);

ros::Subscriber<std_msgs::Float32> kp_("kp", &setkp);
ros::Subscriber<std_msgs::Float32> ki_("ki", &setki);


void setup() {


  nh.initNode();

  nh.subscribe(rpm);
  
  nh.subscribe(kp_);
  nh.subscribe(ki_);
  
  nh.advertise(posss);
  
  nh.advertise(actual_speed_pub);
  //Serial.begin(115200);
  
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}


void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }
  
  a.data = pos;
  posss.publish(&a);

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  actual_speed.data = v2;
  actual_speed_pub.publish(&actual_speed);

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;


  // Compute the control signal u

  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);

  //Serial.print(vt);
  //Serial.print(" ");
  //Serial.print(v1Filt);
  //Serial.println();
  nh.spinOnce();
  delay(1);
}




void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){     
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
