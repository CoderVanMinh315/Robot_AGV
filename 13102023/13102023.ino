/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */
 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
////////////////// Tick Data Publishing Variables and Constants ///////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 3 //2
#define ENC_IN_RIGHT_A 2 //3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5 //4
#define ENC_IN_RIGHT_B 4 //5

enum MOVE
{
  UPWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  STOP
};

MOVE moving = STOP;
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

int duty_right_val = 0;
int duty_left_val = 0;

double linear = 0;
double ang = 0;
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections Right
const int enA = 11; //11
const int in1 = 10; // 10
const int in2 = 9; //9
  
// Motor B connections Left
const int enB = 6; //6
const int in3 = 7; //8
const int in4 = 8; //7

const int STBY = 12;
 
/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
/////////////////////// Motor Controller Functions ////////////////////////////
void duty_right_value(const std_msgs::Int16& duty) {
   int val = duty.data;
   if (val > 120)
   {
    val = 120;
   }
   else{}
   duty_right_val = val;
  // duty_right_val = 70;
   analogWrite(enA, duty_right_val); 
   
}

void duty_left_value(const std_msgs::Int16& duty) {
   int val = duty.data;
   if (val > 120)
   {
    val = 120;
   }
   else{}
   duty_left_val = val;
  // duty_left_val = 70;
   analogWrite(enB, duty_left_val);
}
  
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  linear = cmdVel.linear.x;
  ang = cmdVel.angular.z;

  double wheel_right = (2 * linear + ang * 0.215 ) / 2 ; //34* MIn 0.13 0.3
  double wheel_left = (2 * linear - ang* 0.215 ) / 2 ; //34*

  if (wheel_right > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (wheel_right < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1,HIGH);
    digitalWrite(in2, HIGH);
  }

  if (wheel_left > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (wheel_left < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else {
    digitalWrite(in3,HIGH);
    digitalWrite(in4, HIGH);
  }

}
 
void set_pwm_values() {
 
  analogWrite(enA, duty_right_val); 
  analogWrite(enB, duty_left_val); 
}
 
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
ros::Subscriber<std_msgs::Int16> duty_right("duty_right", &duty_right_value);
ros::Subscriber<std_msgs::Int16> duty_left ("duty_left", &duty_left_value);
 
void setup() {

  TCCR2B = TCCR2B & B11111000 | B00000011; // for PWM frequency of 980.39 Hz
 //TCCR1B = TCCR1B & B11111000 | B00000010;
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
   
  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(STBY, OUTPUT);

  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  digitalWrite(STBY, HIGH);

  
  // Set the motor speed
  analogWrite(enA, 0); 
  analogWrite(enB, 0);
 
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  nh.subscribe(duty_right);
  nh.subscribe(duty_left);
}
 
void loop() {
   
  nh.spinOnce();
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );   
  }
    
 // set_pwm_values();
}
