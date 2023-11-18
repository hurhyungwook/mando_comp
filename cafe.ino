#include <MsTimer2.h>                 // Timer를 실행하기 위한 라이브러리 추가  
#define m_2_pulse     348             // 1m 당 pulse 수  확인 해야 함

#define pulse_2_m     1./348.         // pulse 당 m  확인 해야 함

#define vel_2_pulse   m_2_pulse/20.  // 20Hz 제어 주기에서 속도와 Δpulse 변환 값



///////////////////////////////// ROS 관련  /////////////////////////////////////

#include <ros.h>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Int32.h>

#include <std_msgs/Float32.h>
ros::NodeHandle  nh;

geometry_msgs::Twist cmd_vel;   //모터 명령 수신을 위한 변수(수신) 

std_msgs::Int32 encoder_data1;  //모터 엔코터1 값 전달을 위한 변수(송신)

std_msgs::Int32 encoder_data2;  //모터 엔코터2 값 전달을 위한 변수(송신)

std_msgs::Float32 sonar_data1;

void cmd_vel_callback(const geometry_msgs::Twist& msg);



ros::Subscriber<geometry_msgs::Twist> cmd_sub("teleop_cmd_vel", cmd_vel_callback);

ros::Publisher cmd_pub("cmd_vel2", &cmd_vel);              //cmd_vel 수신확인

ros::Publisher encoder_pub1("encoder1", &encoder_data1);

ros::Publisher encoder_pub2("encoder2", &encoder_data2);

ros::Publisher sonar_pub1("sonar1", &sonar_data1);



int steer_angle = 0;


///////////////////////////////// Sonar Sensor  /////////////////////////////////////

#include <NewPing.h>                  // Timer를 실행하기 위한 라이브러리 추가

#define SONAR_NUM 1                   // Number of sensors.

#define MAX_DISTANCE 200              // Maximum distance (in cm) to ping.


NewPing sonar[SONAR_NUM] = {   // Sensor object array.

  NewPing(13, 12, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 



};


// Front Motor Drive

#define MOTOR1_PWM 5

#define MOTOR1_ENA 6

#define MOTOR1_ENB 7


#define MOTOR2_PWM 2

#define MOTOR2_ENA 3

#define MOTOR2_ENB 4

int f_speed = 0, r_speed = 0;

int front_motor_pwm = 0;

int rear_motor_pwm = 0;


void front_motor_control(int motor1_pwm)

{

  if (motor1_pwm > 0) // forward

  {

    digitalWrite(MOTOR1_ENA, HIGH);

    digitalWrite(MOTOR1_ENB, LOW);

    analogWrite(MOTOR1_PWM, motor1_pwm);

  }

  else if (motor1_pwm < 0) // backward

  {

    digitalWrite(MOTOR1_ENA, LOW);

    digitalWrite(MOTOR1_ENB, HIGH);

    analogWrite(MOTOR1_PWM, -motor1_pwm);

  }

  else

  {

    digitalWrite(MOTOR1_ENA, LOW);

    digitalWrite(MOTOR1_ENB, LOW);

    digitalWrite(MOTOR1_PWM, 0);

  }

}


void rear_motor_control(int motor2_pwm)

{

   if (motor2_pwm > 0) // forward

  {

    digitalWrite(MOTOR2_ENA, HIGH);

    digitalWrite(MOTOR2_ENB, LOW);

    analogWrite(MOTOR2_PWM, motor2_pwm);

  }

  else if (motor2_pwm < 0) // backward

  {

    digitalWrite(MOTOR2_ENA, LOW);

    digitalWrite(MOTOR2_ENB, HIGH);

    analogWrite(MOTOR2_PWM, -motor2_pwm);

  }

  else

  {

    digitalWrite(MOTOR2_ENA, LOW);

    digitalWrite(MOTOR2_ENB, LOW);

    digitalWrite(MOTOR2_PWM, 0);

  }

}


void motor_control(int front_speed, int rear_speed)

{

  front_motor_control(front_speed);

  rear_motor_control(rear_speed);  

}


#include <SPI.h>

#define ENC1_ADD 23  //원래는 22
 
#define ENC2_ADD 22   //원래는 23


signed long encoder1count = 0;

signed long encoder2count = 0;


signed long encoder1_error = 0;

signed long encoder2_error = 0;


signed long encoder1_error_d = 0;

signed long encoder2_error_d = 0;


signed long encoder1_target = 0;

signed long encoder2_target = 0;


signed long encoder1_error_old = 0; 

signed long encoder2_error_old = 0; 


signed long encoder1_error_sum = 0; 

signed long encoder2_error_sum = 0; 


float target_velocity1 = 0.0;

float target_velocity2 = 0.0;


void initEncoders() {

  

  // Set slave selects as outputs

  pinMode(ENC1_ADD, OUTPUT);

  pinMode(ENC2_ADD, OUTPUT); 

  

  // Raise select pins

  // Communication begins when you drop the individual select signsl

  digitalWrite(ENC1_ADD,HIGH);

  digitalWrite(ENC2_ADD,HIGH);

 

  SPI.begin();

  

  // Initialize encoder 1

  //    Clock division factor: 0

  //    Negative index input

  //    free-running count mode

  //    x4 quatrature count mode (four counts per quadrature cycle)

  // NOTE: For more information on commands, see datasheet

  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation

  SPI.transfer(0x88);                       // Write to MDR0

  SPI.transfer(0x03);                       // Configure to 4 byte mode

  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 


  // Initialize encoder 2

  //    Clock division factor: 0

  //    Negative index input

  //    free-running count mode

  //    x4 quatrature count mode (four counts per quadrature cycle)

  // NOTE: For more information on commands, see datasheet

  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation

  SPI.transfer(0x88);                       // Write to MDR0

  SPI.transfer(0x03);                       // Configure to 4 byte mode

  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 

}


long readEncoder(int encoder_no) 

{  

  // Initialize temporary variables for SPI read

  unsigned int count_1, count_2, count_3, count_4;

  long count_value;   

  

  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation

   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation

  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 

  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation

// Calculate encoder count

  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;

  

  return count_value;

}


void clearEncoderCount(int encoder_no) {    

  // Set encoder1's data register to 0

  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  

  // Write to DTR

  SPI.transfer(0x98);    

  // Load data

  SPI.transfer(0x00);  // Highest order byte

  SPI.transfer(0x00);           

  SPI.transfer(0x00);           

  SPI.transfer(0x00);  // lowest order byte

  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 

  

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  

  // Set encoder1's current data register to center

  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  

  SPI.transfer(0xE0);    

  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 

}

/////////////////////////////////////// motor PID 제어  ////////////////////////////////////////////


float Kp_motor = 2;

float Kd_motor = 4;

float Ki_motor = 2;


void front_motor_PID_control(void)

{

  encoder1count = readEncoder(1);

  encoder1_error = encoder1_target - encoder1count;

  encoder1_error_sum += encoder1_error;


  encoder1_error_d = encoder1_error - encoder1_error_old;

  encoder1_error_sum = (encoder1_error_sum >=  100) ?  100 : encoder1_error_sum;

  encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;

  

  front_motor_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;

  front_motor_pwm = (front_motor_pwm >=  255) ?  255 : front_motor_pwm;

  front_motor_pwm = (front_motor_pwm <= -255) ? -255 : front_motor_pwm;

  

  if (fabs(encoder1_error) <= 2)

  {

    encoder1_error_sum = 0;

   //여기서 엔코더를 reset 해야 함 아니면 계속 overflow 발생

    clearEncoderCount(1);     

    //reset후에 target 값을 다시 절정해야 함

    encoder1_target = 0;

  }

  else 

  {

    front_motor_control(front_motor_pwm);    

  }  

  encoder1_error_old = encoder1_error; 

}


///////////////////////////////////////  Steering PID 제어 /////////////////////////////////////////////

#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to

#define NEURAL_ANGLE 0

#define LEFT_STEER_ANGLE  -30

#define RIGHT_STEER_ANGLE  30

#define MOTOR3_PWM 8

#define MOTOR3_ENA 9

#define MOTOR3_ENB 10

#define AD_MIN ( 40 + 10)

#define AD_MAX (940 - 10)


float Kp = 15.5;

float Ki = 3.5;

float Kd = 6.5; //PID 상수 설정, 실험에 따라 정해야 함 중요!

double Setpoint, Input, Output; //PID 제어 변수

double error, error_old;

double error_s, error_d;

int pwm_output;


int sensorValue = 0;        // value read from the pot

int Steer_Angle_Measure = 0;        // value output to the PWM (analog out)

int Steering_Angle = NEURAL_ANGLE;


void steer_motor_control(int motor_pwm)

{

  if( (sensorValue>= AD_MAX  ) || (sensorValue <= AD_MIN  ) )

  {

     digitalWrite(MOTOR3_ENA, LOW);

     digitalWrite(MOTOR3_ENB, LOW);

     analogWrite(MOTOR3_PWM, 0);

     return;    

  }

  

  if (motor_pwm > 0) // forward

  {

    digitalWrite(MOTOR3_ENA, LOW);

    digitalWrite(MOTOR3_ENB, HIGH);

    analogWrite(MOTOR3_PWM, motor_pwm);

  }

  else if (motor_pwm < 0) // backward

  {

    digitalWrite(MOTOR3_ENA, HIGH);

    digitalWrite(MOTOR3_ENB, LOW);

    analogWrite(MOTOR3_PWM, -motor_pwm);

  }

  else // stop

  {

    digitalWrite(MOTOR3_ENA, LOW);

    digitalWrite(MOTOR3_ENB, LOW);

    analogWrite(MOTOR3_PWM, 0);

  }

}


void PID_Control()

{

  error = Steering_Angle - Steer_Angle_Measure ;

  error_s += error;

  error_d = error - error_old;

  error_s = (error_s >=  100) ?  100 : error_s;

  error_s = (error_s <= -100) ? -100 : error_s;


  pwm_output = Kp * error + Kd * error_d + Ki * error_s;

  pwm_output = (pwm_output >=  255) ?  255 : pwm_output;

  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;


  if (fabs(error) <= 0.2)  //특정 값 이하면 제어를 멈추어서 조정이 안되도록

  {

    steer_motor_control(0);

    error_s = 0;

  }

  else          steer_motor_control(pwm_output);

  error_old = error;  

}


void steering_control()

{

  if (Steering_Angle <= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;

  if (Steering_Angle >= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;

  PID_Control(); 

}


void control_callback()

{

  static boolean output = HIGH;

  encoder1_target += target_velocity1 * vel_2_pulse; 

  digitalWrite(13, output);

  output = !output;

  front_motor_PID_control();

}


void cmd_vel_callback(const geometry_msgs::Twist& msg)  // ROS에서 cmd_vel과 연결

{

  target_velocity1 = (float)msg.linear.x;       // 속도제어

  target_velocity2 = target_velocity1;

  steer_angle = (int)msg.angular.z;   // steering motor 각도 제어


  target_velocity1 = (target_velocity1 >= 1.0) ?  1.0 : target_velocity1;

  target_velocity1 = (target_velocity1 <=-1.0) ? -1.0 : target_velocity1;


  target_velocity2 = (target_velocity2 >= 1.0) ?  1.0 : target_velocity2;

  target_velocity2 = (target_velocity2 <=-1.0) ? -1.0 : target_velocity2;

}


void setup() {

  // put your setup code here, to run once:

  pinMode(13, OUTPUT);

 // Front Motor Drive Pin Setup

  pinMode(MOTOR1_PWM, OUTPUT);

  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction

  pinMode(MOTOR1_ENB, OUTPUT);


  // Rear Motor Drive Pin Setup

  pinMode(MOTOR2_PWM, OUTPUT);

  pinMode(MOTOR2_ENA, OUTPUT);  // L298 motor control direction

  pinMode(MOTOR2_ENB, OUTPUT);

 

  initEncoders();          // initialize encoder

  clearEncoderCount(1); 

  clearEncoderCount(2); 


   //Steer

  pinMode(MOTOR3_PWM, OUTPUT);

  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction

  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM

  

  ////////////////////////// ROS //////////////////////////////////

  nh.initNode();

  nh.subscribe(cmd_sub);        // ROS subscribe

  //nh.advertise(cmd_pub);

  nh.advertise(encoder_pub1);

  nh.advertise(encoder_pub2); 

  nh.advertise(sonar_pub1);



  

  MsTimer2::set(50, control_callback); // 50ms period

  MsTimer2::start();

}


void loop(){

  // put your main code here, to run repeatedly:

 //float target_distance = 0.1;

 //target_velocity1  = 0.1;  //0.1m/sec

 Serial.print("Encoder 1 : ");    Serial.println(encoder1count);

 Serial.print("Encoder 2 : ");    Serial.println(encoder2count);

 Serial.print("Encoder error 1 : ");    Serial.println(encoder1_error);

 Serial.print("Encoder error 2 : ");    Serial.println(encoder2_error);


 // ROS publish encoder topic

 encoder_pub1.publish(&encoder_data1);

 encoder_pub2.publish(&encoder_data2);

}   
