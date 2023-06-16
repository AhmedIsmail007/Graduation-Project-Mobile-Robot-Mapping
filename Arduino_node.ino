#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h> //For sending encoder msg
#include<geometry_msgs/Twist.h> //For cmd_vel subscription
#include <Encoder.h> //Encoder library

//----- DC Motor definitions------------

#define LM_LPWM  4 
#define LM_RPWM  3
#define RM_LPWM  5
#define RM_RPWM  6

#define GAS_SENSOR  A0

#define LM_R_ENABLE  46
#define LM_L_ENABLE  48
#define RM_R_ENABLE  50
#define RM_L_ENABLE  52

#define FORWARD  2
#define BACKWARD 1

void motorL_init(void);
void motorR_init(void);

void motorL_run(uint8_t dir, uint8_t spd);
void motorR_run(uint8_t dir, uint8_t spd);

//void motorL_setSpeed(uint8_t speed);
//void motorR_setSpeed(uint8_t speed);

//-------------Encoder definitions
Encoder R_enc(10, 11); //right motor encoder goes for pins 19 18
Encoder L_enc(8, 9);   //left motor encoder goes for pins 21 20
long RoldPosition  = -999;
long LoldPosition  = -999;
 //-----------------------------------------Robot parameters definition------------ 
#define L 0.22
#define R 0.04
//--------------------------------Motors VARS-----------------------------------
// initializing variables
float vel=0.0; //as Twist msgs depend on Vector3 which is float64
float omega=0.0;
float VR,VL;

uint8_t LM_speed = 200;
uint8_t RM_speed = 200;

//-----------------------------------------------------------------------------------------
//ros::NodeHandle  nh2;
ros::NodeHandle  nh;
//------------------------------Publish init----------------------------------------------
geometry_msgs::Point32 Point_msg;
std_msgs::Float32 Sensor_msg;

ros::Publisher sensor_pub("/gas_sensor", &Sensor_msg);
ros::Publisher enc_pub("/encoder", &Point_msg);

//-----------------------------------DC Motors Callback subscribers

void motors_cb(const geometry_msgs::Twist& msg)
{
  
    vel=msg.linear.x * 20;    
    omega=msg.angular.z * 20;  
    
     
    VR=(2*vel+omega*L)/(2*R); 
    VL=(2*vel-omega*L)/(2*R); 

    //-----right motor------

    if (VR<0)
    {
       motorR_run(BACKWARD, abs(VR));
       //motorR_setSpeed(abs(VR));   
    }

    else 
    {
      
      motorR_run(FORWARD, abs(VR));  
      //motorR_setSpeed(VR); 
        
    }

    //-----left motor------

     if (VL<0)
    {
       motorL_run(BACKWARD, abs(VL));
       //motorL_setSpeed(abs(VL));   
    }

    else 
    {
      
      motorL_run(FORWARD, abs(VL));  
      //motorL_setSpeed(VL); 
        
    }



}

//--------------------subscribers---------------------------
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);



 void setup() 
{
      Serial.begin (57600);    
      //-----------------------------

       // ENABLE MOTOR
  pinMode(LM_R_ENABLE, OUTPUT);
  pinMode(LM_L_ENABLE, OUTPUT);
  pinMode(RM_R_ENABLE, OUTPUT);
  pinMode(RM_L_ENABLE, OUTPUT);

  digitalWrite(LM_R_ENABLE, HIGH);
  digitalWrite(LM_L_ENABLE, HIGH);
  digitalWrite(RM_R_ENABLE, HIGH);
  digitalWrite(RM_L_ENABLE, HIGH);
       // turn on motor
       motorL_init();
       motorR_init();     

     //---------------------------ROS Setup
      nh.advertise(sensor_pub);
      nh.advertise(enc_pub);
      nh.subscribe(sub);      
      }


 void loop() { 
      
   //Right Encoder
     long RnewPosition = R_enc.read();
     if (RnewPosition != RoldPosition) {
          RoldPosition = RnewPosition; 

          
          //Serial.println(RnewPosition);
        } 
        
  //----left encoder
  long LnewPosition = L_enc.read();
  if (LnewPosition != LoldPosition) {
      LoldPosition = LnewPosition; //update positions
      //Serial.println(LnewPosition);
      }  
//-------end of encoder

//-----------------------ROS publishing  
        float mq2 = analogRead(A0);
        Sensor_msg.data = mq2;
        sensor_pub.publish(&Sensor_msg);
        
        Point_msg.x=RnewPosition;
        Point_msg.y=LnewPosition;
        enc_pub.publish(&Point_msg);
        
//-------------        
      nh.spinOnce(); 
      //nh2.spinOnce(); 
      delay(10);
 }


void motorL_init(void)
{
  // init left motor
  pinMode(LM_LPWM, OUTPUT);
  pinMode(LM_RPWM, OUTPUT);

  analogWrite(LM_LPWM, 0);
  analogWrite(LM_RPWM, 0);
}

void motorR_init(void)
{
  // init right motor
  pinMode(RM_LPWM, OUTPUT);
  pinMode(RM_LPWM, OUTPUT);

  analogWrite(RM_LPWM, 0);
  analogWrite(RM_RPWM, 0);
}

void motorL_run(uint8_t dir, uint8_t spd)
{
    if (dir == FORWARD)
    {
      analogWrite(LM_RPWM, 0);
      analogWrite(LM_LPWM, spd);
    }
    else if (dir == BACKWARD)
    {
      analogWrite(LM_LPWM, 0);
      analogWrite(LM_RPWM, spd);
    }
}

void motorR_run(uint8_t dir, uint8_t spd)
{
    if (dir == FORWARD)
    {
      digitalWrite(RM_RPWM, 0);
      analogWrite(RM_LPWM, spd);
    }
    else if (dir == BACKWARD)
    {
      digitalWrite(RM_LPWM, 0);
      analogWrite(RM_RPWM, spd);
    }
}

/*void motorL_setSpeed(uint8_t speed)
{
  LM_speed = speed;
}

void motorR_setSpeed(uint8_t speed)
{
  RM_speed = speed;
}*/
