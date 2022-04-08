#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <aubo_control/auboTelemetry.h>

const int num_joints=3;
ros::NodeHandle  nh;
aubo_control::auboTelemetry cmd_msgs;
ros::Publisher pub("arduino/auboTelemetry", &cmd_msgs );

/*
struct armCmd { 
    std_msgs::Float32 current[num_joints];//amps
    std_msgs::Float32 accel[num_joints]; //deg/s^2
    std_msgs::Float32 vel[num_joints]; //deg/s
    std_msgs::Float32 angle[num_joints]; //deg
    //uint32 msg_ctr // count sent msgs to detected missed messages
    
  
  };*/
//struct armCmd cmd_msgs;

/*          
            0---->360
            0---->1023
            0---->




*/



#define pb 8
float i=0.00;
char* joint_name[]={"joint_2_rev", "joint_3_rev", "joint_adcu_1"};
float joint_postion[3];
char* id="/genos";
float js[3]={0,1.2,0.5};

void setup() {
  // put your setup code here, to run once:
pinMode(pb,OUTPUT);
nh.getHardware()->setBaud(115200);

nh.initNode();
nh.advertise(pub);

float i = 0;


//  cmd_msgs.joint_names=joint_name;   //for old aubo control
  cmd_msgs.angle[0]=joint_postion[0];
  cmd_msgs.angle[1]=joint_postion[1];
  cmd_msgs.angle[2]=joint_postion[2];

}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  js[0]=i;
 float x=analogRead(A2);
 //joint_postion[0]=x;
 float y=map(analogRead(A4),0,1023,0,360);
 float h=y*0.01745329251;
 //joint_postion[1]=y;
  joint_postion[2]=h;
 
  cmd_msgs.angle[0]=joint_postion[0];
  cmd_msgs.angle[1]=joint_postion[1];
  cmd_msgs.angle[2]=joint_postion[2];
  
  pub.publish(&cmd_msgs);
  nh.spinOnce();
}
