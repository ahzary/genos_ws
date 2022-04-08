#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <aubo_control/emgmsg.h>

// ros vars
ros::NodeHandle  nh;
aubo_control::emgmsg emg_sig; // array of float for each dof
ros::Publisher pub("arduino/emg", &emg_sig );

// emg input pins
#define elbow A5
#define aduc A4
#define flex A3
// code vars
float e,a,f;


void setup() {
  // put your setup code here, to run once:
nh.getHardware()->setBaud(115200);// make 115200
nh.initNode();
nh.advertise(pub);

// ros msg setup
//emg_sig.data_length = 3;
}

void loop() {
// Read emgsigs
e = analogRead(elbow) * (2*3.1415)/1023;
a = analogRead(aduc) * (2*3.1415)/1023;
f = analogRead(flex) * (2*3.1415)/1023;
// fill array
emg_sig.data[0] = e;
emg_sig.data[1] = 0;//a;
emg_sig.data[2] = 0;//f;
//delay(100);
pub.publish(&emg_sig);
nh.spinOnce();
delay(200);

} 
