#include <ros/ros.h>
#include <aubo_control/auboTelemetry.h>
#include <aubo_control/armCmd.h> 


//// current position callback ////
// for subscribing to telemetry
ros::Publisher telem_pub;

void cmdCallback(const aubo_control::armCmd::ConstPtr &msg){
    static aubo_control::auboTelemetry telem;
    for(int i=0; i < msg->angle.size(); i++){
        telem.angle[i]=msg->angle[i];
    }

    telem_pub.publish(telem);
}


aubo_control::armCmd current_cmd;
aubo_control::auboTelemetry telem;

// create a timer callback they fires at 181 hz . if points are recieved then add them to a buffer. read from this buffer from the timer. 

//// main ////
int main(int argc, char **argv) {
    ros::init(argc, argv, "aubo_sim_fake_echo");

    // prep for ROS communcation
    ros::NodeHandle n; 

    ros::Subscriber cmd_sub = n.subscribe("/arduino/armCmd", 10, cmdCallback); // robot feedback

    telem_pub = n.advertise<aubo_control::auboTelemetry>("/arduino/auboTelemetry", 10);

	


 
    ros::spin();
 

} // end main