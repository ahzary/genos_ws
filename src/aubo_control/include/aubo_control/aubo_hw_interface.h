
#ifndef AUBO_HW_INTERFACE_H
#define AUBO_HW_INTERFACE_H
 
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <sensor_msgs/JointState.h>
#include <aubo_control/auboTelemetry.h>
#include <aubo_control/armCmd.h>

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131
#define DESIRED_BUFFERED_POINTS 12

namespace genos_control_ns
{

/// \brief Hardware interface for a robot
class genosHWInterface : public ros_control_boilerplate::GenericHWInterface
{


public:
  // brief Constructor \param nh - Node handle for topics. 
  genosHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL); // when this is Null it looks for urdf at robot_description parameter server

  // brief Initialize the robot hardware interface 
  virtual void init();

  // brief Read the state from the robot hardware. REQUIRED or wont compile
  virtual void read(ros::Duration &elapsed_time);

  // brief Write the command to the robot hardware.  REQUIRED or wont compile  
  virtual void write(ros::Duration &elapsed_time);

  //  REQUIRED or wont compile
  virtual void enforceLimits(ros::Duration &period);

protected:

  ros::Subscriber telemetry_sub;
  void telemetryCallback(const aubo_control::auboTelemetry::ConstPtr &msg);

  ros::Publisher cmd_pub;
  std::vector<double> joint_position_prev_;
  int bufferHealth=0;

};  // class

}  // namespace
 
#endif
