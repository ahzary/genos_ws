#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <aubo_control/aubo_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_hw_interface");
  ros::NodeHandle nh; 


  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  std::shared_ptr<genos_control_ns::genosHWInterface> genos_hw_interface_instance 
   (new genos_control_ns::genosHWInterface(nh));
  genos_hw_interface_instance->init(); // size and register required interfaces inside generic_hw_interface.cpp


  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, genos_hw_interface_instance);
  control_loop.run(); // Blocks until shutdown signal recieved -> read -> update -> write -> repeat inside generic_hw_control_loop.cpp

  return 0;
}
