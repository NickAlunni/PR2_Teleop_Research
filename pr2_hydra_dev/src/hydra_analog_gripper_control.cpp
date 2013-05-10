#include "ros/ros.h"
#include "pr2_controllers_msgs/Pr2GripperCommand.h"
#include "razer_hydra/Hydra.h"
#include "razer_hydra/HydraPaddle.h"

ros::Publisher r_gripper_command;
ros::Publisher l_gripper_command;


/** Converts the raw hydra messages into commands for the PR2's grippers
 *
 * This function will convert the signal coming from the Razer Hydra into commands for the grippers.
 * The grippers will be controlled by the analog stick on the Razer Hydra and will be fully open when
 * the tripper is not depressed and fully closed when the trigger is fully depressed.  The scaling
 * function will be Gripper = .8 when the trigger = 0, and Gripper = 0 when the trigger = 255;
 */
void hydra_cb(razer_hydra::Hydra input){

    //Commands for the left and right grippers
    pr2_controllers_msgs::Pr2GripperCommand r_command, l_command;

    //Calculate how open the grippers should be
    l_command.position = .07 - (.07 * input.paddles[0].trigger);
    r_command.position = .07 - (.07 * input.paddles[1].trigger);

    //Define the maximum efford that they are allowed to use
    l_command.max_effort = 50;
    r_command.max_effort = 50;

    //Publish the commands
    l_gripper_command.publish(l_command);
    r_gripper_command.publish(r_command);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "hydra_analog_gripper_control");

  //Setup everything for the publishes
  ros::NodeHandle nh;

  //Advertise the two publishers, one for the commands and one for the gui
  l_gripper_command = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("l_gripper_controller/command", 1);
  r_gripper_command = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("r_gripper_controller/command", 1);

  //Create all the subscribers that are needed
  ros::Subscriber sub_hydra = nh.subscribe("hydra_calib",1, hydra_cb);

  //Spin Forever
  ros::spin();

  return 0;

}

