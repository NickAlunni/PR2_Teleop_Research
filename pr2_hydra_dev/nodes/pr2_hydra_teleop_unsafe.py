#!/usr/bin/python

import roslib; roslib.load_manifest('arc_pr2_teleop')
import rospy
import math
from copy import deepcopy

from razer_hydra.msg import *
from geometry_msgs.msg import *
import actionlib
from pr2_controllers_msgs.msg import *

class HydraTeleop:
    def __init__(self, head_pointing_frame):
        rospy.init_node('pr2_hydra_teleop')
        self.head_pointing_frame = head_pointing_frame
        #Paddle definitions
        self.left = 0
        self.right = 1
        #Button definitions
        self.x_axis = 1 #number of up-down axis on right analog stick
        self.y_axis = 0 #number of left-right axis on right analog stick
#        self.left_close = 4 #numbered button on left paddle
#        self.left_open = 3 #numbered button on left paddle
#        self.right_close = 3 #numbered button on right paddle
#        self.right_open = 4 #numbered button on right paddle
        self.deadman_button = 5 #number of the left & right unmarked buttons
        self.gripper_step = 0.0001 #step size for gripper controller
        self.bumper = 0 #number of bumper button
        #Storage for head pan/tilt state
        self.head_pan = 0.0
        self.head_tilt = 0.0
        #Storage for the gripper setpoints
        self.left_gripper = 0.08
        self.right_gripper = 0.08
        #Storage for arm modes
        self.left_mode = -1
        self.right_mode = -1
        #Storage for hydra states
        self.left_hydra_start = None
        self.right_hydra_start = None
        #Storage for the pose data returned from the arm cartesian controllers
        self.current_l_arm_pose = None
        self.current_r_arm_pose = None
        #Set up the publishers
        self.base_pub = rospy.Publisher('base_controller/command', Twist)
        self.left_arm_pub = rospy.Publisher('l_arm_cart_controller/command', PoseStamped)
        self.right_arm_pub = rospy.Publisher('r_arm_cart_controller/command', PoseStamped)
#        self.left_gripper_pub = rospy.Publisher('l_gripper_controller/command', Pr2GripperCommand)
#        self.right_gripper_pub = rospy.Publisher('r_gripper_controller/command', Pr2GripperCommand)
        #Set up actionlib client for the head
        self.head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
        self.head_client.wait_for_server()
        #Subscribe to the arm controllers for pose data
        rospy.Subscriber("l_arm_cart_controller/state/pose", PoseStamped, self.l_arm_pose_cb)
        rospy.Subscriber("r_arm_cart_controller/state/pose", PoseStamped, self.r_arm_pose_cb)
        print "Ready for the Hydra..."
        #Move the arms to the default start pose
        self.set_arms()
        #Subscribe to the Hydra
        rospy.Subscriber("hydra_calib", Hydra, self.hydra_cb)
        print "Waiting for the Hydra..."
        #Spin
        rate = rospy.Rate(rospy.get_param('~hz', 60))
        while not rospy.is_shutdown():
            rate.sleep()

    def l_arm_pose_cb(self, pose_msg):
        #Callback for PoseStamped messages from the left arm's cartesian control
        self.current_l_arm_pose = pose_msg.pose

    def r_arm_pose_cb(self, pose_msg):
        #Callback for PoseStamped messages from the right arm's cartesian control
        self.current_r_arm_pose = pose_msg.pose

    def hydra_cb(self, hydra_msg):
        #Callback for hydra_calib messages from the Razer Hydra motion game controller
        #Command the base
        self.control_base(hydra_msg)
        #Command the head
        #self.control_head(hydra_msg)
        #Command the grippers
#        self.control_grippers(hydra_msg)
        #Command the arms
        self.control_arms(hydra_msg)

    def normalize(self, input_value, scale_factor, clamp):
        #Normalize the input value to the provided limit values
        raw_value = input_value * scale_factor * clamp
        if (raw_value == -0.0):
            raw_value = 0.0
        elif (raw_value > clamp):
            raw_value = clamp
        elif (raw_value < -clamp):
            raw_value = -clamp
        return raw_value

    def control_base(self, hydra_msg):
        base_command = Twist()
        #Get the raw values and normalize them
        base_command.linear.x = self.normalize(hydra_msg.paddles[self.right].joy[self.x_axis], 1, 0.2)
        base_command.linear.y = self.normalize(-hydra_msg.paddles[self.right].joy[self.y_axis], 1, 0.2)
        base_command.angular.z = self.normalize(-(-hydra_msg.paddles[self.left].trigger + hydra_msg.paddles[self.right].trigger), 1, 0.4)
        #Check the deadman's switch
        if (ord(hydra_msg.paddles[self.left].buttons[self.deadman_button]) == 1):
            self.base_pub.publish(base_command)
        else:
            base_command.linear.x = 0.0
            base_command.linear.y = 0.0
            base_command.angular.z = 0.0
            self.base_pub.publish(base_command)

#    def sanitize_gripper(self, close_btn, open_btn, current_state):
#        #Compute the correct new value for the gripper
#        if (close_btn == 1 and open_btn == 1):
#            #Catch the illegal state
#            return current_state
#        elif (close_btn == 1):
#            #Increment the gripper closer
#            new_gripper = current_state - self.gripper_step
#            if (new_gripper < 0.0):
#                new_gripper = 0.0
#            return new_gripper
#        elif (open_btn == 1):
#            #Increment the gripper opener
#            new_gripper = current_state + self.gripper_step
#            if (new_gripper > 0.08):
#                new_gripper = 0.08
#            return new_gripper
#        else:
#            return current_state

#    def control_grippers(self, hydra_msg):
#        #Update gripper settings
#        self.left_gripper = self.sanitize_gripper(ord(hydra_msg.paddles[self.left].buttons[self.left_close]), ord(hydra_msg.paddles[self.left].buttons[self.left_open]), self.left_gripper)
#        self.right_gripper = self.sanitize_gripper(ord(hydra_msg.paddles[self.right].buttons[self.right_close]), ord(hydra_msg.paddles[self.right].buttons[self.right_open]), self.right_gripper)
#        #Command grippers
#        l_gripper_goal = Pr2GripperCommand()
#        l_gripper_goal.position = self.left_gripper
#        l_gripper_goal.max_effort = 50.0
#        r_gripper_goal = Pr2GripperCommand()
#        r_gripper_goal.position = self.right_gripper
#        r_gripper_goal.max_effort = 50.0
#        #Send the commands
#        self.left_gripper_pub.publish(l_gripper_goal)
#        self.right_gripper_pub.publish(r_gripper_goal)

    def control_head(self, hydra_msg):
        #Setup the message components
        headGoal = PointHeadGoal()
        p = PointStamped()
        p.header.frame_id = "base_link"
        headGoal.pointing_frame = self.head_pointing_frame
        #Get raw values from the controller
        pan_axis = hydra_msg.paddles[self.left].joy[self.y_axis]
        tilt_axis = hydra_msg.paddles[self.left].joy[self.x_axis]
        #Check deadman's switch
        if (ord(hydra_msg.paddles[self.right].buttons[self.deadman_button]) == 1):
            self.head_pan = self.head_pan - pan_axis
            self.head_pan = self.normalize((self.head_pan / 120.0), 1.0, 120.0)
            self.head_tilt = self.head_tilt - tilt_axis
            self.head_tilt = self.normalize((self.head_tilt / 30.0), 1.0, 30.0)
        #Figure out actual values
        p.point.x = 1.2 + (5.0 * math.tan(self.head_tilt * (math.pi / 180.0)))
        p.point.y = 1.2 + (5.0 * math.tan(self.head_tilt * (math.pi / 180.0)))
        p.point.z = 1.2 + (5.0 * math.tan(self.head_tilt * (math.pi / 180.0)))
        #Assemble goal
        headGoal.target = p
        headGoal.min_duration = rospy.Duration(0.1)
        headGoal.max_velocity = 1.0
        #Send
        self.head_client.send_goal(headGoal)

    def control_arms(self, hydra_msg):
        #Set the mode for left arm
        if (ord(hydra_msg.paddles[self.left].buttons[self.bumper]) == 1):
            self.left_mode += 1
            if (self.left_mode == 0):
                print "Entering ACQUIRE mode [left]"
            elif (self.left_mode == 1):
                print "Entering TRACKING mode [left]"
            else:
                self.left_mode = 1
        else:
            if (self.left_mode != -1):
                print "Returning to IDLE mode [left]"
            self.left_mode = -1
        #Set the mode for right arm
        if (ord(hydra_msg.paddles[self.right].buttons[self.bumper]) == 1):
            self.right_mode += 1
            if (self.right_mode == 0):
                print "Entering ACQUIRE mode [right]"
            elif (self.right_mode == 1):
                print "Entering TRACKING mode [right]"
            else:
                self.right_mode = 1
        else:
            if (self.right_mode != -1):
                print "Returning to IDLE mode [right]"
            self.right_mode = -1
        #Command each arm correctly
        lerror = self.control_arm(hydra_msg.paddles[self.left], self.left_mode, 0)
        rerror = self.control_arm(hydra_msg.paddles[self.right], self.right_mode, 1)

    def set_arms(self):
        #Set the arms to a safe start point
        cmd = PoseStamped()
        cmd.header.frame_id = "/torso_lift_link"
        cmd.pose.position.x = 0.6
        cmd.pose.position.y = 0.2
        cmd.pose.position.z = -0.1
        cmd.pose.orientation.x = -0.00244781865415
        cmd.pose.orientation.y = -0.548220284495
        cmd.pose.orientation.z = 0.00145617884538
        cmd.pose.orientation.w = 0.836329126239
        self.left_arm_pub.publish(cmd)
        cmd.pose.position.y=-0.2
        self.right_arm_pub.publish(cmd)

    def control_arm(self, paddle, mode, arm_code):
        if (mode == -1):
            return
        elif (mode == 0):
            if (arm_code == 0):
                self.left_hydra_start = deepcopy(paddle.transform)
            elif (arm_code == 1):
                self.right_hydra_start = deepcopy(paddle.transform)
        elif (mode == 1):
            start_state = None
            arm_state = None
            if (arm_code == 0):
                start_state = self.left_hydra_start
                arm_state = self.current_l_arm_pose
            elif (arm_code == 1):
                start_state = self.right_hydra_start
                arm_state = self.current_r_arm_pose
            #Calculate position changes
            xdiff = (paddle.transform.translation.x - start_state.translation.x) / 5.0
            ydiff = (paddle.transform.translation.y - start_state.translation.y) / 5.0
            zdiff = (paddle.transform.translation.z - start_state.translation.z) / 5.0
            #Set the new arm command
            cmd = PoseStamped()
            cmd.header.frame_id = "/torso_lift_link"
            cmd.pose.position.x = xdiff + arm_state.position.x
            cmd.pose.position.y = ydiff + arm_state.position.y
            cmd.pose.position.z = zdiff + arm_state.position.z
            cmd.pose.orientation.x = paddle.transform.rotation.x
            cmd.pose.orientation.y = paddle.transform.rotation.y
            cmd.pose.orientation.z = paddle.transform.rotation.z
            cmd.pose.orientation.w = paddle.transform.rotation.w
            print cmd
            #Send the new arm command
            if (arm_code == 0):
                self.left_arm_pub.publish(cmd)
            elif (arm_code == 1):
                self.right_arm_pub.publish(cmd)
        else:
            print "*** Invalid operating mode! ***"

if __name__ == '__main__':
    #Get parameters
    #THIS IS SPECIFIC TO ARCHIE - NO OTHER PR2 HAS THIS TF FRAME!
    head_pointing_frame = "/head_mount_kinect_rgb_optical_frame"
    HydraTeleop(head_pointing_frame)
