#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <pr2/ik_trajectory/ExecuteCartesianIKTrajectory.h>
#include <vector>

#define MAX_JOINT_VEL 0.5 // in radians/sec

static const std::string ARM_IK_NAME = "/pr2_right_arm_kinematics/get_ik";
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class IKTrajectoryExecutor{

};

int main(int argc, char** argv){

    // init ros node
    ros::init(argc, argv, "cartesian_ik_trajectory_executor");

    IKTrajectoryExecutor ik_traj_exec = IKTrajectoryExecutor();
    ROS_INFO("Waiting for cartesian trajectories to execute");
    ros::spin();

    return 0;
}
