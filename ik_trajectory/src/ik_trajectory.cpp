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
private:
    ros::NodeHandle node;
    ros::ServiceClient ik_client;
    ros::ServiceServer service;
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    kinematics_msgs::GetPositionIK::Request ik_request;
    kinematics_msgs::GetPositionIK::Response ik_response;
    TrajClient *action_client;

public:
    IKTrajectoryExecutor(){

        // create a client function for the IK service
        ik_client = node.serviceClient<kinematics_msgs::GetPositionIK>(ARM_IK_NAME, true);

        // wait for the various services to be ready
        ROS_INFO("waiting for services to be ready");
        ros::service::waitForService(ARM_IK_NAME);
        ROS_INFO("Services ready");

        // tell the joint trajectory action client that we want
        // to spin a thread by default
        action_client = new TrajClient("r_arm_controller/joint_trajectory_action", true);

        // wait for the action server to come up
        while(ros::ok() && !action_client->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the joint_trajectory_action action server to come up");
        }
    }
};

int main(int argc, char** argv){

    // init ros node
    ros::init(argc, argv, "cartesian_ik_trajectory_executor");

    IKTrajectoryExecutor ik_traj_exec = IKTrajectoryExecutor();
    ROS_INFO("Waiting for cartesian trajectories to execute");
    ros::spin();

    return 0;
}
