#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient <pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class RobotArm{
private:
    // action client for the joint trajectory action
    // used to trigger the arm movement action
    TrajClient* traj_client_;

public:
    // initialize the action client and wait for action server to come up
    RobotArm(){
        // tell the action client that we want to spin a thread by default
        traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

        // wait for action server to come up
        while(!traj_client_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }

    // clean up the action client
    ~RobotArm(){
        delete traj_client_;
    }

    // sends the command to start a given trajectory
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal){
        // when to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        traj_client_->sendGoal(goal);
    }


};

int main(int argc, char** argv){
    // init the ros node
    ros::init(argc, argv, "simple_trajectory");

    RobotArm arm;

    //start the trajectory
    arm.startTrajectory(arm.arm)
}
