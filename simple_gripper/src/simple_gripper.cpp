#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// action interface type
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
    GripperClient *gripper_client_;

public:
    // action client intialization
    Gripper(){

        gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

        // wait for the action server to come up
        while(!gripper_client_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the gripper action server to come up");
        }
    }

    ~Gripper(){
        delete gripper_client_;
    }

    // tell the gripper to open
    void open(){

        pr2_controllers_msgs::Pr2GripperCommandGoal open;
        open.command.position = 0.07;
        open.command.max_effort = -50.0;  // do not limit effort (negative)

        ROS_INFO("Sending open goal");
        gripper_client_->sendGoal(open);
        gripper_client_->waitForResult(ros::Duration(10.0));
        if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The gripper opened!");
        else
            ROS_INFO("The gripper failed to open");
    }

    // tell the gripper to close
    void close(){

        pr2_controllers_msgs::Pr2GripperCommandGoal close;
        close.command.position = -.02;
        close.command.max_effort = 50.0; // close gently

        ROS_INFO("Sending close goal");
        gripper_client_->sendGoal(close);
        gripper_client_->waitForResult(ros::Duration(10.0));
        if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The gripper closed!");
        else
            ROS_INFO("The gripper failed to close");
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_gripper_node");

    Gripper gripper;

    gripper.open();
    gripper.close();

    return 0;
}
