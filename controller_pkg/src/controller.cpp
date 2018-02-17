#include "controller_pkg/controller.h"
#include <pluginlib/class_list_macros.h>

namespace my_controller_ns {

// controller initialization in non-realtime
bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n){
    std::string joint_name;
    if(!n.getParam("joint_name", joint_name)){
        ROS_ERROR("No joint given in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }

    joint_state_ = robot->getJointState(joint_name);
    if(!joint_state_){
        ROS_ERROR("MyController could not find joint named '%s'",joint_name.c_str());
        return false;
    }
    return true;
}

// controller startup in realtime
void MyControllerClass::starting(){
    init_pos_ = joint_state_->position_;
}


// controller update loop in realtime
void MyControllerClass::update(){
    double desired_pos = init_pos_ + 15 * sin(ros::Time::now().toSec());
    double current_pos = joint_state_->position_;
    joint_state_->commanded_effort_ = -10 * (current_pos - desired_pos);
}

// controller stopping in realtime
void MyControllerClass::stopping(){}
}

PLUGINLIB_DECLARE_CLASS(controller_pkg, MyControllerPlugin, my_controller_ns::MyControllerClass,pr2_controller_interface::Controller)


