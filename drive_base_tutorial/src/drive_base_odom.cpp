#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <stdlib.h>

class RobotDriver{
private:
    // node handle
    ros::NodeHandle nh_;
    // publishing "cmd_vel" topic
    ros::Publisher cmd_vel_pub_;
    // listening to TF transforms
    tf::TransformListener listener_;

public:
    // node initialization
    RobotDriver(ros::NodeHandle &nh){
        nh_ = nh;
        //set up the publisher
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    // drive forward based on odometry information
    bool driveForwardOdom(double distance){
        // wait for the listener to get the first message
        listener_.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(1.0));

        // we will record transforms here
        tf::StampedTransform start_transform;
        tf::StampedTransform current_transform;

        // record the starting transform from the odometry to the base frame
        listener_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), start_transform);

        // sending command of type "twist"
        geometry_msgs::Twist base_cmd;
        // command to go forward 0.25 m/s
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = 0.25;

        ros::Rate rate(10.0);
        bool done = false;
        while(!done && nh_.ok()){
            //send the drive command
            cmd_vel_pub_.publish(base_cmd);
            rate.sleep();
            // get the current transform
            try{
                listener_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), current_transform);
            }
            catch(tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
                break;
            }

            // see how far traveled
            tf::Transform relative_transform = start_transform.inverse() * current_transform;
            double dist_moved = relative_transform.getOrigin().length();

            if(dist_moved > distance) done = true;
        }
        if(done) return true;
        return false;
    }
};

int main(int argc, char** argv){
    // init the ROS node
    ros::init(argc, argv, "robot_driver_odom");
    ros::NodeHandle nh;

    RobotDriver driver(nh);
    double distance = strtol(argv[1], NULL, 10);
    driver.driveForwardOdom(distance);
}
