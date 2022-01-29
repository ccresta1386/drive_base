#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include "roboclaw_ros.hpp"

#ifndef MecanumDrive_h
#define MecanumDrive_h

class MecanumDrive
{
    private:
        ros::Publisher odom_pub;
        static tf::TransformBroadcaster odom_broadcaster;
        ros::Timer current_speed_timer_;
        ros::Time current_time;
        ros::Time last_time = ros::Time::now();
        double publish_current_speed_frequency_;
        int encoder_scale = -150;
        ros::Subscriber twist_subscriber_;
        double x, y, theta;
        float WHEEL_RADIUS, WHEELBASE_X, WHEELBASE_Y;
        RoboclawNode back_left;
        RoboclawNode front_left;
        RoboclawNode back_right;
        RoboclawNode front_right;

    public:
        MecanumDrive(ros::NodeHandle *nh, float rad, float base_x, float base_y) : WHEEL_RADIUS(rad), WHEELBASE_X(base_x), WHEELBASE_Y(base_y), back_left(RoboclawNode("/dev/ttyACM0", 129, 115200, nh, false)), front_left(RoboclawNode("/dev/ttyACM0", 129, 115200, nh, true)), back_right(RoboclawNode("/dev/ttyACM1", 128, 115200, nh, true)), front_right(RoboclawNode("/dev/ttyACM1", 128, 115200, nh, false)), twist_subscriber_(nh->subscribe("/cmd_vel", 10.0, &MecanumDrive::callbackTwist, this)), odom_pub(nh->advertise<nav_msgs::Odometry>("odom", 50)){};

        void callbackTwist(geometry_msgs::Twist msg);
        void updateOdom(const ros::TimerEvent &event);
        
};

#endif
