#include <rclcpp/rclcpp.hpp>
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include "roboclaw_ros.hpp"


using namespace std::chrono_literals;

class MecanumDrive : public rclcpp::Node
{
    private:
    	size_t count_;
    	rclcpp::TimerBase::SharedPtr timer_;
    	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
        rclcpp::Time current_time;
        rclcpp::Time last_time;
        RoboclawNode back_left;
        RoboclawNode front_left;
        RoboclawNode back_right;
        RoboclawNode front_right;
		nav_msgs::msg::Odometry odomNew;
    	nav_msgs::msg::Odometry odomOld;
		double WHEEL_RADIUS, WHEELBASE_X, WHEELBASE_Y;
        int encoder_scale;
        double x, y, theta;
        double x_acc, y_acc, tht_acc;
        int br_p;
        int fr_p;
        int bl_p;
        int fl_p;
        double ticks_to_m;
        bool omnidirectional;
        bool debug;

    public:
        MecanumDrive(nav_msgs::msg::Odometry odomStart) 
		: Node("mecanum_drive"),
        count_(0),	
		timer_(this->create_wall_timer(500ms, std::bind(&MecanumDrive::updateOdom, this))),
		odom_publisher_(this->create_publisher<nav_msgs::msg::Odometry>("wheel/odometry", 90)),
		twist_subscriber_(this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 60, std::bind(&MecanumDrive::callbackTwist, this, std::placeholders::_1))),
		current_time(this->get_clock()->now()),
		last_time(this->get_clock()->now()),
        // Initialize Roboclaw Controllers
        back_left(RoboclawNode("/dev/ttyACM1", 129, 115200, false)),
        front_left(RoboclawNode("/dev/ttyACM1", 129, 115200, true)),
        back_right(RoboclawNode("/dev/ttyACM0", 128, 115200, true)),
        front_right(RoboclawNode("/dev/ttyACM0", 128, 115200, false)),
		odomNew(odomStart),
		odomOld(odomStart),
		WHEEL_RADIUS(rad),
        WHEELBASE_X(base_x),
        WHEELBASE_Y(base_y),
        // Set encoder counts to the accumulators for zeroing reasons
        br_p(back_right.getEncoder()),
        fr_p(front_right.getEncoder()), 
        bl_p(back_left.getEncoder()),
        fl_p(front_left.getEncoder())
        {
            // Make ROS Params available to set
			this->declare_parameter<int>("encoder_scale", -2560);
            this->declare_parameter<bool>("debug", false);
            this->declare_parameter<double>("wheel_radius", 0.09915f);
            this->declare_parameter<double>("wheelbase_x", 0.16312f);
            this->declare_parameter<double>("wheelbase_y", 0.3458f);
            this->declare_parameter<bool>("omnidirectional", false);
            ticks_to_m = encoder_scale*2.0*M_PI*WHEEL_RADIUS;
        };

        void callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
        void updateOdom();
        
};

void MecanumDrive::respond()
{
    // Set ROS params
    this->get_parameter("encoder_scale", -2560);
    this->get_parameter("wheel_radius", rad);
    this->get_parameter("wheelbase_x", WHEELBASE_X);
    this->get_parameter("wheelbase_y", WHEELBASE_Y);
    this->get_parameter("wheelbase_y", debug);
    this->get_parameter("omnidirectional", omnidirectional);
}

void MecanumDrive::callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg){
    // Convert msg to double
    x = (double)msg->linear.x;
    y = (double)msg->linear.y;
    theta = (double)msg->angular.z;
    
    // Convert x/y/tht speeds to wheel velocities
    float back_left_speed = encoder_scale*(1/(WHEEL_RADIUS*M_PI*2))*(x+y-(WHEELBASE_X + WHEELBASE_Y)*theta);
    float front_left_speed = encoder_scale*(1/(WHEEL_RADIUS*M_PI*2))*(x-y-(WHEELBASE_X + WHEELBASE_Y)*theta);
    float front_right_speed = encoder_scale*(1/(WHEEL_RADIUS*M_PI*2))*(x+y+(WHEELBASE_X + WHEELBASE_Y)*theta);
    float back_right_speed = encoder_scale*(1/(WHEEL_RADIUS*M_PI*2))*(x-y+(WHEELBASE_X + WHEELBASE_Y)*theta);
    
    // Send commands to wheels
    back_left.setSpeed(back_left_speed);
    front_left.setSpeed(front_left_speed);
    front_right.setSpeed(front_right_speed);
    back_right.setSpeed(back_right_speed);
}

void MecanumDrive::updateOdom()
{
    if(omnidirectional)
    {
        odomOmni();
    }
    else
    {
        odomDifferential();
    }
}


void MecanumDrive::odomDifferential()
{
    double m_x, m_y, m_tht;
    // Get change in wheel rotations
    int bl = back_left.getEncoder() - bl_p;
    int br = back_right.getEncoder() - br_p;
    int fl = front_left.getEncoder() - fl_p;
    int fr = front_right.getEncoder() - fr_p;
    // Update accumulators
    bl_p += bl;
    br_p += br;
    fl_p += fl;
    fr_p += fr;
    
    current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds(); 

    m_x = (fr+br+fl+bl)/(4.0*ticks_to_m);
    m_y = 0;
    m_tht = double((fr + br - fl - br))/ticks_to_m/WHEELBASE_Y;
    
    double accAngle = m_tht/1.5;

    // Print pos data
    if(debug)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"x_pos="+std::to_string(x_acc)+" y_pos="+std::to_string(y_acc)+" tht="+std::to_string(tht_acc)+"\n");
    }

    // Build odom msg
    odomNew.header.stamp = current_time;
    x_acc += cos(tht_acc)*m_x + sin(tht_acc)*m_y;
    y_acc += cos(tht_acc)*m_y + sin(tht_acc)*m_x;
    tht_acc += accAngle;
    tf2::Quaternion q;
    // Skip update if data is bad
    if (!(std::isnan(x_acc) || std::isnan(y_acc) || std::isnan(tht_acc))) {
        odomNew.pose.pose.position.x = x_acc;
        odomNew.pose.pose.position.y = y_acc;
        q.setRPY(0,0, tht_acc);
        odomNew.pose.pose.orientation.x = q.x();
        odomNew.pose.pose.orientation.y = q.y();
        odomNew.pose.pose.orientation.z = q.z();
        odomNew.pose.pose.orientation.w = q.w();
    }
    
    //Update Velocities
    odomNew.twist.twist.linear.x = m_x/dt;
    odomNew.twist.twist.linear.y = 0; //TODO Update with mecanum math
    odomNew.twist.twist.angular.z = accAngle/dt;
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation = odomNew.pose.pose.orientation;
    odomOld.header.stamp = odomNew.header.stamp;
    odom_publisher_->publish(odomNew);
    
    last_time = current_time;
}

void MecanumDrive::odomOmni()
{
    double m_x, m_y, m_tht;
    // Get change in wheel rotations
    int bl = back_left.getEncoder() - bl_p;
    int br = back_right.getEncoder() - br_p;
    int fl = front_left.getEncoder() - fl_p;
    int fr = front_right.getEncoder() - fr_p;
    // Update accumulators
    bl_p += bl;
    br_p += br;
    fl_p += fl;
    fr_p += fr;
    
    current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds(); 

    m_x = (fr+br+fl+bl)/(4.0*ticks_to_m);
    // TODO update below with mecanum math
    m_y = 0;
    m_tht = double((fr + br - fl - br))/ticks_to_m/WHEELBASE_Y;
    
    double accAngle = m_tht/1.5;

    // Print pos data
    if(debug)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"x_pos="+std::to_string(x_acc)+" y_pos="+std::to_string(y_acc)+" tht="+std::to_string(tht_acc)+"\n");
    }

    // Build odom msg
    odomNew.header.stamp = current_time;
    x_acc += cos(tht_acc)*m_x + sin(tht_acc)*m_y;
    y_acc += cos(tht_acc)*m_y + sin(tht_acc)*m_x;
    tht_acc += accAngle;
    tf2::Quaternion q;
    // Skip update if data is bad
    if (!(std::isnan(x_acc) || std::isnan(y_acc) || std::isnan(tht_acc))) {
        odomNew.pose.pose.position.x = x_acc;
        odomNew.pose.pose.position.y = y_acc;
        q.setRPY(0,0, tht_acc);
        odomNew.pose.pose.orientation.x = q.x();
        odomNew.pose.pose.orientation.y = q.y();
        odomNew.pose.pose.orientation.z = q.z();
        odomNew.pose.pose.orientation.w = q.w();
    }
    
    //Update Velocities
    odomNew.twist.twist.linear.x = m_x/dt;
    odomNew.twist.twist.linear.y = 0; //TODO Update with mecanum math
    odomNew.twist.twist.angular.z = accAngle/dt;
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation = odomNew.pose.pose.orientation;
    odomOld.header.stamp = odomNew.header.stamp;
    odom_publisher_->publish(odomNew);
    
    last_time = current_time;
}



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
	nav_msgs::msg::Odometry odomStart;
	odomStart.header.frame_id = "odom";
	odomStart.child_frame_id = "base_link";
	odomStart.pose.pose.position.z = 0;
	odomStart.pose.pose.orientation.z = 0;
    odomStart.pose.pose.orientation.x = 0;
    odomStart.pose.pose.orientation.y = 0;
    odomStart.pose.pose.orientation.w = 0;
    odomStart.twist.twist.linear.x = 0;
    odomStart.twist.twist.linear.y = 0;
    odomStart.twist.twist.linear.z = 0;
    odomStart.twist.twist.angular.x = 0;
    odomStart.twist.twist.angular.y = 0;
    odomStart.twist.twist.angular.z = 0;
    odomStart.pose.pose.position.x = 0;
    odomStart.pose.pose.position.y = 0;
    
    rclcpp::spin(std::make_shared<MecanumDrive>(odomStart));
    rclcpp::shutdown();
    return 1;
}
