#include <ros/ros.h>
#include <string>
#include roboclaw.h

int main(int argc, char **argv){
    ros:: init(argc, argv, "motor_driver");
    ros::NodeHandle nh;
    struct roboclaw *rc;
	uint8_t address=0x80; //address of roboclaw unit
	int16_t voltage;
	float voltage_float;
	int baudrate, duty_cycle;
    
}

class RoboclawROSWrapper
{
private:
    std::unique_ptr<struct roboclaw> rc;
    string dev;
    string address;
    int baudrate;
    ros::Publisher current_speed_publisher_;
    ros::Timer current_speed_timer_;
    double publish_current_speed_frequency_;
public:
    RoboclawROSWrapper(string dev, string address, int baudrate, ros::NodeHandle *nh, bool m1)
    {
        baudrate=baudrate;
        address = (uint8_t)strtol(address, NULL, 0);
        rc=roboclaw_init(dev, baudrate);

        if( rc == NULL )
        {
            perror("Unable to initialize Roboclaw");
            return -1;
        }
        speed_command_subscriber_ = nh->subscribe("speed_command", 10, &RoboclawROSWrapper::callbackSpeedCommand, this);
        stop_motor_server_ = nh->advertiseService("stop_motor", &RoboclawROSWrapper::callbackStop, this);
        if (!ros::param::get("~publish_current_speed_frequency", publish_current_speed_frequency_))
        {
            publish_current_speed_frequency_ = 5.0;
        }
        current_speed_publisher_ = nh->advertise<std_msgs::Int32>("current_speed", 10);
        current_speed_timer_ = nh->createTimer(
            ros::Duration(1.0 / publish_current_speed_frequency_),
            &MotorDriverROSWrapper::publishCurrentSpeed, this);

    }
    void callbackStop()
    {
        roboclaw_duty_m1m2(rc, address, 0, 0)
    }
    void callbackSpeedCommand(const std_msgs::Int32 &msg)
    {
        if(m1)
        {
            roboclaw_duty_m1(rc, address, msg.data)
        }
        else
        {
            roboclaw_duty_m2(rc, address, msg.data)
        }
    }
    void publishCurrentSpeed(const ros::TimerEvent &event)
    {
        std_msgs::Int32 msg;
        int enc1, enc2;
        comm_status = roboclaw_encoders(rc, address, enc1, enc2);
        if(!comm_status){
            cout << "Error communicating with Roboclaw" << endl;
            return;
        }
        if(m1){
            msg.data = enc1;
        }
        else{
            msg.data = enc2;
        }
        current_speed_publisher_.publish(msg);
    }
    
    
}

