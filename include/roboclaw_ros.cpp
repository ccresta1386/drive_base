#include "roboclaw_ros.hpp"



using namespace std;

RoboclawNode::RoboclawNode(string devname, int addr, int baudrate, bool m1)
{
    this->baudrate=baudrate;
    this->address = addr;
    this->m1 = m1;
    char* dev_addr;
    dev_addr = &devname[0];
    this->rc = roboclaw_init(dev_addr, baudrate);
    
    if( this->rc == NULL )
    {
        perror("Unable to initialize Roboclaw");
        
    }
    if(roboclaw_speed_m1m2(this->rc, this->address, 0,0) != ROBOCLAW_OK){
		string msg = "Roboclaw initialization Error, check roboclaw address: " + devname + " " + to_string(addr);
        perror(msg.c_str());
     }
    

}

void RoboclawNode::setSpeed(int speed)
{
    int resp;
    if(this->m1)
    {
        
        resp = roboclaw_speed_m1(this->rc, this->address, speed);
    }
    else
    {
        resp = roboclaw_speed_m2(this->rc, this->address, speed);
    }
    switch (resp)
    {
    case -1:
        perror("Roboclaw Error on speed command");
        break;
    case 0:
        break;//RCLCPP_INFO_STREAM(node->get_logger(),"Roboclaw Okay");
    case -2:
        perror("Roboclaw Retries Exceeded");
    default:
        break;
    }
}

int RoboclawNode::getEncoder()
{
    std_msgs::msg::Int32 msg;
    int32_t enc1, enc2;
    
    if(roboclaw_encoders(this->rc, this->address, &enc1, &enc2) != ROBOCLAW_OK){
        perror("Error: Communication with Roboclaw failed on encoder read");
        return 0;
    }
        
    if(this->m1){
        return (int)enc1;
    }
    else{
        return (int)enc2;
    }
}



// int main(int argc, char **argv){
//     ros::init(argc, argv, "roboclaw_node");
//     struct roboclaw *rc;
//     ros::NodeHandle nh;
// 	int address = 129; //address of roboclaw unit
// 	string dev = "/dev/ttyACM0";
// 	int baudrate=115200;
// 	RoboclawNode rcn = RoboclawNode(dev, address, baudrate, &nh, true);
// 	ROS_INFO_STREAM("BEFORE SPIN");
// 	ros::spin();
    
// }


