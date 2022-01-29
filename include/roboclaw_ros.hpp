#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_msgs/msg/int32.hpp>
#include "roboclaw.h"



#ifndef RoboclawNode_h
#define RoboclawNode_h

class RoboclawNode
{
    private:
        struct roboclaw *rc;
        std::string device_name;
        uint8_t address=0x80;
        int baudrate;
        bool m1;
    public:

        explicit RoboclawNode(std::string devname, int addr, int baudrate, bool m1);

        void setSpeed(int speed);

        int getEncoder();
        
        
};
#endif
