#include "ros/ros.h"
#include "robot_voice/Iat.h"
#include "robot_voice/Voice.h"
#include "thread"
#include "mutex"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
std::mutex mtx;

class demo{
public:
    demo();
    void task(void);
    bool flag_,flag1;
    std::string str_;

private:
    ros::ServiceClient iat_,tts_;
    ros::Publisher vel_pub;

    // std::thread thread1(task);

};
