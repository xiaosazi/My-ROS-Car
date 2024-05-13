#include "demo.h"

demo::demo()
{
    ros::NodeHandle n;
    iat_ = n.serviceClient<robot_voice::Iat>("Iat");
    tts_ = n.serviceClient<robot_voice::Voice>("Voice");
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::service::waitForService("Iat");
    ros::service::waitForService("Voice");
    ros::Rate r(1);
    std::thread thread1(&demo::task,this);
    thread1.detach();
    while (ros::ok())
    {
        if(flag_)
        {
            if(str_.find("同学") != std::string::npos)
            {
                robot_voice::Voice tts;
                tts.request.name = "我在，什么事呢？";
                tts_.call(tts);
            }
            else if(str_.find("前") != std::string::npos)
            {
                robot_voice::Voice tts;
                tts.request.name = "正在前进";
                tts_.call(tts);

                geometry_msgs::Twist vel_;
                vel_.linear.x = 0.2;
                vel_pub.publish(vel_);
                r.sleep();
                r.sleep();
                r.sleep();
                vel_.linear.x = 0;
                vel_pub.publish(vel_);
            }
            else if(str_.find("后") != std::string::npos)
            {
                robot_voice::Voice tts;
                tts.request.name = "正在后退";
                tts_.call(tts);

                geometry_msgs::Twist vel_;
                vel_.linear.x = -0.2;
                vel_pub.publish(vel_);
                r.sleep();
                r.sleep();
                r.sleep();
                vel_.linear.x = 0;
                vel_pub.publish(vel_);
            }
            flag_ = false;
        }
    }
    ros::spinOnce();
    

}
void demo::task(void)
{
    while (true)
    {
        mtx.lock();
        char ch;
        std::cout << "请按下Enter:" <<std::endl;
        if (std::cin.get(ch) && ch == '\n')
        {
            robot_voice::Iat iat;
            iat.request.num =  1;
            flag_ = iat_.call(iat);
            str_ = iat.response.string.c_str();
            // if(flag_)
            // {
            //     str_ = iat.response.string.c_str();
            //     ROS_INFO_STREAM(str_);
            // }
            // str_ = iat.response.string.c_str();
            // ROS_INFO_STREAM(iat.response.string.c_str());

        }
        mtx.unlock();
    }
    
    
}
int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"demo_node");
    demo demo;
    return 0;
}