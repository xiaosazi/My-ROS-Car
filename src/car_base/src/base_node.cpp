#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
//Macro definition
//宏定义
#define FRAME_HEADER      0XFC       //Frame head //帧头
#define FRAME_TAIL        0XDF             //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 12              //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define SEND_DATA_SIZE 12
#define PI 				  3.1415926f //PI //圆周率



class CarBase{
    public:
        CarBase()
        {
            ros::NodeHandle n;
            ros::NodeHandle private_n("~");
            private_n.param<std::string>("odom_frame",odom_frame_,"odom"); //从参数服务器获取参数odom_frame
            private_n.param<std::string>("baselink_frame",baselink_frame_,"base_link"); //从参数服务器获取参数odom_frame
            private_n.param<std::string>("port",port_,"dev/ttyUSB0");                      //从参数服务器获取串口端口
            private_n.param<int>("baudrate",baudrate_,115200);                                //从参数服务器获取波特率
            ser_init(); //初始化串口
            Robot_Pos.Z = 0.0;
            cmd_vel_ = n.subscribe("cmd_vel",100,&CarBase::CmdvelCallback, this); // 速度订阅者
            voltage_pub = n.advertise<std_msgs::Float32>("voltage",10);                         // 电压发布者
            imu_pub = n.advertise<sensor_msgs::Imu>("imu",10);                                       // Imu发布者
            static tf2_ros::StaticTransformBroadcaster  odom_broadcaster_;
            odom_pub = n.advertise<nav_msgs::Odometry>("odom",10) ;                       // 里程计发布者
            Txdata_timer_ = n.createTimer(ros::Duration(1.0/10000000), &CarBase::TxCallback, this); //创建一个timer，用于发布消息

            ros::spin();   
        }
        ~CarBase()
        {
            //Sends the stop motion command to the lower machine before the turn_on_robot object ends
            //对象turn_on_robot结束前向下位机发送停止运动命令
            uint8_t data[12] = {0xfc, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfa, 0xdf};
            try
            {
                ser.write(data,sizeof (data)); //Send data to the serial port //向串口发数据  
            }
            catch (serial::IOException& e)   
            {
                ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
            }
            ser.close(); //Close the serial port //关闭串口  
            ROS_INFO_STREAM("Shutting down"); //Prompt message //提示信息
        }
        private:
            //The structure of the ROS to send data to the down machine
            //ROS向下位机发送数据的结构体
            typedef struct _SEND_DATA_  
            {
                    uint8_t tx[SEND_DATA_SIZE];
                    float X_speed;	       
                    float Y_speed;           
                    float Z_speed;         
                    unsigned char Frame_Tail; 
            }SEND_DATA;

            //The structure in which the lower computer sends data to the ROS
            //下位机向ROS发送数据的结构体
            typedef struct _RECEIVE_DATA_     
            {
                    uint8_t rx[RECEIVE_DATA_SIZE];
                    uint8_t Flag_Stop;
                    unsigned char Frame_Header;
                    float X_speed;  
                    float Y_speed;  
                    float Z_speed; 
                    float A_speed;  
                    float B_speed;  
                    float C_speed; 
                    float D_speed; 
                    float Power_Voltage;	
                    unsigned char Frame_Tail;
            }RECEIVE_DATA;

            typedef struct __Vel_Pos_Data_
            {
                float X;
                float Y;
                float Z;
                int32_t A_encode;
                int32_t B_encode;
                int32_t C_encode;
                int32_t D_encode;
                int32_t A_encode_d;
                int32_t B_encode_d;
                int32_t C_encode_d;
                int32_t D_encode_d;
            }Vel_Pos_Data;

            // enum Receive_enum{
            //     VALTAGE = 0x01;

            // };
            SEND_DATA Send_Data;               //The serial port sends the data structure //串口发送数据结构体
            RECEIVE_DATA Receive_Data;   //The serial port receives the data structure //串口接收数据结构体
            Vel_Pos_Data Robot_Pos;
            serial::Serial ser;
            std::string odom_frame_,baselink_frame_,port_;
            int baudrate_;
            bool imu_flag[3];
            ros::Subscriber cmd_vel_;
            ros::Publisher imu_pub,odom_pub,voltage_pub;
            ros::Timer Txdata_timer_; 
            ros::Time now_, last_time_, last_twist_time_;
            
            float odom_x_scale, odom_y_scale, odom_z_scale_positive, odom_z_scale_negative,count_time_;
            bool start_flag_ = true;
            sensor_msgs::Imu imu_msg;

            //Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
            //协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
            const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
                                                        0, 1e-3,    0,   0,   0,    0,
                                                        0,    0,  1e6,   0,   0,    0,
                                                        0,    0,    0, 1e6,   0,    0,
                                                        0,    0,    0,   0, 1e6,    0,
                                                        0,    0,    0,   0,   0,  1e3 };

            const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
                                                        0, 1e-3, 1e-9,   0,   0,    0,
                                                        0,    0,  1e6,   0,   0,    0,
                                                        0,    0,    0, 1e6,   0,    0,
                                                        0,    0,    0,   0, 1e6,    0,
                                                        0,    0,    0,   0,   0, 1e-9 };

            const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
                                                        0, 1e-3,    0,   0,   0,    0,
                                                        0,    0,  1e6,   0,   0,    0,
                                                        0,    0,    0, 1e6,   0,    0,
                                                        0,    0,    0,   0, 1e6,    0,
                                                        0,    0,    0,   0,   0,  1e3 };
                                                        
            const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
                                                        0, 1e-3, 1e-9,   0,   0,    0,
                                                        0,    0,  1e6,   0,   0,    0,
                                                        0,    0,    0, 1e6,   0,    0,
                                                        0,    0,    0,   0, 1e6,    0,
                                                        0,    0,    0,   0,   0, 1e-9} ;

            /****************************
            Data：2024 04 09
            功能：异或校验
            参数：Count_Number：前Count_Number个字节异或运算
            ****************************/
            int Check_Sum(unsigned char Count_Number)
            {
                int check_sum=0,k;
                for(k=0;k<Count_Number;k++)
                {
                    check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
                }
                return check_sum;
            }
            
            /************************************************
             * Data: 2024 04 09
             *功能： 速度话题回调函数
             * 输入参数：cmd_vel:订阅速度话题获取的消息数据
            ************************************************/
            void CmdvelCallback(const geometry_msgs::Twist &cmd_vel)
            {
                short  transition;  //intermediate variable //中间变量

                // TxData = {0xfc,0x06,0x00,0X00,0x0f,0xa0,0x00,0x00,0x00,0x00,0x55,0xdf};
                Send_Data.tx[0] = FRAME_HEADER;     //frame head 0x7B //帧头0XFC
                Send_Data.tx[1] = 0x06;                             //功能位 0x06

                Send_Data.tx[2] = 0;                                    //保留位
                Send_Data.tx[3] = 0;                                    //保留位

                //The target velocity of the X-axis of the robot
                //机器人x轴的目标线速度
                transition = 0;
                transition = cmd_vel.linear.x*1000; //将浮点数放大一千倍，简化传输
                Send_Data.tx[5] = transition & 0xff;     //取数据的低8位
                Send_Data.tx[4] = transition>>8;  //取数据的高8位

                //The target velocity of the Y-axis of the robot
                //机器人y轴的目标线速度
                transition = 0;
                transition = cmd_vel.linear.y*1000;
                Send_Data.tx[7] = transition & 0xff;
                Send_Data.tx[6] = transition>>8;

                //The target angular velocity of the robot's Z axis
                //机器人z轴的目标角速度
                transition = 0;
                transition = cmd_vel.angular.z*1000;
                Send_Data.tx[9] = transition & 0xff;
                Send_Data.tx[8] = transition>>8;
                
                Send_Data.tx[10] = Check_Sum(10); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
                Send_Data.tx[11] = FRAME_TAIL; //frame tail 0xDF //帧尾0xDF
                ser.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  

            }
            
            /*********************
           Data: 2024 04 10
           功能：定时器回调函数，用于处理数据
           *********************/
            void TxCallback(const ros::TimerEvent &)
            {
                // ser.write("l");
                if (Get_Sensor_Data())
                {
                    switch (Receive_Data.rx[1])
                    {
                    case 0x01:
                        voltage_pub_();
                        break;
                    case 0x02:
                        start_flag_ = true;
                        odom_pub_();
                        break;
                    case 0x03:
                        imu_flag[0] = true;
                        IMU_pub_(1);
                        break;
                    case 0x04:
                        imu_flag[1] = true;
                        IMU_pub_(2);
                        break;
                    case 0x05:
                        imu_flag[2] = true;
                        IMU_pub_(3);
                        break;
                    }
                }
            }

            /*********************
           Data: 2024 04 10
           功能：串口初始化
           *********************/
            void ser_init()
            {
                ser.setPort(port_);
                ser.setBaudrate(baudrate_);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                ser.setTimeout(to);
                ser.open();

            }

            /**************************************
            Date: 4, 9, 2024
            Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
            功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
            ***************************************/
            bool Get_Sensor_Data()
            {
                short transition_16=0; //Intermediate variable //中间变量
                uint8_t i=0,check=0, error=1,Receive_Data_Pr[1]; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
                static int count; //Static variable for counting //静态变量，用于计数
                ser.read(Receive_Data_Pr,sizeof(Receive_Data_Pr));
                
                Receive_Data.rx[count] = Receive_Data_Pr[0]; //Fill the array with serial data //串口数据填入数组

                Receive_Data.Frame_Header = Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
                Receive_Data.Frame_Tail = Receive_Data.rx[11];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

                if(Receive_Data_Pr[0] == FRAME_HEADER || count>0) //Ensure that the first data in the array is FRAME_HEADER //确保数组第一个数据为FRAME_HEADER
                    count++;
                else
                     count = 0;
                if (count == 12)
                {
                    count = 0;
                     if(Receive_Data.Frame_Tail == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
                     {
                        return true;
                     }
                }
                return false;
            }

           /*********************
           Data: 2024 04 09
           功能：发布电压数据
           *********************/
            void voltage_pub_()
            {
                std_msgs::Float32 voltage;
                voltage.data = (Receive_Data.rx[2] << 8 & 0xff00 | Receive_Data.rx[3] & 0xff)/1000.0;
                voltage_pub.publish(voltage);
            }

           /*********************
           Data: 2024 04 09
           功能：处理IMU数据，发布IMU消息
           *********************/
            void IMU_pub_(int mode)
            {
                if(mode == 1)
                {
                    
                    imu_msg.angular_velocity.x = (int16_t)(Receive_Data.rx[2] << 8 | Receive_Data.rx[3])*0.1;
                    imu_msg.angular_velocity.y = (int16_t)(Receive_Data.rx[4] << 8 | Receive_Data.rx[5])*0.1;
                    imu_msg.angular_velocity.z = (int16_t)(Receive_Data.rx[6] << 8 | Receive_Data.rx[7])*0.1;
                    CheckAndPublishImu();
                }
                else if(mode == 2)
                {   
                    if((int16_t)(Receive_Data.rx[2] << 8 | Receive_Data.rx[3]) != 2)
                    imu_msg.linear_acceleration.x = (int16_t)(Receive_Data.rx[2] << 8 | Receive_Data.rx[3])*0.98;
                    if ((int16_t)(Receive_Data.rx[4] << 8 | Receive_Data.rx[5])*0.98 != 0)
                    imu_msg.linear_acceleration.y = (int16_t)(Receive_Data.rx[4] << 8 | Receive_Data.rx[5])*0.98;
                    imu_msg.linear_acceleration.z = (int16_t)(Receive_Data.rx[6] << 8 | Receive_Data.rx[7])*0.98;
                    std::cout << imu_msg.linear_acceleration.x << std::endl;
                    std::cout << imu_msg.linear_acceleration.y << std::endl;
                    std::cout << imu_msg.linear_acceleration.z << std::endl;
                    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl;
                    // imu_msg.linear_acceleration.x = 0;
                    // imu_msg.linear_acceleration.y = 0;
                    // imu_msg.linear_acceleration.z = 9.8;
                    CheckAndPublishImu();
                }
                else if(mode == 3)
                {
                    double roll = (int16_t)(Receive_Data.rx[2] << 8 | Receive_Data.rx[3]) * 0.001744444;
                    double pitch = (int16_t)(Receive_Data.rx[4] << 8 | Receive_Data.rx[5]) * 0.001744444;
                    double yaw = (int16_t)(Receive_Data.rx[6] << 8 | Receive_Data.rx[7]) * 0.001744444;
                    tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
                    imu_msg.orientation.x = q.x();
                    imu_msg.orientation.y = q.y();
                    imu_msg.orientation.z = q.z();
                    imu_msg.orientation.w = q.w();
                    CheckAndPublishImu();
                }
            }
            void CheckAndPublishImu(void)
            {
                if(imu_flag[0] & imu_flag[1] & imu_flag[2])
                {

                    imu_msg.header.frame_id = "imu";
                    imu_msg.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
                    imu_msg.orientation_covariance[4] = 1e6;
                    imu_msg.orientation_covariance[8] = 1e-6;
                    imu_msg.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
                    imu_msg.angular_velocity_covariance[4] = 1e6;
                    imu_msg.angular_velocity_covariance[8] = 1e-6;
                    imu_msg.header.stamp = ros::Time::now();
                    imu_pub.publish(imu_msg);
                    imu_flag[0] = imu_flag[1] = imu_flag[2] = false;
                }
            }

            /*********************
             Data: 2024 04 10
            功能：发布odom数据
            *********************/
            void odom_pub_()
            {
                odom_z_scale_positive= odom_z_scale_negative = 1;
                odom_x_scale = odom_y_scale = 1;
                odom_data_();
                Receive_Data.A_speed = float(Robot_Pos.A_encode_d)*int((1/count_time_)*100)/1560*PI*0.060/100;
                Receive_Data.B_speed = float(Robot_Pos.B_encode_d)*int((1/count_time_)*100)/1560*PI*0.060/100;
                Receive_Data.C_speed = float(Robot_Pos.C_encode_d)*int((1/count_time_)*100)/1560*PI*0.060/100;
                Receive_Data.D_speed = float(Robot_Pos.D_encode_d)*int((1/count_time_)*100)/1560*PI*0.060/100;

                Receive_Data.X_speed = float(Receive_Data.B_speed + Receive_Data.C_speed)/2.0;
                Receive_Data.Y_speed = float(Receive_Data.C_speed - Receive_Data.D_speed)/2.0;
                Receive_Data.Z_speed = float(Receive_Data.D_speed - Receive_Data.B_speed)/2.0/0.19;

                Receive_Data.X_speed=Receive_Data.X_speed*odom_x_scale;
                Receive_Data.Y_speed=Receive_Data.Y_speed*odom_y_scale;
                if(Receive_Data.Z_speed>=0)
                    Receive_Data.Z_speed=Receive_Data.Z_speed*odom_z_scale_positive;
                else
                    Receive_Data.Z_speed=Receive_Data.Z_speed*odom_z_scale_negative;
  
                //Speed * Time = displacement (odometer)
                //速度*时间=位移（里程计）
                Robot_Pos.X+=(Receive_Data.X_speed * cos(Robot_Pos.Z) - Receive_Data.Y_speed * sin(Robot_Pos.Z)) * count_time_; //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
                Robot_Pos.Y+=(Receive_Data.X_speed * sin(Robot_Pos.Z) + Receive_Data.Y_speed * cos(Robot_Pos.Z)) * count_time_; //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
                Robot_Pos.Z+=Receive_Data.Z_speed * count_time_; //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 
                

                static tf2_ros::StaticTransformBroadcaster  odom_broadcaster_;
                geometry_msgs::TransformStamped odom_trans;

                odom_trans.header.stamp = ros::Time::now();
                odom_trans.header.frame_id = odom_frame_;
                odom_trans.child_frame_id = baselink_frame_;
                odom_trans.transform.translation.x = Robot_Pos.X;
                odom_trans.transform.translation.y = Robot_Pos.Y;
                odom_trans.transform.translation.z = 0.0;

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
                odom_trans.transform.rotation = odom_quat;
                // odom_broadcaster_.sendTransform(odom_trans);

                nav_msgs::Odometry odom_msg;
                // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
// 
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.header.stamp = ros::Time::now();
                odom_msg.twist.twist.linear.x = Receive_Data.X_speed;
                odom_msg.twist.twist.linear.y = Receive_Data.Y_speed;
                odom_msg.twist.twist.angular.z = Receive_Data.Z_speed;

                odom_msg.child_frame_id = baselink_frame_;
                odom_msg.pose.pose.orientation = odom_quat;
                odom_msg.pose.pose.position.x = Robot_Pos.X;
                odom_msg.pose.pose.position.y = Robot_Pos.Y;
                // odom_msg.pose.pose.position.z = Robot_Pos.Z;
                // odom 协方差矩阵
                if (Receive_Data.X_speed==0 & Receive_Data.Y_speed==0 & Receive_Data.Z_speed == 0)
                {
                    for(int i=0;i<36;i++)
                    {
                        odom_msg.pose.covariance[i] = odom_pose_covariance[i];
                        odom_msg.twist.covariance[i] = odom_twist_covariance[i];
                    } 
                }
                else
                {
                    for(int i=0;i<36;i++)
                    {
                        odom_msg.pose.covariance[i] = odom_pose_covariance2[i];
                        odom_msg.twist.covariance[i] = odom_twist_covariance2[i];
                    } 
                }
                
                
                odom_pub.publish(odom_msg);
            }

            void odom_data_()
            {
                now_ = ros::Time::now();
                if((ros::Time::now()-last_time_).toSec()<1)
                {
                    count_time_ = (ros::Time::now()-last_time_).toSec();
                }
                else count_time_ = 0;
                ROS_INFO("%.4f",count_time_);
                
                if (start_flag_) 
                {
                    Robot_Pos.A_encode_d = -CalculateDelta(Robot_Pos.A_encode,((Receive_Data.rx[2] << 8 & 0xff00) | Receive_Data.rx[3] & 0xff));
                    Robot_Pos.B_encode_d = CalculateDelta(Robot_Pos.B_encode,((Receive_Data.rx[4] << 8 & 0xff00) | Receive_Data.rx[5] & 0xff));
                    Robot_Pos.C_encode_d = CalculateDelta(Robot_Pos.C_encode,((Receive_Data.rx[6] << 8 & 0xff00) | Receive_Data.rx[7] & 0xff));
                    Robot_Pos.D_encode_d = -CalculateDelta(Robot_Pos.D_encode,((Receive_Data.rx[8] << 8 & 0xff00) | Receive_Data.rx[9] & 0xff));
                    last_time_ = now_;
                    start_flag_ = false;
                    return;
                }


                
            }

            int CalculateDelta(int& current_encode, int receive_encode)
            {
                int delta;
                if (receive_encode > current_encode) {
                    delta = (receive_encode - current_encode) < (current_encode - receive_encode + 65535)
                                ? (receive_encode - current_encode)
                                : (receive_encode - current_encode - 65535);
                } else {
                    delta = (current_encode - receive_encode) < (receive_encode - current_encode + 65535)
                                ? (receive_encode - current_encode)
                                : (receive_encode - current_encode + 65535);
                }
                current_encode = receive_encode;
                return delta;
            }

            /*********************************
             * Data: 2024 04 11
             * 功能：发布odom的tf关系
            **********************************/
            // void publish_odom_tf()
            // {
            //     geometry_msgs::TransformStamped odom_trans;

            //     odom_trans.header.stamp = ros::Time::now();
            //     odom_trans.header.frame_id = odom_frame_;
            //     odom_trans.child_frame_id = baselink_frame_;
            //     odom_trans.transform.translation.x = Robot_Pos.X;
            //     odom_trans.transform.translation.y = Robot_Pos.Y;
            //     odom_trans.transform.translation.z = 0.0;

            //     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
            //     odom_trans.transform.rotation = odom_quat;
            //     odom_broadcaster_.sendTransform(odom_trans);
            // }
};



int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"base_node");
    CarBase base;
    return 0;
}
