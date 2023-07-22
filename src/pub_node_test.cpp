#include <thread>
#include <ctime>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <robot_data_pipeline_pkg/RobotState.h>
#include "data_trans.h"
#include "log_lib.h"


int main(int argc, char **argv){
    ///init ROS node
    ros::init(argc, argv, "RobotMini_State_Pub_Node");
    ros::NodeHandle nh("~");

    dataTrans::DataTrans data_trans;
    std::string xmlName = "/home/nvidia/ros_ws/rl_dog/src/robot_data_pipeline_pkg/robotGuide_1.xml";
    std::string clientID = "RLGet";
    std::string gaitName = "RL";
    data_trans.init(xmlName, clientID, gaitName);

    ros::Publisher pub_test = nh.advertise<std_msgs::String>("Hello",10);

    ros::Publisher robot_data_pub = nh.advertise<robot_data_pipeline_pkg::RobotState>("Robot_Data",10);
    robot_data_pipeline_pkg::RobotState robot_state_message;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "hello, world";
        pub_test.publish(msg);

        double test_msg = 0.5;

///////////////////////////////////////////////////////////////
        // data_trans.getRobotData();  //// 这行代码 只有 机器人运动控制器运行， 才能跑通
        for(int i=0;i<6;i++){
            robot_state_message.q[i*3] = 0;
            robot_state_message.q[i*3 + 1] = 0.71;
            robot_state_message.q[i*3 + 2] = -1.49;
            robot_state_message.qd[i] =  0;
            robot_state_message.trq[i] = 0;
        }
        for(int i=0;i<3;i++){
            robot_state_message.imu_data[i] = 0;
            robot_state_message.imu_data[3+i] = 0;
            robot_state_message.imu_data[6+i] = 0;      
            robot_state_message.body_pos[i] = 0;   
            robot_state_message.body_vel[i] = 0;     
        }


        robot_data_pub.publish(robot_state_message); 
///////////////////////////////////////////////////////////////

        // ros::spinOnce(); // 处理回调函数的 
        loop_rate.sleep();

        // spdlog::info("main freq");
    }

    return 0;
}

