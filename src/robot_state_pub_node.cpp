/**
 * @file robot_state_pub_node.cpp
 * @author yang (you@domain.com)
 * @brief 作为ROS Node 的 pub
 * @version 0.1
 * @date 2023-05-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

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

    ros::Rate loop_rate(20);//1000Hz

    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "hello, world";
        pub_test.publish(msg);


///////////////////////////////////////////////////////////////
        data_trans.getRobotData();  //// 这行代码 只有 机器人运动控制器运行， 才能跑通
        for(int i=0;i<18;i++){
            robot_state_message.q[i] = data_trans.q[i];
            robot_state_message.qd[i] = data_trans.qd[i];
            robot_state_message.trq[i] = data_trans.trq[i];
        }
        for(int i=0;i<3;i++){
            robot_state_message.imu_data[i] = data_trans.imu_rpy[i];
            robot_state_message.imu_data[3+i] = data_trans.imu_gyro[i];
            robot_state_message.imu_data[6+i] = data_trans.imu_acc[i];      
            robot_state_message.body_pos[i] = data_trans.body_pos[i];  
            // robot_state_message.body_vel[i] = 0.0;    
            robot_state_message.body_vel[i] = data_trans.body_vel[i];     
        }
        // spdlog::info("q_vec:{},{},{}",robot_state_message.q[0],robot_state_message.q[1],robot_state_message.q[2]);
        // spdlog::info("qd_vec:{},{},{}",robot_state_message.qd[0],robot_state_message.qd[1],robot_state_message.qd[2]);
        spdlog::info("rpy:{}",data_trans.imu_rpy.transpose());
        spdlog::info("gyro:{}",data_trans.imu_gyro.transpose());
        spdlog::info("acc:{}",data_trans.imu_acc.transpose());
        spdlog::info("body_height:{}",data_trans.body_pos[2]);
        spdlog::info("body_vel:{},{}",data_trans.body_vel[0],data_trans.body_vel[1]);
        robot_data_pub.publish(robot_state_message); 
///////////////////////////////////////////////////////////////

        // ros::spinOnce(); // 处理回调函数的 
        loop_rate.sleep();

        // spdlog::info("main freq");
    }

    return 0;
}

