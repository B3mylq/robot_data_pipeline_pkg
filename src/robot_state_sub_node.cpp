/**
 * @file robot_state_sub_node.cpp
 * @author yang (you@domain.com)
 * @brief ROS 下的 sub
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
#include <robot_data_pipeline_pkg/RobotCmd.h>
#include "data_trans.h"
#include "log_lib.h"


class RobotSub{
public:
    void init(){
        data_trans.init(xmlName, clientID, gaitName);
    }

    void callback_test(const std_msgs::String::ConstPtr& msg){
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

    void callback(const robot_data_pipeline_pkg::RobotCmd::ConstPtr& robotCmd){
        if(robotCmd->q_first_flag == 1.0){
            if(first_flag){
                first_flag = false;   
            }else{
                return;
            }
        }
        for(int i=0;i<18;i++){
            data_trans.q_target[i] = robotCmd->q_target[i];
            data_trans.qd_target[i] = robotCmd->qd_target[i];
            data_trans.trq_target[i] = robotCmd->trq_target[i];
        }
        data_trans.q_first_flag = robotCmd->q_first_flag;

        VecXd q_vec = Eigen::Map<VecXd>(data_trans.q_target,18);
        // spdlog::info("q_target{}",q_vec.transpose());
        spdlog::info("q_first_flag:{}",robotCmd->q_first_flag);
        data_trans.putRobotControl();

    }

    dataTrans::DataTrans data_trans;
    std::string xmlName = "/home/nvidia/ros_ws/rl_dog/src/robot_data_pipeline_pkg/robotGuide_1.xml";
    std::string clientID = "RLPut";
    // std::string gaitName = "tt";
    std::string gaitName = "RL";
    bool first_flag = true;

};


int main(int argc, char **argv){
    ///init ROS node
    ros::init(argc, argv, "RobotMini_State_Sub_Node");
    ros::NodeHandle nh;

    RobotSub robot_suber;
    robot_suber.init();

    // ros::Subscriber sub_test = nh.subscribe("/RobotMini_State_Pub_Node/Hello",10, &RobotSub::callback_test, &robot_suber);

    ///  topic 名字  改一下,  这个 频率 取决于 topic 的频率
    ros::Subscriber robot_cmd_sub = nh.subscribe("/RL_Gait/RobotCmd",1000, &RobotSub::callback, &robot_suber);

    ros::spin();

    return 0;
}

