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
        ROS_INFO("get robot cmd");
        for(int i=0;i<18;i++){
            data_trans.q_target[i] = robotCmd->q_target[i];
            data_trans.qd_target[i] = robotCmd->qd_target[i];
            data_trans.trq_target[i] = robotCmd->trq_target[i];
        }
        ROS_INFO("send robot cmd");
        // data_trans.putRobotControl();

    }

    dataTrans::DataTrans data_trans;
    std::string xmlName = "/home/nvidia/ros_ws/rl_dog/src/robot_data_pipeline_pkg/robotGuide_1.xml";
    std::string clientID = "RLGet";
    std::string gaitName = "RL";
    

};


int main(int argc, char **argv){
    ///init ROS node
    ros::init(argc, argv, "RobotMini_State_Sub_Node");
    ros::NodeHandle nh;

    RobotSub robot_suber;
    robot_suber.init();

    ros::Subscriber sub_test = nh.subscribe("/RobotMini_State_Pub_Node/Hello",10, &RobotSub::callback_test, &robot_suber);

    ///  topic 名字  改一下,  这个 频率 取决于 topic 的频率
    ros::Subscriber robot_cmd_sub = nh.subscribe("/RL_Gait/RobotCmd",10, &RobotSub::callback, &robot_suber);

    ros::spin();

    return 0;
}
