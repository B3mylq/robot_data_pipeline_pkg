/**
 * @file data_trans.h
 * @author yang (you@domain.com)
 * @brief 基于DataClient 搭建的 机器人 数据传输通道
 *          从机器人本体采集数据,通过 ros topic pub, 
 *          sub 机器人控制指令, 下发给 机器人本体
 * @version 0.1
 * @date 2023-04-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef DATA_TRANS_H
#define DATA_TRANS_H

#include <zmq.hpp>
#include "DataClient.h"
#include "DataType.h"
#include "DataHost.h"
#include <exception>
#include "eigen_lib.h"
#include "log_lib.h"

namespace dataTrans{


class DataTrans{

public:
    DataTrans(); 
    ~DataTrans(); 

    void init(const std::string &xmlFileName_, const std::string &clientID_, const std::string & gaitName_);

    void getRobotData();

    void putRobotControl();


    //// 上层所能使用的数据  懒得写到 private， 外加 get函数了------------------------///
    double p[18];
    double q[18];
    double qd[18];
    double trq[18];
    Vec3d imu_rpy;
    Vec3d imu_gyro;
    Vec3d imu_acc;
    Vec3d body_pos;///世界坐标系下的高度
    Vec3d body_vel;///body坐标系下的x，y轴速度


    /// --------- 下发给机器人的数据
    double q_target[18];
    double qd_target[18];
    double trq_target[18];
    double q_first_flag;



    double q_cmd[18];



    void getRobotRawData();
    void getRobotCustomData();
    void getDataTransform();
    void putDataTransform();

    void downloadData2Robot();






    
/// ------------  从机器人读取的 原始数据 ---- 可能需要转换
    double q_raw[18];
    double qd_raw[18];
    double trq_raw[18];
    double imu_rpy_raw[3];
    double imu_gyro_raw[3];
    double imu_acc_raw[3];

    double body_pos_raw[3];
    double body_vel_raw[3];


    double pos2count_ratio[18] = {42243.033,23468.3517,23468.3517,42243.033,23468.3517,23468.3517,42243.033,23468.3517,23468.3517,
                                        42243.033,23468.3517,23468.3517,42243.033,23468.3517,23468.3517,42243.033,23468.3517,23468.3517};
    double gear_ratio[18] = {16.2,9,9,16.2,9,9,16.2,9,9,16.2,9,9,16.2,9,9,16.2,9,9};
    double trq_constant = 0.10;
    double max_current = 21.21;




///  gait name --------------
    std::string gaitName;

/// 网络 节点 ---------------------------///
    robot_libs::networking::DataHost data_host; /// 数据交互 buffer
    robot_libs::networking::DataClient client;




};




}//namespace dataTrans

#endif // !DATA_TRANS_H


