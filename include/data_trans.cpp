#include "data_trans.h"

using robot_libs::networking::RobotDataType;


namespace dataTrans{


DataTrans::DataTrans(){

}
DataTrans::~DataTrans(){

} 

/// @brief 根据 xml内容信息 初始化  client  
/// @param xmlFileName 
void DataTrans::init(const std::string &xmlFileName_, const std::string &clientID_, const std::string & gaitName_){
    client.Initialize(xmlFileName_, clientID_);
    gaitName = gaitName_;

}

void DataTrans::getRobotData(){

    getRobotRawData();
    // spdlog::info("test1");
    // getRobotCustomData();
    // spdlog::info("test2");
    getDataTransform();



}

void DataTrans::putRobotControl(){
    // putDataTransform();
    downloadData2Robot();
}


    



/// -------------------  private functions ------------------------///

void DataTrans::getDataTransform(){
    /// 转换顺序----------------------------///
    int list_id[18] = {14,13,12, 17,16,15, 2,1,0, 5,4,3, 8,7,6, 11,10,9};
    for(int i=0;i<18;i++){
        if(i%3!=0){
            q[i] = - q_raw[list_id[i]];
            qd[i] = - qd_raw[list_id[i]];
            trq[i] = - trq_raw[list_id[i]];            
        }else{
            q[i] = q_raw[list_id[i]];
            qd[i] = qd_raw[list_id[i]];
            trq[i] = trq_raw[list_id[i]]; 
        }
    }

    imu_rpy = Vec3d(imu_rpy_raw[0],imu_rpy_raw[1],imu_rpy_raw[2]);
    imu_gyro = Vec3d(imu_gyro_raw[0],imu_gyro_raw[1],imu_gyro_raw[2]);
    imu_acc = Vec3d(imu_acc_raw[0],imu_acc_raw[1],imu_acc_raw[2]);

    body_pos = Vec3d(0,0,body_pos_raw[2]);
    body_vel = Vec3d(body_vel_raw[0],body_vel_raw[1],0);

}

void DataTrans::putDataTransform(){
    int list_id[18] = {14,13,12, 17,16,15, 2,1,0, 5,4,3, 8,7,6, 11,10,9};

    for(int i=0;i<18;i++){
        if(i%3!=0){
            q_cmd[i] = - q_target[list_id[i]];
           
        }else{
            q_cmd[i] = q_target[list_id[i]];
 
        }
    }

}

void DataTrans::getRobotRawData(){
    int byte_received = client.GetData(RobotDataType(1), data_host);/// 获取机器人本体传感器数据
    if(byte_received==-1) std::cout << "error happened when reading robot raw data" << std::endl;
    for(int i=0;i<18;i++){
        q_raw[i] = data_host.GetRobotStateData()->motion_data[i].actual_pos/pos2count_ratio[i];
        qd_raw[i] = data_host.GetRobotStateData()->motion_data[i].actual_vel/pos2count_ratio[i];
        trq_raw[i] = data_host.GetRobotStateData()->motion_data[i].actual_cur / 1000 * max_current * trq_constant * gear_ratio[i];
    }
    for(int i=0;i<3;i++){
        imu_rpy_raw[i] = data_host.GetRobotStateData()->imu_data.euler[i];
        imu_gyro_raw[i] = data_host.GetRobotStateData()->imu_data.gyro[i];
        imu_acc_raw[i] = data_host.GetRobotStateData()->imu_data.accel[i];        
    }
}

void DataTrans::getRobotCustomData(){
    int byte_received = client.GetData(RobotDataType(3), data_host); /// 获取机器人位置、速度估计的数据
    if(byte_received==-1) std::cout << "error happened when reading robot gait custom data" << std::endl;
    auto* p_data = data_host.GetCustomGaitData()->data();
    // std::cout << data_host.GetCustomGaitData()->Serialize() << std::endl; /// 后续删除
    rapidjson::Value& param1 = (*p_data)["body_height"];
    rapidjson::Value& param2 = (*p_data)["body_vel_x"];
    rapidjson::Value& param3 = (*p_data)["body_vel_y"];  
    body_pos_raw[2] = param1.GetDouble();
    body_vel_raw[0] = param2.GetDouble();
    body_vel_raw[1] = param3.GetDouble();
}


void DataTrans::downloadData2Robot(){
    std::string cmdName = gaitName + " --online" + " --rl_control_flag=1.0" + " --first_q_flag=";
    cmdName = cmdName + std::to_string(q_first_flag);
    cmdName =cmdName + " --update_q_flag=1.0" + " --q_target=";
    for(int i=0;i<17;i++){
        cmdName = cmdName + std::to_string(q_target[i]) + ",";
    }
    cmdName = cmdName + std::to_string(q_target[17]);
    spdlog::info("cmdName:{}",cmdName);


    bool ret = client.SendCmd(cmdName);
    spdlog::info("ret:{}",ret);
    if(!ret) std::cout << "error happen when send cmd" << std::endl;
}



}//namespace dataTrans





