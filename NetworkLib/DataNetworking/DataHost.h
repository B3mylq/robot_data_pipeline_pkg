#ifndef DATA_HOST_H
#define DATA_HOST_H

#include <string>
#include <map>
#include <sstream>
#include "DataType.h"
// #include "robot_definition.h"
#include "DataFrame.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace robot_libs
{
    namespace networking
    {
        class RobotStateRawData
        {
        public:
            std::string data_string;
            RobotStateRawData();
        };

        class MotionData
        {   
        public:
            static const int COL_NUM = 7;
            int status_word;
            int target_pos;
            int actual_pos;
            int actual_vel;
            int actual_cur;
            int digital_inputs;
            float trq_sensor_data;
            void Update(const std::vector<std::string>::iterator& data_vec_it);
        };

        class TrqSensorData
        {
        public:
            static const int COL_NUM = 6;
            float readings[COL_NUM];
            void Update(const std::vector<std::string>::iterator& data_vec_it);
        };

        class IMUData
        {
        public:
            static const int COL_NUM = 9;
            float gyro[3];
            float accel[3];
            float euler[3]; // RPY
            void Update(const std::vector<std::string>& data_vec);
        };

        class GaitInfo 
        {
        public:
            int gait_id;
            char gait_name[8];

            void Update(const std::vector<std::string>& data_vec);
        };

        class RobotStateData
        {
        public:
            int count;
            long long time_stamp;
            MotionData motion_data[18];
            TrqSensorData trq_sensor_data[3];
            IMUData imu_data;
            GaitInfo gait_info;

            void Update(std::vector<std::vector<std::string> >& data_vec);
        };

        class CustomJsonData 
        {
        public:
            CustomJsonData();
            std::string Serialize();
            int Deserialize(std::string str);
            void Update(std::vector<std::string>& data_vec);
            rapidjson::Document* data() { return &custom_data; };

        private:
            rapidjson::StringBuffer sb;
            rapidjson::Writer<rapidjson::StringBuffer> writer;
            rapidjson::Document custom_data;
        };

        class RobotMapData 
        {
        public:
            static const int MAP_WIDTH = 300;
            static const int MAP_LENGTH = 300;
            float map_data[MAP_LENGTH][MAP_WIDTH];
            float robot_displacement[3];
        };

        class DataHost 
        {
            public:
                DataHost();
                int GetDataFromHost(zmq::message_t& dest_msg, int type);
                int PutDataToHost(zmq::message_t& src_msg, int type);
                int DecodeUploadString(std::string data);

                CustomJsonData* GetCustomGaitData() { return &custom_gait_data_; };
                CustomJsonData* GetHMIData() { return &hmi_data_; };
                RobotStateData* GetRobotStateData() { return &robot_state_data_; };
                RobotMapData* GetRobotMapData() { return &robot_map_data_; };
                
            private:
                RobotStateRawData robot_data_;

                RobotStateData robot_state_data_;
                RobotMapData   robot_map_data_;
                CustomJsonData custom_gait_data_;
                CustomJsonData hmi_data_;

                ZmqMessageHelper helper_;
        };

    }
}

#endif
