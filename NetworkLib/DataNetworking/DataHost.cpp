#include "DataHost.h"

namespace robot_libs
{
    namespace networking 
    {
        RobotStateRawData::RobotStateRawData()
        {
            //data_string = "10 MOTIONDTA 4663 278 277 10000 -1 491912 23.13 592 278 277 0 0 4981911 1.21 TRQDTA  0  0  0  12.3  0.1  15.5 IMUDTA  0.2 0.5 11.1 0.1 -0.3 -0.1 0.12 -0.41 -9.81  TSTAMP  1592789987591771235 GAITINFO tt CUSTOM  1  { \"turn_rate\" : 0.23, \"trot_vel\" : -0.76 }";
            data_string = "10 MOTIONDTA 4663 278 277 10000 -1 491912 23.13 592 278 277 0 0 4981911 1.21 TRQDTA  0  0  0  12.3  0.1  15.5 IMUDTA  0.2 0.5 11.1 0.1 -0.3 -0.1 0.12 -0.41 -9.81  TSTAMP  1592789987591771235 GAITINFO  -1  CUSTOM" ;
        }

        void MotionData::Update(const std::vector<std::string>::iterator& data_vec_it)
        {
            using namespace std;
            auto it = data_vec_it;
            status_word = stoi((*it).c_str()); it++;
            target_pos  = stol((*it).c_str()); it++;
            actual_pos  = stol((*it).c_str()); it++;
            actual_vel  = stol((*it).c_str()); it++;
            actual_cur  = stoi((*it).c_str()); it++;
            digital_inputs = stol((*it).c_str()); it++;
            trq_sensor_data = stof((*it).c_str());
        }

        void TrqSensorData::Update(const std::vector<std::string>::iterator& data_vec_it)
        {
            for(int i = 0; i < COL_NUM; i++)
            {
                auto it = data_vec_it + i;
                readings[i] = std::stof((*it).c_str());
            }
        }

        void IMUData::Update(const std::vector<std::string>& data_vec)
        {
            if (data_vec.size() != 9) 
                return;

            for(int i = 0; i < 3; i++)
            {
                gyro[i]  = std::stof(data_vec[i].c_str());
                accel[i] = std::stof(data_vec[3 + i].c_str());
                euler[i] = std::stof(data_vec[6 + i].c_str());
            }
        }

        void GaitInfo::Update(const std::vector<std::string>& data_vec)
        {
            if (data_vec.size() != 2)
                return;

            gait_id = std::stoi(data_vec[0]);
            strncpy(gait_name, data_vec[1].c_str(), 7);
            gait_name[7] = '\0';
        }

        void RobotStateData::Update(std::vector<std::vector<std::string> >& data_vec)
        {
            if (data_vec[0].size() > 0)
                count = std::stol(data_vec[0][0].c_str());

            for (int i = 0; i < data_vec[1].size() / MotionData::COL_NUM; i++)
                motion_data[i].Update(data_vec[1].begin() + i * MotionData::COL_NUM);

            for (int i = 0; i < data_vec[2].size() / TrqSensorData::COL_NUM; i++)
                trq_sensor_data[i].Update(data_vec[2].begin() + i * TrqSensorData::COL_NUM);

            imu_data.Update(data_vec[3]);

            if (data_vec[4].size() > 0)
                time_stamp = std::stoll(data_vec[4][0].c_str());
            
            gait_info.Update(data_vec[5]);
        }

        CustomJsonData::CustomJsonData() 
        : sb(), writer(sb)
        {
        }

        std::string CustomJsonData::Serialize()
        {
            sb.Clear();
            custom_data.Accept(writer);
            return  std::string(sb.GetString());
        }

        int CustomJsonData::Deserialize(std::string str)
        {
            custom_data.Clear();
            rapidjson::ParseResult result = custom_data.Parse(str.c_str());
            if (!result)
                return -1;
            return 0;
        }

        void CustomJsonData::Update(std::vector<std::string>& data_vec)
        {
            if (data_vec.size() == 2)
                Deserialize(*data_vec.rbegin());
            else
                Deserialize("{}");
        }

        DataHost::DataHost()
        {
        };

        int DataHost::GetDataFromHost(zmq::message_t &dest_msg, int type)
        {
            std::string str;
            switch(type)
            {
                case RobotDataType::ROBOT_STATE:
                    dest_msg.rebuild(sizeof(RobotStateData));
                    memcpy(dest_msg.data(), &robot_state_data_, sizeof(RobotStateData));
                    break;
                case RobotDataType::HEIGHT_MAP:
                    dest_msg.rebuild(sizeof(RobotMapData));
                    memcpy(dest_msg.data(), &robot_map_data_, sizeof(RobotMapData));
                    break;
                case RobotDataType::CUSTOM_GAIT_DATA:
                    str = custom_gait_data_.Serialize();
                    helper_.SetZmsgString(dest_msg, str);
                    break;
                case RobotDataType::HMI_DATA:
                    str = hmi_data_.Serialize();
                    helper_.SetZmsgString(dest_msg, str);
                    break;
                default:
                    helper_.SetZmsgString(dest_msg, "");
                    break;
            }
            return 0;
        }

        int DataHost::PutDataToHost(zmq::message_t &src_msg, int type)
        {
            std::string str;
            int ret = 0;
            switch(type)
            {
                case RobotDataType::ROBOT_STATE:
                    if (src_msg.size() != sizeof(RobotStateData))
                    {
                        return -1;
                    }
                    memcpy(&robot_state_data_, src_msg.data(), src_msg.size());
                    break;
                case RobotDataType::HEIGHT_MAP:
                    if (src_msg.size() != sizeof(RobotMapData))
                    {
                        return -1;
                    }
                    memcpy(&robot_map_data_, src_msg.data(), src_msg.size());
                    break;
                case RobotDataType::CUSTOM_GAIT_DATA:
                    if (src_msg.size() > 0)
                    {
                        str = helper_.GetZmsgString(src_msg);
                        ret = custom_gait_data_.Deserialize(str);
                    }
                    break;
                case RobotDataType::HMI_DATA:
                    if (src_msg.size() > 0)
                    {
                        str = helper_.GetZmsgString(src_msg);
                        ret = hmi_data_.Deserialize(str);
                    }
                    break;
                default:
                    helper_.SetZmsgString(src_msg, "");
                    ret = -1;
                    break;
            }
            return ret;
        }

        int DataHost::DecodeUploadString(std::string data)
        {
            using namespace std;
            //split data string into substrings
            stringstream ss(data);
            string token;

            string labels[] = {"MOTIONDTA", "TRQDTA", "IMUDTA", "TSTAMP", "GAITINFO", "CUSTOM"};
            int label_index = 0;
            vector<vector<string>> robot_state_data_vec(6);
            vector<string> custom_data_vec;

            // extract all the data areas from the original data string
            while(getline(ss, token, ' '))
            {
                if (token.size() <= 0)
                    continue;
                if (token != labels[label_index])
                {
                    robot_state_data_vec[label_index].push_back(token);
                }
                else
                {
                    label_index++;
                    if (token == "CUSTOM")
                        break;
                }
            }
            // the custom area should be treated in another way
            while (getline(ss, token, ' '))
            {
                if (token.size() > 0)
                {
                    custom_data_vec.push_back(token);
                    break;
                }
            }
            getline(ss, token, '\0');
            custom_data_vec.push_back(token);

            // update data area respectively
            robot_state_data_.Update(robot_state_data_vec);
            custom_gait_data_.Update(custom_data_vec);


            return 0;
        }
    }
}
