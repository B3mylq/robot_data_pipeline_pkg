#include "DataClient.h"
#include <algorithm>
#include "log_lib.h"
namespace robot_libs
{
    namespace networking 
    {
        zmq::context_t DataClient::client_context_(1);

        DataClient::DataClient() :
            requester_(client_context_, ZMQ_REQ)
        {
        }

        void DataClient::Initialize(const std::string& xmlFileName, const std::string& clientName)
        {
            aris::core::XmlDocument doc;

            if (doc.LoadFile(xmlFileName.c_str()) != 0){
                throw std::logic_error("failed to read configuration xml file");
            }


            auto host_ele = doc.RootElement()->FirstChildElement("DataAgent");
            std::string host_ip = host_ele->Attribute("ip");
            std::string host_router_port = host_ele->Attribute("router_port");
            std::string host_router_addr = "tcp://" + host_ip + ":" + host_router_port;
            auto this_ele = doc.RootElement()->FirstChildElement("Clients")->FirstChildElement(clientName.c_str());
            std::string this_net_id = this_ele->Attribute("net_id");

            requester_.setsockopt(ZMQ_IDENTITY, this_net_id.c_str(), this_net_id.size());
            requester_.connect(host_router_addr.c_str());
        }

        bool DataClient::SendCmd(const std::string& cmd)
        {
            RequestFrame frame;

            // ask agent to fwd the cmd to the robot server
            std::string req_type = "fwd";
            aris::core::Msg msg;
            msg.copy(cmd.c_str());

            frame.SetHeader(req_type);
            frame.SetContent(msg);

            // send and wait for reply
            bool ret = frame.Send(requester_);
            return ret;
        }

        int DataClient::GetData(RobotDataType type, DataHost& host)
        {
            RequestFrame frame;

            std::string req_type = "get " + std::to_string((int)type);
            std::string s = "";

            frame.SetHeader(req_type);
            frame.SetContent(s);

            // send and wait for reply
            bool ret = frame.Send(requester_);
            if (!ret)
                return -1;

            host.PutDataToHost(frame.reply_content, type);
            return frame.reply_content.size();
        }

        int DataClient::PutData(RobotDataType type, DataHost& host)
        {
            RequestFrame frame;

            std::string req_type = "put " + std::to_string((int)type);
            aris::core::Msg msg;

            frame.SetHeader(req_type);
            
            switch (type)
            {
                case RobotDataType::ROBOT_STATE:
                    frame.SetContent((const char *)host.GetRobotStateData(), sizeof(RobotStateData));
                    break;
                case RobotDataType::HEIGHT_MAP:
                    frame.SetContent((const char *)host.GetRobotMapData(), sizeof(RobotMapData));
                    break;
                case RobotDataType::CUSTOM_GAIT_DATA:
                    frame.SetContent(host.GetCustomGaitData()->Serialize());
                    break;
                case RobotDataType::HMI_DATA:
                    frame.SetContent(host.GetHMIData()->Serialize());
                    break;
                default:
                    std::cout << "invalid data type" << std::endl;
                    return -1;
            }

            // send and wait for reply
            bool ret = frame.Send(requester_);

            if (!ret)
                return -1;

            return frame.reply_content.size();
        }
    }
}