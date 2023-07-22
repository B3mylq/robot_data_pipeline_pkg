#include "DataAgent.h"

namespace robot_libs
{
    namespace networking 
    {
        zmq::context_t RobotDataAgent::agent_context_(1);

        RobotDataAgent::RobotDataAgent()
        : request_router_(agent_context_, ZMQ_ROUTER),
          request_dealer_(agent_context_, ZMQ_DEALER),
          robot_data_subscriber_(agent_context_, ZMQ_SUB)
        {
        }

        void RobotDataAgent::initialize(std::string& xmlFileName, bool isTest)
        {
            using namespace std;

            aris::core::XmlDocument doc;

            if (doc.LoadFile(xmlFileName.c_str()) != 0)
                throw std::logic_error("failed to read configuration xml file");


            auto host_ele = doc.RootElement()->FirstChildElement("DataAgent");
            string host_ip = host_ele->Attribute("ip");

            string router_port = host_ele->Attribute("router_port");
            string router_addr = "tcp://" + host_ip + ":" + router_port;

            string dealer_port = host_ele->Attribute("dealer_port");
            string dealer_addr = "tcp://" + host_ip + ":" + dealer_port;

            cout << "Request router at: " << router_addr << endl;

            // establish req router
            request_router_.bind(router_addr.c_str());

            // connect to the robot's data publisher

            aris::core::XmlElement* rbt_ele;
            if (isTest)
                rbt_ele = doc.RootElement()->FirstChildElement("Clients")->FirstChildElement("FakeServer");
            else
                rbt_ele = doc.RootElement()->FirstChildElement("Server");
            
            string rbt_ip = rbt_ele->Attribute("ip");
            string rbt_data_pub_port = rbt_ele->Attribute("data_pub_port");
            string rbt_pub_addr = "tcp://" + rbt_ip + ":" + rbt_data_pub_port;
            cout << "Pub addr: " << rbt_pub_addr << endl;
            robot_data_subscriber_.connect(rbt_pub_addr.c_str());
            robot_data_subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);

            // establish req dealer
            string rbt_cmd_port = rbt_ele->Attribute("cmd_port");
            string rbt_cmd_addr = "tcp://" + rbt_ip + ":" + rbt_cmd_port;
            request_dealer_.connect(rbt_cmd_addr.c_str());

            // poll settings
            poll_items_[0].socket = (void *)request_router_;
            poll_items_[0].events = ZMQ_POLLIN;
            poll_items_[1].socket = (void *)request_dealer_;
            poll_items_[1].events = ZMQ_POLLIN;
            poll_items_[2].socket = (void *)robot_data_subscriber_;
            poll_items_[2].events = ZMQ_POLLIN;
        }

        void RobotDataAgent::run()
        {
            using namespace std;
            while (1)
            {
                zmq::poll(poll_items_, 3, 100); // timeout = 100ms

                // received request frome router
                if (poll_items_[0].revents & ZMQ_POLLIN)
                {
                    DataFrame frame;
                    frame.Receive(request_router_);

                    handle_request(frame);
                }
                // received reply from request dealer
                if (poll_items_[1].revents & ZMQ_POLLIN)
                {
                    DataFrame frame;
                    frame.Receive(request_dealer_);

                    frame.Send(request_router_);
                }
                // received published data updates from the robot
                if (poll_items_[2].revents & ZMQ_POLLIN)
                {
                    DataFrame frame;
                    frame.Receive(robot_data_subscriber_);

                    handle_upload(frame);
                }
            }
        }

        void RobotDataAgent::handle_request(DataFrame& recv_frame)
        {
            using namespace std;
            if (recv_frame.MsgCount() != 4)
            {
                cout << "Invalid request received" << endl;
                cout << recv_frame.str() << endl;
                return;
            }

            zmq::message_t* p_header_msg = recv_frame.Header();
            string header_str = helper_.GetZmsgString(*p_header_msg);

            cout << "Message header info: " <<  header_str << endl;

            if ( header_str == "fwd")
            {
                handle_fwd(recv_frame);
            }
            else if ( header_str.substr(0, 3) == "get")
            {
                handle_get(recv_frame, header_str);
            }
            else if ( header_str.substr(0, 3) == "put")
            {
                handle_put(recv_frame, header_str);
            }
            else
            {
                helper_.SetZmsgString(*recv_frame.Content(), "");
                recv_frame.Send(request_router_);
            }
        }

        void RobotDataAgent::handle_fwd(DataFrame& recv_frame)
        {
            recv_frame.Send(request_dealer_);
            std::cout << "Forward to robot" << std::endl;
        }

        void RobotDataAgent::handle_get(DataFrame& recv_frame, std::string& header_str)
        {
            int data_type_index = -1;
            try
            {
                data_type_index = std::stoi(header_str.substr(4));
            }
            catch (std::exception &e)
            {
                std::cout << "Failed to convert data type" << std::endl;
                return;
            }

            data_host_.GetDataFromHost(*recv_frame.Content(), data_type_index);
            recv_frame.Send(request_router_);
        }

        void RobotDataAgent::handle_put(DataFrame &recv_frame, std::string &header_str)
        {
            int data_type_index = -1;
            try
            {
                data_type_index = std::stoi(header_str.substr(4));
            }
            catch (std::exception &e)
            {
                std::cout << "Failed to convert data type" << std::endl;
                return;
            }
            int ret = data_host_.PutDataToHost(*recv_frame.Content(), data_type_index);
            if (ret < 0)
                std::cout << "Failed to put data" << std::endl;

            helper_.SetZmsgString(*recv_frame.Content(), "");
            recv_frame.Send(request_router_);
        }

        void RobotDataAgent::handle_upload(DataFrame &recv_frame)
        {
            using namespace std;
            aris::core::Msg data_msg;
            data_msg.convertFromZmqMsg(*recv_frame.Content());

            if (data_msg.data()[data_msg.size() - 1] == '\0')
            {
                string s(data_msg.data());
                data_host_.DecodeUploadString(s);
            }
        }
    }

}
