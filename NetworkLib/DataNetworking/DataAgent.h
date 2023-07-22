#ifndef DATA_AGENT_H
#define DATA_AGENT_H

#include <string>
#include <zmq.hpp>
#include "DataFrame.h"
#include "DataType.h"
#include "DataHost.h"

namespace robot_libs
{
    namespace networking 
    {
        class RobotDataAgent
        {
            public:
                RobotDataAgent();
                void initialize(std::string& xmlFileName, bool isTest);
                void run();
            
            private:
                // networking components
                static zmq::context_t agent_context_;
                zmq::socket_t request_router_;
                zmq::socket_t request_dealer_;
                zmq::socket_t robot_data_subscriber_;
                zmq::pollitem_t poll_items_[3];
                ZmqMessageHelper helper_;

                void handle_request(DataFrame& recv_frame);
                void handle_fwd(DataFrame& recv_frame);
                void handle_get(DataFrame& recv_frame, std::string& header_str);
                void handle_put(DataFrame& recv_frame, std::string& header_str);
                void handle_upload(DataFrame& recv_frame);

                // data backends
                DataHost data_host_;

        };


    }
}

#endif
