#ifndef DATA_CLIENT_H
#define DATA_CLIENT_H

#include "DataType.h"
#include "DataFrame.h"
#include "DataHost.h"
#include "zmq.hpp"
#include <string>


namespace robot_libs
{
    namespace networking
    {
        class DataClient 
        {
            public:
                DataClient();
                void Initialize(const std::string& xmlFileName, const std::string& clientName);
                int  GetData(RobotDataType type, DataHost& host);
                int  PutData(RobotDataType type, DataHost& host);
                bool SendCmd(const std::string& cmd);
            
            private:
                static zmq::context_t client_context_;
                zmq::socket_t requester_;

        };
    }
}
#endif
