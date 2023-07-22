#ifndef DATA_FRAME_H
#define DATA_FRAME_H

#include <vector>
#include <zmq.hpp>
// #include <aris_core.h>
#include <iostream>
#include "aris_core_msg.h"
#include "aris_core_xml.h"
#include "log_lib.h"
namespace robot_libs
{
    namespace networking
    {
        class ZmqMessageHelper 
        {
            public:
                std::string GetZmsgString(const zmq::message_t &msg)
                {
                    char *buf = new char[msg.size() + 1];
                    memcpy(buf, msg.data(), msg.size());
                    buf[msg.size()] = '\0';

                    std::string s = buf;
                    delete[] buf;
                    return s;
                }

                void SetZmsgString(zmq::message_t &msg, const std::string s)
                {
                    msg.rebuild(s.size());
                    memcpy(msg.data(), s.c_str(), s.size());
                }
        };

        class DataFrame
        {
            public:
                static const int MAX_MSG_COUNT = 6;

                DataFrame()
                    : frame_msgs_(MAX_MSG_COUNT), msg_count_(0)
                {
                }

                int MsgCount() {return msg_count_; };
                void SetMsgCount(int count) { msg_count_ = count; };

                std::string str()
                {
                    ZmqMessageHelper helper;
                    std::string str = "FRAME:\n";
                    for (int i = 0; i < msg_count_; i++)
                    {
                        str += helper.GetZmsgString(frame_msgs_[i]) + '\n';
                    }
                    str += "END";
                    return str;
                }

                zmq::message_t *Content()
                {
                    return &(frame_msgs_[msg_count_ - 1]);
                }

                zmq::message_t *Header()
                {
                    return &(frame_msgs_[msg_count_ - 2]);
                }

                void Receive(zmq::socket_t& sock)
                {
                    int more;
                    size_t more_size = sizeof(more);
                    msg_count_ = 0;

                    while (1)
                    {
                        sock.recv(&frame_msgs_[msg_count_]);
                        msg_count_++;

                        sock.getsockopt(ZMQ_RCVMORE, &more, &more_size);
                        if (!more)
                            break;

                        if (msg_count_ == MAX_MSG_COUNT)
                            throw "Too many frame segments!";
                    }
                }

                void Send(zmq::socket_t& sock)
                {
                    for (int i = 0; i < msg_count_ - 1; i++)
                    {
                        sock.send(frame_msgs_[i], ZMQ_SNDMORE);
                    }
                    sock.send(frame_msgs_[msg_count_ - 1], 0);
                }

            private:
                std::vector<zmq::message_t> frame_msgs_;
                int msg_count_;
        };

        class RequestFrame
        {
            public:
                zmq::message_t request_header;
                zmq::message_t request_content;
                zmq::message_t reply_header;
                zmq::message_t reply_content;

                void SetHeader(const std::string& request_type)
                {
                    ZmqMessageHelper helper;
                    helper.SetZmsgString(request_header, request_type);
                }

                void SetContent(const char* src, size_t size)
                {
                    request_content.rebuild(size);
                    memcpy(request_content.data(), src, size);
                }

                void SetContent(const std::string& s)
                {
                    ZmqMessageHelper helper;
                    helper.SetZmsgString(request_content, s);
                }

                void SetContent(const aris::core::Msg& content_msg)
                {
                    content_msg.convertToZmqMsg(request_content);
                }

                bool Send(zmq::socket_t& sock)
                {   
                    sock.send(request_header, ZMQ_SNDMORE);
                    sock.send(request_content, 0);
                    bool ret = sock.recv(&reply_header);
                    if (!ret)
                        return ret;
                    ret = sock.recv(&reply_content);
                    return ret;
                }
        };

    }
}

#endif
