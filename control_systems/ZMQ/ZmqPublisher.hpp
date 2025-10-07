#ifndef ZMQ_PUBLISHER_HPP
#define ZMQ_PUBLISHER_HPP

#include <string>
#include <zmq.hpp>
#include <iostream>

class ZmqPublisher {
public:
    ZmqPublisher(zmq::context_t& context, const std::string& host, int port, const std::string& protocol = "tcp");

    ~ZmqPublisher();

    bool publishMessage(const std::string& data);

    bool isConnected() const;

private:
    zmq::context_t& _context;
    zmq::socket_t _socket;
    std::string _bindAddress;
    bool _isConnected;
};

#endif // ZMQ_PUBLISHER_HPP