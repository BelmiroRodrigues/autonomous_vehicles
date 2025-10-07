#ifndef ZMQ_SUBSCRIBER_HPP
#define ZMQ_SUBSCRIBER_HPP

#include <zmq.hpp>
#include <string>
#include <atomic>
#include <thread>
#include <memory>

class ZmqSubscriber {
public:
    ZmqSubscriber(zmq::context_t& context, const std::string& host, int port, std::atomic<double>& speed_var, const std::string& protocol = "tcp");

    ~ZmqSubscriber();

    void start();

    void stop();

    bool isConnected() const;

private:
    void receiveLoop();

    zmq::context_t& _context;
    zmq::socket_t _socket;
    std::string _connectAddress;
    std::atomic<double>& _speed_var;
    std::unique_ptr<std::thread> _receiveThread;
    std::atomic<bool> _running{false};
    bool _isConnected{false};
};

#endif // ZMQ_SUBSCRIBER_HPP
