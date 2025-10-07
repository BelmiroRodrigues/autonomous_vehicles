#include "ZmqSubscriber.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <errno.h>
#include <algorithm>

ZmqSubscriber::ZmqSubscriber(zmq::context_t& context, const std::string& host, int port, std::atomic<double>& speed_var, const std::string& protocol)
    : _context(context),
      _socket(_context, zmq::socket_type::sub),
      _speed_var(speed_var) {
    _connectAddress = protocol + "://" + host + ":" + std::to_string(port);
    std::cout << "ZMQ Subscriber: Connecting to " << _connectAddress << "..." << std::endl;

    try {
        _socket.connect(_connectAddress);
        std::cout << "ZMQ Subscriber: Connected to " << _connectAddress << std::endl;

        _socket.set(zmq::sockopt::subscribe, "");
        std::cout << "ZMQ Subscriber: Subscription pattern set (all messages)" << std::endl;

        int timeout = 1000;
        _socket.set(zmq::sockopt::rcvtimeo, timeout);
        std::cout << "ZMQ Subscriber: Receive timeout set to " << timeout << "ms" << std::endl;

        int rcvhwm = 1000;
        _socket.set(zmq::sockopt::rcvhwm, rcvhwm);

        int linger = 0;
        _socket.set(zmq::sockopt::linger, linger);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        _isConnected = true;
        std::cout << "ZMQ Subscriber: Successfully connected." << std::endl;

    } catch (const zmq::error_t& e) {
        std::cerr << "ZMQ Subscriber ERROR: Failed to connect: " << e.what() << std::endl;
        _isConnected = false;
    } catch (const std::exception& e) {
        std::cerr << "ZMQ Subscriber ERROR: Unexpected error in constructor: " << e.what() << std::endl;
        _isConnected = false;
    }
}

ZmqSubscriber::~ZmqSubscriber() {
    stop();
    try {
        _socket.close();
        std::cout << "ZMQ Subscriber: Socket closed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "ZMQ Subscriber ERROR: Error in destructor: " << e.what() << std::endl;
    }
}

void ZmqSubscriber::start() {
    if (!_isConnected || _running.load()) return;
    _running.store(true);
    std::cout << "ZMQ Subscriber: Starting receive thread..." << std::endl;
    _receiveThread = std::make_unique<std::thread>(&ZmqSubscriber::receiveLoop, this);
}

void ZmqSubscriber::stop() {
    _running.store(false);
    if (_receiveThread && _receiveThread->joinable()) {
        _receiveThread->join();
    }
}

bool ZmqSubscriber::isConnected() const {
    return _isConnected;
}

void ZmqSubscriber::receiveLoop() {
    auto last_heartbeat = std::chrono::steady_clock::now();
    std::size_t parse_errors = 0;

    while (_running.load()) {
        zmq::message_t msg;
        if (_socket.recv(msg, zmq::recv_flags::none)) {
            if (msg.size() == 0) {
                continue;
            }

            std::string data(static_cast<const char*>(msg.data()), msg.size());

            if (data.rfind("speed:", 0) == 0) {
                const size_t p = data.find(':');
                const size_t q = data.find(';', p + 1);
                if (p != std::string::npos && q != std::string::npos && q > p) {
                    const std::string value_str = data.substr(p + 1, q - p - 1);
                    try {
                        const double speed_mms = std::stod(value_str);
                        _speed_var.store(speed_mms / 1000.0);           // mm/s -> m/s
                    } catch (const std::exception& e) {
                        if ((++parse_errors % 10) == 1) {
                            std::cerr << "ZMQ Subscriber parse error (speed='" 
                                      << value_str << "'): " << e.what() << std::endl;
                        }
                    } catch (...) {
                        if ((++parse_errors % 10) == 1) {
                            std::cerr << "ZMQ Subscriber unknown parse error (speed payload)" 
                                      << std::endl;
                        }
                    }
                }
            }
            continue;
        }
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat).count() >= 5) {
            std::cout << "ZMQ Subscriber: idle (timeout)" << std::endl;
            last_heartbeat = now;
        }
    }
}

