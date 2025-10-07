#include "ZmqPublisher.hpp"
#include <thread>
#include <chrono>

ZmqPublisher::ZmqPublisher(zmq::context_t& context, const std::string& host, int port, const std::string& protocol)
    : _context(context),
      _socket(_context, zmq::socket_type::pub),
      _isConnected(false) {

    _bindAddress = protocol + "://" + host + ":" + std::to_string(port);

    std::cout << "ZMQ Publisher: Binding to " << _bindAddress << "..." << std::endl;

    try {
        int sndhwm = 1000;
        _socket.set(zmq::sockopt::sndhwm, sndhwm);

        int linger = 0;
        _socket.set(zmq::sockopt::linger, linger);

        _socket.bind(_bindAddress);

        // Short delay to allow subscribers to connect before first messages (slow-joiner guard)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        _isConnected = true;
        std::cout << "ZMQ Publisher: Successfully bound." << std::endl;

    } catch (const zmq::error_t& e) {
        std::cerr << "ZMQ Publisher ERROR: Failed to bind to address: " << e.what() << std::endl;
        _isConnected = false;
    } catch (const std::exception& e) {
        std::cerr << "ZMQ Publisher ERROR: Unexpected error in constructor: " << e.what() << std::endl;
        _isConnected = false;
    }
}

ZmqPublisher::~ZmqPublisher() {
    if (_isConnected) {
        try {
            _socket.close();
            std::cout << "ZMQ Publisher: Socket closed." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "ZMQ Publisher ERROR: Error in destructor: " << e.what() << std::endl;
        }
    }
}

bool ZmqPublisher::publishMessage(const std::string& data) {
    if (!_isConnected) {
        std::cerr << "ZMQ Publisher ERROR: Publisher is not connected. Cannot send message." << std::endl;
        return false;
    }

    try {
        zmq::message_t message(data.begin(), data.end());
        zmq::send_result_t send_result = _socket.send(message, zmq::send_flags::none);

        if (!send_result) {
            std::cerr << "ZMQ Publisher ERROR: Failed to send message." << std::endl;
            return false;
        }

        return true;

    } catch (const zmq::error_t& e) {
        std::cerr << "ZMQ Publisher ERROR: Error publishing message: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "ZMQ Publisher ERROR: Unexpected error while publishing: " << e.what() << std::endl;
        return false;
    }
}

bool ZmqPublisher::isConnected() const {
    return _isConnected;
}
