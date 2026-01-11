#ifndef ZMQ_BRIDGE_H
#define ZMQ_BRIDGE_H

#include <string>
#include <zmq.hpp>
#include <functional>

namespace slam {


class ZMQSubscriber {
public:

    explicit ZMQSubscriber(const std::string& address = "tcp://localhost:5555");

    ~ZMQSubscriber();
    

    void subscribe(const std::string& topic);
    
    bool receiveMessage(std::function<void(const std::string&, const std::string&)> callback, 
                       int timeout_ms = -1);

    void startListening(std::function<void(const std::string&, const std::string&)> callback);

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    std::string address_;
};


class ZMQPublisher {
public:

    explicit ZMQPublisher(const std::string& address = "tcp://*:5556");

    ~ZMQPublisher();

    void publishMessage(const std::string& topic, const std::string& message);

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    std::string address_;
};

} // namespace slam

#endif // ZMQ_BRIDGE_H
