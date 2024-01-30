/**
 * Header file with UDPClient class that can read udp messages using the boost asio framework
 */

// system headers
#include <iostream>

// networking headers
#include <boost/asio.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

using boost::asio::ip::address;
using boost::asio::ip::udp;

/**
 * Interface for other classes that aim to handle messages captured by UDPClient should inherit from.
 * Subclasses should override the `handle_message` function.
 */
class MessageReceiver
{
public:
    /**
     * @brief function that is called on every packet captured by UDPClient.
     * @param buf unsigned char buffer containing the data of the captured packet
     * @param buf_size size of buf
     */
    virtual void handle_message(const unsigned char *buf, size_t buf_size) = 0;
};

/**
 * Class to asynchronously capture UDP packets and the appropriate message handler.
 */
class UDPClient
{
public:
    /**
     * @brief Creates a new UDPClient to start listening for UDP packets. DOES NOT START PACKET CAPTURE. use `start()` to start capturing packets and invoke handlers.
     * @param ip_address remote ip_address to listen to
     * @param port udp port to listen on
     * @param max_buffer_size sets the max packet size able to be read by this UDPClient. Any captured packet greater than this size is truncated.
     */
    UDPClient(const std::string &ip_address, int port, size_t max_buffer_size) : recv_buffer_(max_buffer_size), ip_address_(ip_address), port_(port){};
    

    // TODO (aspratap): change to accepy multiple handlers instead of single one
    /**
     * Start the asynchronous packet capture. returns immediately.
     * @param handler_obj for every packet captures, message handler is invoked on this object. Note that handler runs on same thread as
     * UDPClient causing packet capture to be suspended while handler is running
     *
     */
    void start(MessageReceiver &handler_obj)
    {
        socket_.open(udp::v4());
        // socket_.bind(udp::endpoint(address::from_string(ip_address_), port_));
        socket_.bind(udp::endpoint(udp::v4(), port_));
        handler_obj_ = &handler_obj;

        async_receive_message();

        std::cout << "starting receiving\n";
        io_service_.run();
    }

private:
    // internal handler invoked on each packet.
    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
    {
        if (error)
        {
            std::cout << "Receive failed: " << error.message() << "\n";
        }
        else
        {
            handler_obj_->handle_message(recv_buffer_.data(), bytes_transferred);
        }

        // start listening for the next packet
        async_receive_message();
    }

    void async_receive_message()
    {
        socket_.async_receive_from(boost::asio::buffer(recv_buffer_),
                                   remote_endpoint_,
                                   boost::bind(&UDPClient::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    boost::asio::io_service io_service_{};
    udp::socket socket_{io_service_};
    std::vector<unsigned char> recv_buffer_;
    udp::endpoint remote_endpoint_;
    std::string ip_address_;
    int port_;
    MessageReceiver *handler_obj_;
};