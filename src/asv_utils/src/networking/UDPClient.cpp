#include "asv_utils/networking/UDPClient.h"

// system headers
#include <iostream>

// networking headers
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

using boost::asio::ip::address;
using boost::asio::ip::udp;

namespace asv::networking
{
    void UDPClient::start(MessageReceiver &handler_obj)
    {
        socket_.open(udp::v4());
        // socket_.bind(udp::endpoint(address::from_string(ip_address_), port_));
        socket_.bind(udp::endpoint(udp::v4(), port_));
        handler_obj_ = &handler_obj;

        async_receive_message();

        std::cout << "starting receiving\n";
        io_service_.run();
    }

    void UDPClient::handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
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

    void UDPClient::async_receive_message()
    {
        socket_.async_receive_from(boost::asio::buffer(recv_buffer_),
                                   remote_endpoint_,
                                   boost::bind(&UDPClient::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}
