#pragma once

/**
 * Header file with UDPClient class that can read udp messages using the boost asio framework
 */

// networking headers
#include <boost/asio.hpp>
#include "MessageReceiver.h"

namespace asv::networking
{
/**
 * Class to asynchronously capture UDP packets and the appropriate message handler.
 */
class UDPClient
{
public:
  /**
   * @brief Creates a new UDPClient to start listening for UDP packets. DOES NOT START PACKET CAPTURE. use `start()` to
   * start capturing packets and invoke handlers.
   * @param ip_address remote ip_address to listen to
   * @param port udp port to listen on
   * @param max_buffer_size sets the max packet size able to be read by this UDPClient. Any captured packet greater than
   * this size is truncated.
   */
  UDPClient(const std::string& ip_address, unsigned short port, size_t max_buffer_size)
    : recv_buffer_(max_buffer_size), ip_address_(ip_address), port_(port){};

  // TODO (aspratap): change to accepy multiple handlers instead of single one
  /**
   * Start the asynchronous packet capture. returns immediately.
   * @param handler_obj for every packet captures, message handler is invoked on this object. Note that handler runs on
   * same thread as UDPClient causing packet capture to be suspended while handler is running
   *
   */
  void start(MessageReceiver& handler_obj);

private:
  // internal handler invoked on each packet.
  void handle_receive(const boost::system::error_code& error, size_t bytes_transferred);

  // async call to listen for next packet. invokes `handle_receive` when said packet is received
  void async_receive_message();

  boost::asio::io_service io_service_{};
  boost::asio::ip::udp::socket socket_{ io_service_ };
  std::vector<unsigned char> recv_buffer_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  std::string ip_address_;
  unsigned short  port_;
  MessageReceiver* handler_obj_;
};
}  // namespace asv::networking
