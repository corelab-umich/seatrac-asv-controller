#pragma once

#include <boost/asio.hpp>

namespace asv::networking
{

/**
 * Class to send UDP packets to a certain ip address/port. Works synchronously.
 */
class UDPSender
{
private:
  boost::asio::io_service io_service_{};
  boost::asio::ip::udp::socket socket_{ io_service_ };
  boost::asio::ip::udp::endpoint remote_endpoint_;

public:
  UDPSender() = default;

  /**
   * @brief Creates a new UDPSender and prepares a socket to send data with. Calls `init()` with  specified ip_address
   * and port
   * @param ip_address destination ip address
   * @param port destination port
   */
  UDPSender(const std::string& ip_address, unsigned short port);

  /**
   * @brief Configures and opens remote endpoint and udp socket with the specified ip_address and port. Must be called
   * before calling `send_message`.
   * @param ip_address destination ip address
   * @param port destination port
   */
  void init(const std::string& ip_address, unsigned short port);

  /**
   * @brief send a message to the ip address/port specified during
   * construction
   * @param buf the data to send
   * @returns boost error code associated with socket send operation as well
   * as bytes transferred over socket
   */
  std::pair<boost::system::error_code, size_t> send_message(const std::vector<unsigned char>& buf);
};
}  // namespace asv::networking