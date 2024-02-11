#pragma once

#include <boost/asio.hpp>

namespace asv::networking {

/**
 * Class to send UDP packets to a certain ip address/port. Works synchronously.
 */
class UDPSender {
  private:
    boost::asio::io_service io_service_{};
    boost::asio::ip::udp::socket socket_{io_service_};
    boost::asio::ip::udp::endpoint remote_endpoint_;

  public:
    /**
     * @brief Creates a new UDPSender and prepares a socket to send data with.
     * @param ip_address destination ip address
     */
    UDPSender(const std::string &ip_address, unsigned short port);

    /**
     * @brief send a message to the ip address/port specified during
     * construction
     * @param buf the data to send
     * @returns boost error code associated with socket send operation as well
     * as bytes transferred over socket
     */
    std::pair<boost::system::error_code, size_t>
    send_message(const std::vector<unsigned char> &buf);
};
} // namespace asv::networking