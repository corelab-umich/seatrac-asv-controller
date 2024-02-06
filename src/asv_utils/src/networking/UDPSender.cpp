#include "asv_utils/networking/UDPSender.h"

namespace asv::networking {

UDPSender::UDPSender(const std::string &ip_address, unsigned short port)
    : remote_endpoint_(boost::asio::ip::address::from_string(ip_address),
                       port) {
    socket_.open(boost::asio::ip::udp::v4());
}

std::pair<boost::system::error_code, size_t>
UDPSender::send_message(const std::vector<unsigned char> &buf) {
    boost::system::error_code err;
    size_t sent =
        socket_.send_to(boost::asio::buffer(buf), remote_endpoint_, 0, err);
    return std::make_pair(err, sent);
}
} // namespace asv::networking