#pragma once

#include <cstddef>

namespace asv::networking
{
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
  virtual void handle_message(const unsigned char* buf, size_t buf_size) = 0;
};
}  // namespace asv::networking