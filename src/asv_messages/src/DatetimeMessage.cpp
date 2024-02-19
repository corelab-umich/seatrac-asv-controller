#include "asv_messages/DatetimeMessage.h"

#include <asv_utils/utils/utils.h>
#include <iostream>
#include <stdexcept>

using asv::utils::to_ushort;
using asv::utils::ushort_to_bytes;

namespace asv::messages
{

const size_t DatetimeMessage::buffer_size;

DatetimeMessage DatetimeMessage::decode(const unsigned char* buf, size_t buf_len)
{
  if (buf_len != buffer_size)
  {
    throw std::invalid_argument("incorrect buffer size. must be" + std::to_string(buffer_size) + " bytes");
  }
  DatetimeMessage out{};
  out.year = to_ushort(buf);
  out.month = buf[2];
  out.day = buf[3];
  out.hour = buf[4];
  out.minute = buf[5];
  out.second = buf[6];
  out.hundreths = buf[7];

  std::cout << "year: " << out.year;
  return out;
}

unsigned char ushort_to_uchar(unsigned short num)
{
  // just truncate ushort to uchar
  return (unsigned char)(num & 0xFF);
}

std::vector<unsigned char> DatetimeMessage::encode()
{
  std::vector<unsigned char> out{};
  out.reserve(DatetimeMessage::buffer_size);
  auto year_bytes = ushort_to_bytes(year);
  out.insert(out.end(), year_bytes.begin(), year_bytes.end());
  out.push_back(ushort_to_uchar(month));
  out.push_back(ushort_to_uchar(day));
  out.push_back(ushort_to_uchar(hour));
  out.push_back(ushort_to_uchar(minute));
  out.push_back(ushort_to_uchar(second));
  out.push_back(ushort_to_uchar(hundreths));
  return out;
}
}  // namespace asv::messages