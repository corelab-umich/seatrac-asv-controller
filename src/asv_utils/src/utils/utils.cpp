#include "asv_utils/utils/utils.h"

#include <math.h>
#include <stdexcept>
#include <cstring>
#include <array>
#include <algorithm>

namespace asv::utils
{

/**
 * Is this machine little endian?
 */
int isLittleEndian()
{
  int i = 1;
  return *((char*)&i);
}

unsigned short to_ushort(const unsigned char* buf, bool little_endian)
{
  unsigned short out = 0;
  if (little_endian)
  {
    out = buf[1];
    out <<= 8;
    out |= buf[0];
  }
  else
  {
    out = buf[0];
    out <<= 8;
    out |= buf[1];
  }

  return out;
}

short to_short(const unsigned char* buf, bool little_endian)
{
  unsigned short temp = to_ushort(buf, little_endian);

  return (short)temp;
}

double to_double(const unsigned char* buf, bool little_endian)
{
  bool machine_little_endian = isLittleEndian() == 1;
  double out = 0;
  if (little_endian == machine_little_endian)
  {
    memcpy(&out, buf, 8);
  }
  else
  {
    std::array<unsigned char, 8> rev{};
    std::reverse_copy(buf, buf + 8, rev.begin());
    memcpy(&out, rev.data(), 8);
  }

  return out;
}

float to_radians(float degrees)
{
  return degrees * M_PI / 180;
}

float to_degrees(float radians)
{
  return radians / M_PI * 180;
}

double to_radians(double degrees)
{
  return degrees * M_PI / 180;
}

double to_degrees(double radians)
{
  return radians / M_PI * 180;
}

std::vector<unsigned char> ushort_to_bytes(unsigned short num, bool little_endian)
{
  std::vector<unsigned char> out{};
  out.reserve(2);
  if (little_endian)
  {
    out.push_back((num)&0xFF);
    out.push_back((num >> 8) & 0xFF);
  }
  else
  {
    out.push_back((num >> 8) & 0xFF);
    out.push_back((num)&0xFF);
  }

  return out;
}

std::vector<unsigned char> double_to_bytes(double num, bool little_endian)
{
  bool machine_little_endian = isLittleEndian() == 1;
  unsigned char *num_buf = (unsigned char *) &num;
  auto out = std::vector<unsigned char>(num_buf, num_buf + 8);
  if (little_endian != machine_little_endian)
  {
    std::reverse(out.begin(), out.end());
  }

  return out;
}

std::vector<unsigned char> short_to_bytes(short num, bool little_endian)
{
  return ushort_to_bytes((unsigned short)num, little_endian);
}

void add_checksum_bytes(std::vector<unsigned char>& buf, int message_specific_data_length)
{
  unsigned char* pData = buf.data();
  int iDataLength = message_specific_data_length;
  int iHeaderLength = 6;
  int iSum1 = 0, iSum2 = 0;
  int iLength = iHeaderLength + iDataLength;
  while (iLength--)
  {
    iSum1 += *pData++;
    if (iSum1 >= 255)
      iSum1 -= 255;
    iSum2 += iSum1;
    if (iSum2 >= 255)
      iSum2 -= 255;
  }
  if (!((iSum1 <= 255) && (iSum2 <= 255)))
  {
    throw std::invalid_argument("checksum failed");
  }
  int iCheck1 = 255 - (iSum1 + iSum2) % 255;
  int iCheck2 = 255 - (iSum1 + iCheck1) % 255;
  buf.push_back((unsigned char)iCheck1);
  buf.push_back((unsigned char)iCheck2);
}

}  // namespace asv::utils