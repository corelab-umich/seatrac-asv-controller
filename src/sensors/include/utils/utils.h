/**
 * collection of various inline utility methods
 */

namespace util
{
    /**
     * @brief reads the first 2 elements of the specified char buffer and translates them into an unsigned short
     * @param buf pointer to a unsigned char buffer containing data to translate. First 2 elements are read.
     * @param little_endian endian-ness of the data in the buf. true by default
     */
    unsigned short to_ushort(const unsigned char *buf, bool little_endian = true)
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

    /**
     * @brief reads the first 2 elements of the specified char buffer and translates them into a short
     * @param buf pointer to a unsigned char buffer containing data to translate. First 2 elements are read.
     * @param little_endian endian-ness of the data in the buf. true by default
     */
    short to_short(const unsigned char *buf, bool little_endian = true)
    {
        unsigned short temp = to_ushort(buf, little_endian);

        return (short) temp;
    }
}