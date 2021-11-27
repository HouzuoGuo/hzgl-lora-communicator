#include <Arduino.h>
#include "data_packet.h"

DataPacket::DataPacket(size_t size)
{
    content = (uint8_t *)calloc(1, size);
    cursor = 0;
}

DataPacket::~DataPacket()
{
    free(content);
}

void DataPacket::writeInteger(uint64_t val, size_t num_bytes)
{
    // Write the value in little endian - the less significant number is written first.
    for (size_t x = 0; x < num_bytes; x++)
    {
        uint8_t next = 0;
        if (sizeof(val) > x)
        {
            // Take 256 (0, 1) for example:
            // Round 0 - shift right (divide by) 0 bits, BitAnd 255 (extract 1 byte): equals to 0.
            // Round 1 - shift right (divide by) 8b its, BitAnd 255 (extract 1 byte): equals to 1.
            next = static_cast<uint8_t>((val >> (x * 8)) & 0xFF);
        }
        content[cursor] = next;
        ++cursor;
    }
}

void DataPacket::write32BitDouble(double val)
{
    // Represent a double value, ranging from -21474.83648 to 21474.83647 inclusive, in 4 bytes.

    // Example 1 - converting 1234.5678 to bytes:
    // Multiply by 100000: equals to 123456780.
    // BitAnd 255 (extract the least significant 1 byte): equals to 12.
    // Shift right (divide by) 8 bits then BitAnd 255 (extract 1 byte): equals to 205.
    // Shift right (divide by) 16 bits then BitAnd 255 (extract 1 byte): equals to 91.
    // Shift right (divide by) 24 bits then BitAnd 255 (extract 1 byte): equals to 7.
    // 12 + 205 * 2^8 + 91 * 2^16 + 7 * 2^24 = 123456780

    // Example 2 - converting -1234.5678 to bytes:
    // Multiply by 100000: equals to -123456780.
    // Invert all bits and then plus 1: equals to -123456780
    // BitAnd 255 (extract the least significant 1 byte): equals to 244.
    // Shift right (divide by) 8 bits then BitAnd 255 (extract 1 byte): equals to 50.
    // Shift right (divide by) 16 bits then BitAnd 255 (extract 1 byte): equals to 164.
    // Shift right (divide by) 24 bits then BitAnd 255 (extract 1 byte): equals to 248.
    // 244 + 50 * 2^8 + 164 * 2^16 + 248 * 2^24 = 4171510516
    // 4171510516 is greater than 2147483647, therefore it represents a negative number.
    // Minus 1 and, invert all bits, and then BitAnd 2147483647: equals to 123456780.
    // Put the minus sign back in: equals to -123456780.
    int32_t val_int = (int32_t)(val * 100000);
    if (val < 0)
    {
        val_int = ~-val_int;
        val_int = val_int + 1;
    }
    content[cursor++] = (uint8_t)val_int & 0xFF;
    content[cursor++] = (uint8_t)((val_int >> 8) & 0xFF);
    content[cursor++] = (uint8_t)((val_int >> 16) & 0xFF);
    content[cursor++] = (uint8_t)((val_int >> 24) & 0xFF);
}
