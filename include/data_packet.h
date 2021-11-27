#pragma once

#include <stdint.h>
#include <stddef.h>

class DataPacket
{
public:
    size_t cursor;
    uint8_t *content;
    DataPacket(size_t);
    ~DataPacket();
    void writeInteger(uint64_t, size_t);
    void write32BitDouble(double);
};
