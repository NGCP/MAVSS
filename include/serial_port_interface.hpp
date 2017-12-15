#pragma once

#include <string>
#include <cstdint>

class SerialPortInterface
{
public:
	virtual ~SerialPortInterface() {}
	virtual uint8_t ReadByte() = 0;
	virtual int WriteByte(const std::string& output) = 0;
	virtual void Connect(const std::string& path, uint32_t baud_rate) = 0;
};