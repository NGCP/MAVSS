#pragma once

#include <string>
#include <cstdint>

class SerialPortInterface
{
public:
	enum Parity 
	{
		None, Even, Odd
	};
	virtual ~SerialPortInterface() {}
	virtual uint8_t ReadByte() = 0;
	virtual void Connect(const std::string& path, uint32_t baud_rate,
	 uint8_t data_size, Parity parity, bool single_stop) = 0;
	virtual int WriteString(const std::string& message) = 0;
};