#pragma once

#include <string>
#include <cstdint>
#include <mutex>

class SerialPort
{
public:
	SerialPort();
	SerialPort(const SerialPort& other) = delete;
	SerialPort& operator=(const SerialPort&) = delete;

private:
	std::mutex io_mutex;
	bool is_open;
	std::string uart_path;
	uint32_t baud_rate;
	uint32_t fd;

	void Connect(const std::string& path, uint32_t baud_rate);
	uint8_t ReadByte();
	int WriteByte(const std::string& output);
};