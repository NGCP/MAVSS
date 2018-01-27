#pragma once

#include <mutex>

#include <serial_port_interface.hpp>

class SerialPortUnix : public SerialPortInterface
{
public:
	SerialPortUnix();
	SerialPortUnix(const std::string& device_path, uint32_t baud_rate,
	 uint8_t data_size = 8, Parity parity = None, bool single_stop = true);
	SerialPortUnix(const SerialPortUnix& other) = delete;
	SerialPortUnix& operator=(const SerialPortUnix&) = delete;

	void Connect(const std::string& path, uint32_t baud_rate,
	 uint8_t data_size, Parity parity, bool single_stop);
	uint8_t ReadByte();
	int WriteByte(const std::string& output);

private:
	std::mutex io_mutex;
	bool is_open;
	std::string uart_path;
	uint32_t baud_rate;
	uint32_t fd;
	uint8_t data_size;
	bool single_stop;
	Parity parity;
};