#pragma once

#include <stdexcept>
#include <string>

class DeviceConnectionError : public std::runtime_error
{
public:
	
private:
	std::string device_path;
	uint32_t error_code;
};