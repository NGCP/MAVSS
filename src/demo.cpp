#include <iostream>
#include <cstdint>

#include <serial_port_unix.hpp>

int main(int argc, char *argv[])
{
	std::cout << "Interactive Demo of MAVSS" << std::endl;
	SerialPortUnix sink("/dev/ttyUSB0", 57600);
}