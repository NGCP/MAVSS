#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <system_error>

#include <serial_port_unix.hpp>

SerialPortUnix::SerialPortUnix() {}

SerialPortUnix::SerialPortUnix(const std::string& device_path, uint32_t baud_rate,
 uint8_t data_size, Parity parity, bool single_stop)
{
	uart_path = device_path;
	this->baud_rate = baud_rate;
	this->data_size = data_size;
	this->parity = parity;
	this->single_stop = single_stop;
	Connect(uart_path, this->baud_rate, this->data_size, this->parity, this->single_stop);
}

void SerialPortUnix::Connect(const std::string& path, uint32_t baud_rate,
 uint8_t data_size, Parity parity, bool single_stop)
{
	fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	struct termios port_config;
	bool is_connected = isatty(fd);
	bool config_written;

	if(!is_connected)
	{
		throw std::system_error(errno, std::system_category(), path);
	}

	fcntl(fd, F_SETFL, 0);
	int port_read_result = tcgetattr(fd, &port_config);
	if(port_read_result < 0)
	{
		throw std::system_error(errno, std::system_category(), path);
	}

	// Set input flags
	port_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
		INLCR | PARMRK | INPCK | ISTRIP | IXON);
	// Set output flags
	port_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
		ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		port_config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		port_config.c_oflag &= ~ONOEOT;
	#endif

	switch(data_size)
	{
		case 8: 
			port_config.c_cflag |= CS8;
			break;
		case 7:
			port_config.c_cflag |= CS7;
			break;
		case 6:
			port_config.c_cflag |= CS6;
			break;
		case 5:
			port_config.c_cflag |= CS5;
			break;
		default:
			throw std::runtime_error("Error: Data size must be 5, 6, 7, or 8\n");
	}

	if (single_stop == false) {
			port_config.c_cflag |= CSTOPB;
	}

	if (parity == Odd) {
		port_config.c_cflag &= ~(CSIZE | PARENB);
		port_config.c_cflag |= PARODD;
	}
	else if (parity == Even) {
		port_config.c_cflag &= ~(CSIZE | PARENB);
	}
	else if (parity == None) {
		port_config.c_cflag &= ~CSIZE;
	}
	else {
		throw std::runtime_error("Error: Parity must be None, Even, or Odd\n");
	}

	port_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	port_config.c_cc[VMIN]  = 1;
	port_config.c_cc[VTIME] = 10;
	
	switch(baud_rate)
	{
		case 1200:
			config_written =	(cfsetispeed(&port_config, B1200) < 0 ||
								cfsetospeed(&port_config, B1200) < 0) ? false : true;
			break;
		case 1800:
			config_written =	(cfsetispeed(&port_config, B1800) < 0 ||
								cfsetospeed(&port_config, B1800) < 0) ? false : true;
			break;
		case 9600:
			config_written =	(cfsetispeed(&port_config, B9600) < 0 ||
								cfsetospeed(&port_config, B9600) < 0) ? false : true;
			break;
		case 19200:
			config_written =	(cfsetispeed(&port_config, B19200) < 0 ||
								cfsetospeed(&port_config, B19200) < 0) ? false : true;
			break;
		case 38400:
			config_written =	(cfsetispeed(&port_config, B38400) < 0 ||
								cfsetospeed(&port_config, B38400) < 0) ? false : true;
			break;
		case 57600:
			config_written =	(cfsetispeed(&port_config, B57600) < 0 ||
								cfsetospeed(&port_config, B57600) < 0) ? false : true;
			break;
		case 115200:
			config_written =	(cfsetispeed(&port_config, B115200) < 0 ||
								cfsetospeed(&port_config, B115200) < 0) ? false : true;
			break;
		default:
			config_written = false;
			break;
	}

	if(!config_written)
	{
		throw std::system_error(errno, std::system_category(), path);
	}

	if(tcsetattr(fd, TCSAFLUSH, &port_config) < 0)
	{
		throw std::system_error(errno, std::system_category(), path);
	}
}

uint8_t SerialPortUnix::ReadByte()
{
	std::lock_guard<std::mutex> lock(io_mutex);
	uint8_t byte = 0;
	bool is_read = read(fd, &byte, 1) < 0 ? false : true;

	if(!is_read)
	{
		throw std::system_error(errno, std::system_category());
	}

	return byte;
}

int SerialPortUnix::WriteString(const std::string& message)
{
	std::lock_guard<std::mutex> lock(io_mutex);
	int bytes_written = static_cast<int>(write(fd, message.data(), message.size()));
	tcdrain(fd);

	if(bytes_written != message.size())
	{
		throw std::system_error(errno, std::system_category());
	}

	return bytes_written;
}