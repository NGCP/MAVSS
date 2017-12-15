#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <system_error>

#include <serial_port_unix.hpp>

const uint32_t kDefaultBaudRate = 57600;
const std::string kDefaultPath = "/dev/ttyUSB0";

SerialPortUnix::SerialPortUnix()
{
	uart_path = kDefaultPath;
	baud_rate = kDefaultBaudRate;
	Connect(kDefaultPath, kDefaultBaudRate);
}

// TODO(kjayakum): Add parameters for parity, I/O bit size & hardware control
// Note: This function requires POSIX compliant system calls
void SerialPortUnix::Connect(const std::string& path, uint32_t baud_rate)
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

	port_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	port_config.c_cflag &= ~(CSIZE | PARENB);
	port_config.c_cflag |= CS8;
	port_config.c_cc[VMIN]  = 1;
	port_config.c_cc[VTIME] = 10;
	
	// Apply Baudrate
	switch(baud_rate)
	{
		case 1200:
			config_written =	(cfsetispeed(&port_config, B1200) < 0 ||
								cfsetospeed(&port_config, B1200) < 0) ? false : true;
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

	is_open = true;
}

uint8_t SerialPortUnix::ReadByte()
{
	std::lock_guard<std::mutex> lock(io_mutex);
	uint8_t byte;
	// Should we handle when read returns 0 (EOF)?
	bool is_read = read(fd, &byte, 1) < 0 ? false : true;

	if(!is_read)
	{
		throw std::system_error(errno, std::system_category());
	}

	return byte;
}

int SerialPortUnix::WriteByte(const std::string& output)
{
	std::lock_guard<std::mutex> lock(io_mutex);
	int bytes_written = static_cast<int>(write(fd, output.data(), output.size()));
	tcdrain(fd);

	if(bytes_written == -1)
	{
		throw std::system_error(errno, std::system_category());
	}

	return bytes_written;
}