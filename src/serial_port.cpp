#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <serial_port.hpp>

const uint32_t kDefaultBaudRate = 57600;
// TODO(kjayakum): Path names are platform dependent
const std::string kDefaultPath = "/dev/ttyUSB0";

SerialPort::SerialPort()
{
	uart_path = kDefaultPath;
	baud_rate = kDefaultBaudRate;
}

// TODO(kjayakum): Add parameters for parity, I/O bit size & hardware control
// TODO(kjayakum): Change
// Note: This function requires POSIX compliant system calls
void SerialPort::Connect(const std::string& path, uint32_t baud_rate)
{
	fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	struct termios port_config;
	bool is_connected = isatty(fd);
	bool config_written;

	if(!is_connected)
	{
		// Throw device connection error
		throw std::runtime_error("No device connected!");
	}

	fcntl(fd, F_SETFL, 0);
	if(tcgetattr(fd, &port_config) < 0)
	{
		// Throw cannot read port configuration error
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
		// Throw Baud Rate Configuration Error
	}

	if(tcsetattr(fd, TCSAFLUSH, &port_config) < 0)
	{
		// Throw Configuration Error
	}

	is_open = true;
}

uint8_t SerialPort::ReadByte()
{
	
}