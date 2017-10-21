#include <fcntl.h>
#include <unistd.h>

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
	}

	fcntl(fd, F_SETFL, 0);
	if(tcgetattr(fd, &port_config) < 0)
	{
		// Throw cannot read port configuration error
	}

	// Set input flags
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
		INLCR | PARMRK | INPCK | ISTRIP | IXON);
	// Set output flags
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
		ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10;
	
	// Apply Baudrate
	switch(baud_rate):
	{
		case 1200:
			config_written =	(cfsetispeed(&config, B1200) < 0 ||
								cfsetospeed(&config, B1200) < 0) ? false : true;
			break;
		default:
			config_written = false;
			break;
	}

	if(!config_written)
	{
		// Throw Baud Rate Configuration Error
	}

	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		// Throw Configuration Error
	}

	is_open = true;
}

SerialPort::ReadByte()
{
	
}