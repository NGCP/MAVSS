#include <mavlink_adapter.hpp>

MavlinkAdapter::MavlinkAdapter(SerialPortInterface& sp)
: serial(sp)
{
	exit = false;
}