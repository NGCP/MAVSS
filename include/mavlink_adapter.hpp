#pragma once

#include <string>
#include <thread>
#include <signal.h>
#include <time.h>
#include <cstdint>
#include <sys/time.h>

#include <mavlink.h>

#include <serial_port_interface.hpp>

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 */

enum class LocalPositionMask : uint16_t
{
	NED_POSITION		= 0b0000110111111000,
	NED_VELOCITY		= 0b0000110111000111,
	NED_ACCELERATION	= 0b0000110000111111,
	NED_FORCE			= 0b0000111000111111,
	NED_YAW_ANGLE		= 0b0000100111111111,
	NED_YAW_RATE		= 0b0000010111111111
};

class MavlinkAdapter
{
public:
	MavlinkAdapter(SerialPortInterface& sp);

private:
	bool exit;

	std::thread read;
	std::thread write;
	SerialPortInterface& serial;
};