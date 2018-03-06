#pragma once

#include <thread>
#include <cstdint>
#include <sys/time.h>

#include <mavlink.h>

#include <serial_port_interface.hpp>
#include <mavlink_time_stamps.hpp>

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats ax ay az should be interpreted as force
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
 * bit 10: enable force instead of acceleration,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 */

enum class MAVSetLocalTargetTypeMask : uint16_t
{
	NED_POSITION		= 0b0000110111111000,
	NED_VELOCITY		= 0b0000110111000111,
	NED_ACCELERATION	= 0b0000110000111111,
	NED_FORCE			= 0b0000111000111111,
	NED_YAW_ANGLE		= 0b0000100111111111,
	NED_YAW_RATE		= 0b0000010111111111,
	NED_ALL				= 0b0000000000000000,
	NED_ALL_FORCES		= 0b0000001000000000
};

class MavlinkAdapter
{
public:
	MavlinkAdapter(SerialPortInterface& sp);
	bool IsOffboard() const;

private:
	uint8_t system_id;
	uint8_t autopilot_id;
	uint8_t companion_id;
	mavlink_heartbeat_t heartbeat;
	mavlink_sys_status_t sys_status;
	mavlink_battery_status_t battery_status;
	mavlink_radio_status_t radio_status;
	mavlink_local_position_ned_t local_position_ned;
	mavlink_global_position_int_t global_position_int;
	mavlink_position_target_local_ned_t position_target_local_ned;
	mavlink_position_target_global_int_t position_target_global_int;
	mavlink_highres_imu_t highres_imu;
	mavlink_attitude_t attitude;
	MavlinkTimeStamps time_stamps;

	bool exit;
	bool offboard_control;

	std::thread read;
	std::thread write;
	SerialPortInterface& serial_port;

	void read_thread();
	void read_messages();
	void write_setpoint(mavlink_set_position_target_local_ned_t sp);
};