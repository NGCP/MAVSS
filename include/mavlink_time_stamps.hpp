#pragma once

#include <cstdint>

class MavlinkTimeStamps
{
public:
	void reset();
	uint64_t heartbeat;
	uint64_t system_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
private:
};