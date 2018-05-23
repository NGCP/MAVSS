#include <chrono>
#include <iostream>

#include <mavlink_adapter.hpp>

uint64_t get_time_usec()
{
	auto cur_time = std::chrono::system_clock::now().time_since_epoch();
	return std::chrono::duration_cast<std::chrono::microseconds>(cur_time).count();
}

mavlink_message_t read_mavlink_message(SerialPortInterface& sp)
{
	mavlink_status_t status;
	mavlink_message_t message;

	uint8_t cp = sp.ReadByte();
	bool msg_recieved = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

	return message;
}

void MavlinkAdapter::read_messages()
{
	bool success;
	bool received_all = false;
	MavlinkTimeStamps this_timestamps;

	while(!received_all and !exit)
	{
		mavlink_message_t message = read_mavlink_message(serial_port);

		// TODO(kjayakum): Handle id's

		switch(message.msgid)
		{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				std::cout << "MAVLINK_MSG_ID_HEARTBEAT" << std::endl;
				mavlink_msg_heartbeat_decode(&message, &(heartbeat));
				time_stamps.heartbeat = get_time_usec();
				this_timestamps.heartbeat = time_stamps.heartbeat;
				break;
			}

			case MAVLINK_MSG_ID_SYS_STATUS:
			{
				//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
				mavlink_msg_sys_status_decode(&message, &(sys_status));
				time_stamps.system_status = get_time_usec();
				this_timestamps.system_status = time_stamps.system_status;
				break;
			}

			case MAVLINK_MSG_ID_BATTERY_STATUS:
			{
				//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
				mavlink_msg_battery_status_decode(&message, &(battery_status));
				time_stamps.battery_status = get_time_usec();
				this_timestamps.battery_status = time_stamps.battery_status;
				break;
			}

			case MAVLINK_MSG_ID_RADIO_STATUS:
			{
				//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
				mavlink_msg_radio_status_decode(&message, &(radio_status));
				time_stamps.radio_status = get_time_usec();
				this_timestamps.radio_status = time_stamps.radio_status;
				break;
			}

			case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			{
				//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
				mavlink_msg_local_position_ned_decode(&message, &(local_position_ned));
				time_stamps.local_position_ned = get_time_usec();
				this_timestamps.local_position_ned = time_stamps.local_position_ned;
				break;
			}

			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			{
				//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
				mavlink_msg_global_position_int_decode(&message, &(global_position_int));
				time_stamps.global_position_int = get_time_usec();
				this_timestamps.global_position_int = time_stamps.global_position_int;
				break;
			}

			case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
			{
				//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
				mavlink_msg_position_target_local_ned_decode(&message, &(position_target_local_ned));
				time_stamps.position_target_local_ned = get_time_usec();
				this_timestamps.position_target_local_ned = time_stamps.position_target_local_ned;
				break;
			}

			case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
			{
				//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
				mavlink_msg_position_target_global_int_decode(&message, &(position_target_global_int));
				time_stamps.position_target_global_int = get_time_usec();
				this_timestamps.position_target_global_int = time_stamps.position_target_global_int;
				break;
			}

			case MAVLINK_MSG_ID_HIGHRES_IMU:
			{
				//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
				mavlink_msg_highres_imu_decode(&message, &(highres_imu));
				time_stamps.highres_imu = get_time_usec();
				this_timestamps.highres_imu = time_stamps.highres_imu;
				break;
			}

			case MAVLINK_MSG_ID_ATTITUDE:
			{
				//printf("MAVLINK_MSG_ID_ATTITUDE\n");
				mavlink_msg_attitude_decode(&message, &(attitude));
				time_stamps.attitude = get_time_usec();
				this_timestamps.attitude = time_stamps.attitude;
				break;
			}

			default:
			{
				// printf("Warning, did not handle message id %i\n",message.msgid);
				break;
			}
		}

		received_all = this_timestamps.heartbeat && this_timestamps.system_status;
		//TODO(kjayakum): No writing thread status check?
	}
}

// This is just a test function to tell the px4 to stay in position
void write_zero()
{
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx				= 0.0;
	sp.vy				= 0.0;
	sp.vz				= 0.0;
	sp.yaw_rate			= 0.0;

	// Don't move quad
	sp.x				= 0.0;
	sp.y				= 0.0;
	sp.z				= 0.0;
	sp.target_system	= 5;
	sp.target_component	= 1;

}

void MavlinkAdapter::read_thread()
{
	while(!exit)
	{
		read_messages();
		// Read batches at 10Hz
		std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	}
}

void MavlinkAdapter::write_thread()
{
	write_zero();
	while(!exit)
	{
		// Write batches at 10Hz
		std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	}
}

MavlinkAdapter::MavlinkAdapter(SerialPortInterface& sp)
: serial_port(sp)
{
	exit = false;

	// Start connection
	std::cout << "Starting read thread" << std::endl;
	read = std::thread(&MavlinkAdapter::read_thread, this);
	std::cout << "Starting write thread" << std::endl;
	write = std::thread(&MavlinkAdapter::write_thread, this);
}