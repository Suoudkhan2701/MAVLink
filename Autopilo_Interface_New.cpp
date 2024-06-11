#include<Autopilot_Interface_Addition.h>

uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return ((_time_stamp.tv_sec*1000000) +(_time_stamp.tv_usec));
}

void set_position(float x, float y, float z, float r, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask=MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.x   = x;
	sp.y   = y;
	sp.z   = z;
	sp.r   = r;
	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);
}
void set_velocity(float vx, float vy, float vz, mavlink_set_velocity_target_local_ned_t &sp)
{
	sp.type_mask=MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx   = vx;
	sp.vy   = vy;
	sp.vz   = vz;
	printf("VELOCITY SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
}
void set_acceleration(float ax, float ay, float az, mavlink_set_acceleration_target_local_ned_t &sp)
{
	sp.type_mask=MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.ax   = ax;
	sp.ay   = ay;
	sp.az   = az;
	//printf("ACCELERATION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.ax, sp.ay, sp.az);
}
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask=sp.type_mask & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
	sp.yaw=yaw;
	printf("POSITION SETPOINT YAW = %.4f\n",sp.yaw);
}

void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask= sp.type_mask & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;
	sp.yaw_rate  = yaw_rate;
}
//void set_roll(float roll, mavlink_set_position_target_local_ned_t &sp)
//{
//	sp.type_mask=sp.type_mask & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ROLL;
//	sp.roll=roll;
//	printf("POSITION SETPOINT ROLL = %.4f\n",sp.roll);
//}
//void set_pitch(float pitch, mavlink_set_position_target_local_ned_t &sp)
//{
//	sp.type_mask=sp.type_mask & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_PITCH;	
//  sp.pitch=pitch;
//	printf("POSITION SETPOINT PITCH = %.4f\n",sp.pitch);
//}
Autopilot_Interface::Autopilot_Interface(Generic_Port *port_)
{
	write_count = 0;
    //Thread and Status related quantiites
	reading_status = 0;      
	writing_status = 0;      
	control_status = 0;
	read_tid  = 0;
	write_tid = 0;

    time_to_exit   = false;
    //ID's initial ID's
	system_id    = 0;
	autopilot_id = 0;
	companion_id = 0;
	buttons      = 0x1111;

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // port management object

}

Autopilot_Interface::~Autopilot_Interface()
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	current_setpoint.data = setpoint;
}

void Autopilot_Interface::read_messages()
{
	bool success;
	bool received_all = false;
	Time_Stamps this_timestamps;
	while (!received_all and !time_to_exit )
	{
        mavlink_message_t message;
		success = port->read_message(message);
        if (success==True)
        {
            current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;
            switch (message.msgid)
			{   
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
                    printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}
				case MAVLINK_MSG_ID_SYS_STATUS:
				{
                    printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
                }
				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				default:
				{
					printf("No message id %i\n",message.msgid);
					break;
				}
			}
		}
    }
    received_all=this_timestamps.heartbeat && this_timestamps.sys_statu;
    if (writing_status>false)
    {
        usleep(100); //Switches Off
    }
}
// Write Messages
int Autopilot_Interface::write_message(mavlink_message_t message)
{
	int len = port->write_message(message);
	write_count++;
	return len;
}
void Autopilot_Interface::write_setpoint()
{
	mavlink_set_position_target_local_ned_t sp;
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		sp = current_setpoint.data;
	}
	if ( not sp.time_boot_ms)
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);
	int len = write_message(message);
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	else
        printf("Message Sent\n");
        printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);
	return;
}

void Autopilot_Interface::enable_offboard_control()
{
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");
		int success = toggle_offboard_control( true );
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
		}
		printf("\n");
	} 
}
void Autopilot_Interface::disable_offboard_control()
{
    if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");
		int success = toggle_offboard_control( false );
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
		}
		printf("\n");
	}

}
int Autopilot_Interface::arm_disarm( bool flag )
{
	if(flag)
	{
		printf("ARM ROTORS\n");
	}
	else
	{
		printf("DISARM ROTORS\n");
	}
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = (float) flag;
	com.param2           = 21196;
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
	int len = port->write_message(message);
	return len;
}
int Autopilot_Interface::move_control(uint16_t x, uint16_t y, uint16_t z, uint16_t r, uint16_t buttons, bool flag)
{
	if(flag)
	{
		printf("MANUAL CONTROL ENABLED\n");
		mavlink_command_long_t com = {0};
	    com.target_system    = system_id;
	    com.target_component = autopilot_id;
	    com.command          = MANUAL_CONTROL;
	    com.confirmation     = true;
	    com.param1           = autopilot_id;
	    com.param2           = x;
	    com.param3           = y;
	    com.param4           = z;
	    com.param5           = buttons;
	    mavlink_message_t message;
	    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
	    int len = port->write_message(message);
	    return len;
	}
	else
	{
		printf("MANUAL CONTROL DISABLED\n");
	}
}
int Autopilot_Interface:: toggle_offboard_control( bool flag )
{
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
	int len = port->write_message(message);
	return len;
}

void Autopilot_Interface::start()
{
	int result;
	if ( !port->is_running() ) // PORT_OPEN
	{
		fprintf(stderr,"ERROR: port not open\n");
		throw 1;
	}
	printf("START READ THREAD \n");
	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if (result) throw result;
	printf("\n");
	printf("CHECK FOR MESSAGES\n");
	while (not current_messages.sysid)
	{
		if (time_to_exit)
			return;
		usleep(500000); // check at 2Hz
	}
	printf("Found\n");
	printf("\n");
	if (not system_id)
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}
	if (not autopilot_id)
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}
	while (not(current_messages.time_stamps.local_position_ned && current_messages.time_stamps.attitude))
	{
		if (time_to_exit)
			return;
		usleep(500000);
	}
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;
	//intial_position.roll      = local_data.attittude.roll;
	//initial_position.pitch    = local_data.attitude.roll;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
    //printf("INITIAL POSITION ROLL = %.4f\n", intial_position.roll);
    //printf("INITIAL POSITION PITCH = %.4f\n", intial_position.pitch);

	printf("\n");
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this);
	if (result) throw result;
	while (not writing_status)
		usleep(100000); // 10Hz
	printf("\n");
	return;
}
void Autopilot_Interface::stop()
{
	printf("CLOSE THREADS\n");
	time_to_exit = true;
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);
	printf("\n");
}

void Autopilot_Interface::start_read_thread()
{
	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}

void Autopilot_Interface::start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}
	else
	{
		write_thread();
		return;
	}
}

void Autopilot_Interface::handle_quit( int sig )
{
	disable_offboard_control();
	try
    {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}
}

void Autopilot_Interface::read_thread()
{
	reading_status = true;
	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}
	reading_status = false;
	return;
}

void Autopilot_Interface::write_thread(void)
{
	writing_status = 2;
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ROLL MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PITCH;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		current_setpoint.data = sp;
	}
	write_setpoint();
	writing_status = true;
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}
	writing_status = false;
	return;
}

void*start_autopilot_interface_read_thread(void *args)
{
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;
	autopilot_interface->start_read_thread();
	return NULL;
}
void*start_autopilot_interface_write_thread(void *args)
{
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;
	autopilot_interface->start_write_thread();
	return NULL;
}