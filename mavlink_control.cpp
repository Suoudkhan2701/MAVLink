#include "mavlink_control.h"
int top (int argc, char **argv)
{
    #ifdef __APPLE__
    char *uart_name = (char*)"/dev/tty.usbmodem1";
    #else
	char *uart_name = (char*)"/dev/ttyUSB0";
    #endif
	int baudrate = 57600;

	bool use_udp = false;
	char *udp_ip = (char*)"127.0.0.1";
	int udp_port = 14540;
	bool autotakeoff = false;

	Generic_Port *port;
	if(use_udp)
	{
		port = new UDP_Port(udp_ip, udp_port);
	}
	else
	{
		port = new Serial_Port(uart_name, baudrate);
	}

	Autopilot_Interface_Addition autopilot_interface(port);

	port_quit = port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	port->start();
	autopilot_interface.start();
	commands(autopilot_interface, autotakeoff);
	autopilot_interface.stop();
	port->stop();
	delete port;
	return 0;

}

void commands(Autopilot_Interface &api, bool autotakeoff)
{
	api.enable_offboard_control();
	usleep(100);
	if(autotakeoff)
	{
		api.arm_disarm(true);
		usleep(100);
	}
	printf("SEND OFFBOARD COMMANDS\n");

	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.MANUAL_CONTROL;

	// Example 1 - Fly up by to 2m
	set_position( ip.x ,       // [m]
			 	  ip.y ,       // [m]
				  ip.z ,       // [m]
				  ip.r ,       // [m]
				  sp         );

	if(autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}
	api.update_setpoint(sp);
	for (int i=0; i < 8; i++)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	}
	// Example 2 - Set Velocity
	//set_velocity( -1.0       , // [m/s]
	//			  -1.0       , // [m/s]
	//			   0.0       , // [m/s]
	//			   sp        );

	// Example 2.1 - Append Yaw Command
	//set_yaw( ip.yaw + 90.0/180.0*M_PI, // [rad]
	//		 sp     );

	// SEND THE COMMAND
	//api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 4 seconds, check position
	//for (int i=0; i < 4; i++)
	//{
	//	mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
	//	printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
	//	sleep(1);
	//}

	//if(autotakeoff)
	//{
		// Example 3 - Land using fixed velocity
		//set_velocity(  0.0       , // [m/s]
					   //0.0       , // [m/s]
					   //1.0       , // [m/s]
					   //sp        );

		//sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;
		//api.update_setpoint(sp);
		//for (int i=0; i < 8; i++)
		//{
			//mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
			//printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
			//sleep(1);
		//}
		//printf("\n");
		//api.arm_disarm(false);
		//usleep(100);
	}
	api.disable_offboard_control();
	printf("READ SOME MESSAGES \n");

	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Got message LOCAL_POSITION_NED\n");
	printf("Position  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU\n");
	printf("Time (Since Boot):     %lu \n", imu.time_usec);
	printf("Accelerometer  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("Gyroscope (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("Mag  (NED):  % f % f % f (Ga)\n", imu.xmag , imu.ymag , imu.zmag );
	printf("Barometer:        %f (mBar) \n",imu.abs_pressure);
	printf("Altitude:    %f (m) \n",imu.pressure_alt);
	printf("Temperature: %f C \n",imu.temperature );
    printf("\n");

    mavlink_attitude_t attitude = messages.attitude;
    printf("Got message ATTITUDE\n");
    printf("Roll: %f (rad)", attitude.roll);
    printf("Yaw: %f (rad)", attitude.yaw);
    printf("Pitch: %f (rad)", attitude.pitch);
    printf("Roll Speed: %f (rad/s)",attitude.rollspeed);
    printf("Yaw Speed: %f (rad/s)",attitude.yawspeed);
    printf("Pitch Speed: %f (rad/s)",attitude.pitchspeed);
    printf("\n");

	return;
}

void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff)
{
	const char *commandline_usage = "usage: mavlink_control [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a ]";
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) 
        {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) 
        {
			if (argc > i + 1) 
            {
				i++;
				uart_name = argv[i];
			} 
            else 
            {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) 
        {
			if (argc > i + 1) 
            {
				i++;
				baudrate = atoi(argv[i]);
			} 
            else 
            {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) 
        {
			if (argc > i + 1) 
            {
				i++;
				udp_ip = argv[i];
				use_udp = true;
			} 
            else 
            {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) 
        {
			if (argc > i + 1) 
            {
				i++;
				udp_port = atoi(argv[i]);
			} 
            else 
            {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) 
        {
			autotakeoff = true;
		}
	}
	return;
}

void quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
	try 
    {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}
	try 
    {
		port_quit->stop();
	}
	catch (int error){}
}

int main(int argc, char **argv)
{
	try
	{
		int result = top(argc,argv);
		return result;
	}
	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}
}