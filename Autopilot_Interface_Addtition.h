#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

#include <generic_port.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>

#include <mavlink.h>
 //Mapping:
 //bit 1: x,
 //bit 2: y,
 //bit 3: z,
 //bit 4: vx,
 //bit 5: vy,
 //bit 6: vz,
 //bit 7: ax,
 //bit 8: ay,
 //bit 9: az,
 //bit 10: is force setpoint,
 //bit 11: yaw,
 //bit 12: yaw rate
 //bit 13: roll
 //bit 14: pitch
 //remaining bits unused
 
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ROLL         0b0010111111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PITCH        0b0001111111111111

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF      0x1000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND         0x2000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER       0x3000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE         0x4000

uint64_t get_time_usec();
void set_position(float x, float y, float z, float r, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);
//void set_roll(float a, mavlink_set_position_target_local_ned_t &sp);
//void set_pitch(float b, mavlink_set_position_target_local_ned_t &sp);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);

struct Time_Stamps
{
	Time_Stamps()
	{reset_timestamps();}
	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
	}
};

struct Mavlink_Messages {

	int sysid;
	int compid;

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

	Time_Stamps time_stamps; //Periodically Checked

	void reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}
};

class Autopilot_Interface{
    public:
    Autopilot_Interface()
    Autopilot_Interface(Generic_Port*port_)
    ~Autopilot_Interface();

    char reading_status;
	char writing_status;
	char control_status;
    uint64_t write_count;

    int system_id;
	int autopilot_id;
	int companion_id;
	int buttons;
	int x;
	int y;
	int z;
	int r;

	Mavlink_Messages current_messages;
	mavlink_set_position_target_local_ned_t initial_position;

	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void read_messages();
	int  write_message(mavlink_message_t message);

	int	 arm_disarm(bool flag);
	void enable_offboard_control();
	void disable_offboard_control();
	int move_control(uint16_t x, uint16_t y, uint16_t z, uint16_t r, uint16_t buttons);

	void start();
	void stop();
	void start_read_thread();
	void start_write_thread(void);
	void handle_quit( int sig );


private:
	Generic_Port *port;
	bool time_to_exit;
	pthread_t read_tid;
	pthread_t write_tid;
	struct {
		std::mutex mutex;
		mavlink_set_position_target_local_ned_t data;
	} current_setpoint;

	void read_thread();
	void write_thread(void);

	int toggle_offboard_control( bool flag );
	void write_setpoint();
};
#endif