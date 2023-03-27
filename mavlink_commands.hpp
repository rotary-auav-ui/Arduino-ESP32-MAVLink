#ifndef MAVCPP_HPP_
#define MAVCPP_HPP_

#include "common/mavlink.h"

#include <Stream.h>
#include <time.h>

#include <vector>
#include <array>
#include <tuple>

class MAVLink{
  public :
    // should be changeable
    uint8_t tgt_sys = 1; // id of pxhawk = 1
    uint8_t tgt_comp = 0; // 0 broadcast, 1 work juga

    uint16_t mis_count;

    // Setup serial communication
    /*
    1st style:
    #include "HardwareSerial.h" or "SoftwareSerial.h"

    std::shared_ptr<MAVLink> mavlink;

    void setup() {
      Serial.begin(baudrate, serial type, rx, tx);
      mavlink = std::make_shared<MAVLink> mavlink(Serial, Serial2);
    }
    */

    // comm is serial for mavlink write and read data, and logger is serial for logging purposes
    MAVLink(Stream *comm_, Stream *logger_ = nullptr);

    ~MAVLink();

    uint8_t get_px_mode();

    uint8_t get_px_status();

    uint8_t get_battery_status();

    uint16_t get_mis_reached();

    std::array<float, 3> get_global_pos_curr();

    std::array<float, 3> get_velocity_curr();

    float get_time_boot();

    float get_yaw_curr();

    uint16_t get_mis_seq();

    bool get_mis_req_status();

    bool get_armed();

    void set_fly_alt(const float& hgt);

    // Overload waypoint to use default height
    void add_waypoint(const float& lat, const float& lng);

    void add_waypoint(float lat, float lng, float hgt);

    void send_heartbeat();

    // Set data requests from pixhawk
    void req_data_stream();

    void req_data(uint16_t msg_id);

    // Read data from pixhawk via UART2
    void read_data();

    void takeoff();

    // Takeoff
    void takeoff(const float& height);

    // Land
    void land();

    // Hold position for a certain amount of time
    void loiter_time(const uint16_t& time, const float& lat, const float& longitude, const float& alt);
    
    // Set mode (use MAV_MODE enum for parameter)
    void set_mode(const uint16_t& mode);

    // Return to launch position (without land)
    void return_to_launch();

    // Send mission count (needed for pixhawk to start requesting mission)
    void send_mission(const uint16_t& num_of_mission = 0);

    // Clear All Mission
    void clear_all_mission();

    // Starts mission
    void start_mission();

    // Sends 1 mission item
    void send_mission_item(const float& hold_time = 10);

    // Arms or disarms the drone (true == arm, false == disarm)
    void arm_disarm(const bool& arm);

    void timeout(const double& duration);

    void set_servo(uint8_t port, uint16_t pwm);

  private :
    // buffer data for mavlink. no need to reinit every function
    uint8_t buf[MAVLINK_MAX_PACKET_LEN]; // send data buffer
    uint16_t len; // send data buffer length
    uint8_t recv; // recieve data buffer

    Stream *comm;
    Stream *logger;

    uint8_t px_mode = 0;
    uint8_t px_status = 0;

    uint8_t sys_id = 255; // GCS id
    uint8_t comp_id = 2; // any?
    uint8_t mis_status;
    uint8_t battery_status;
    uint16_t reached;
    uint16_t mis_seq;
    std::array<int32_t, 2> home_pos;
    std::array<float, 3> global_pos_curr; //lat, long, relative alt
    std::array<float, 3> velocity_curr; // velocity north, velocity east, velocity down
    uint16_t yaw_curr;
    float time_boot_sec;
    float fly_alt = 5;
    bool req_mis;
    bool mission_valid = false;
    bool armed = false;
    std::vector<std::tuple<float, float, float>> waypoints;

    // Check pixhawk current mode
    void parse_heartbeat(mavlink_message_t* msg);

    // Accept a mission request (int32) and send a mission item
    void parse_mission_request_int(mavlink_message_t* msg);

    // Accept a mission reqeuest (float) and send a mission item
    void parse_mission_request(mavlink_message_t* msg);

    // Check mission items reached
    void parse_mission_item_reached(mavlink_message_t* msg);

    // Check whether uploaded mission is accepted
    void parse_mission_ack(mavlink_message_t* msg);

    // Check whether command was successfuly accepted
    void parse_command_ack(mavlink_message_t* msg);

    // Results of prearm checks
    void parse_sys_status(mavlink_message_t* msg);

    // Receive current global pos and velocity
    void parse_global_pos(mavlink_message_t * msg);

    // Status of currently run mission
    void parse_mission_current(mavlink_message_t* msg);

    // Get downloaded mission count
    void parse_mission_count(mavlink_message_t* msg);

    // Get downloaded missions
    void parse_mission_item(mavlink_message_t* msg);

    void parse_home_position(mavlink_message_t* msg);

    // Run pre-arm checks
    void run_prearm_checks();

    // Downloads mission from pixhawk
    void req_mission_list();

    // Requests a mission item
    void req_mission_item();

    // Sends mission download acknowledgement
    void send_mission_ack();

    // log serial purposes
    void log_printf(const char *print, ...)  __attribute__ ((format (printf, 2, 3)));

    double get_clock();
};

#endif // MAVCPP_HPP_
