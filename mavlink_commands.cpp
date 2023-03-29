#include "mavlink_commands.hpp"

MAVLink::MAVLink(const int& baud_rate, const uint8_t& rx, const uint8_t& tx) {
  Serial.println("Initializing mavlink connection\n");
  Serial2.begin(baud_rate, SERIAL_8N1, rx, tx);

  this->req_data_stream();

  this->timeout(2);

  this->req_data(MAVLINK_MSG_ID_HOME_POSITION);
}

MAVLink::~MAVLink() {}

uint8_t MAVLink::get_px_mode(){
  return this->px_mode;
}

uint8_t MAVLink::get_px_status(){
  return this->px_status;
}

uint8_t MAVLink::get_battery_status(){
  return this->battery_status;
}

uint16_t MAVLink::get_mis_reached(){
  return this->reached;
}

bool MAVLink::get_armed(){
  return this->armed;
}

std::array<int32_t, 2> MAVLink::get_home_pos_curr(){
  return this->home_pos;
} 

std::array<int32_t, 3> MAVLink::get_global_pos_curr(){
  return this->global_pos_curr;
}

std::array<float, 3> MAVLink::get_velocity_curr(){
  return this->velocity_curr;
}

float MAVLink::get_time_boot(){
  return this->time_boot_sec;
}

float MAVLink::get_yaw_curr(){
  return this->yaw_curr;
}

void MAVLink::set_fly_alt(const float& hgt){
  this->fly_alt = hgt;
}

void MAVLink::add_waypoint(const float& lat, const float& lng){
  Serial.printf("Added waypoint %f %f %f to list\n", lat, lng, this->fly_alt);
  this->waypoints.push_back(std::make_tuple(lat, lng, this->fly_alt));
}

void MAVLink::add_waypoint(const float& lat, const float& lng, const float& hgt){
  Serial.printf("Added waypoint %f %f %f to list\n", lat, lng, hgt);
  this->waypoints.push_back(std::make_tuple(lat, lng, hgt));
}

void MAVLink::send_heartbeat(){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    this->sys_id,
    this->comp_id,
    &msg,
    MAV_TYPE_GCS,
    MAV_AUTOPILOT_INVALID,
    MAV_MODE_MANUAL_ARMED,
    0,
    MAV_STATE_ACTIVE
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial2.write(buf, len);
}

void MAVLink::req_data_stream(){
  uint8_t req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t req_msg_rate = 0x01; // 1 times per second
  uint8_t start_stop = 1; // 1 start, 0 = stop

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(
    this->sys_id, 
    this->comp_id, 
    &msg, 
    this->tgt_sys, 
    this->tgt_comp, 
    req_stream_id, 
    req_msg_rate, 
    start_stop
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial2.write(buf, len);
}

void MAVLink::req_data(const uint16_t& msg_id){

  Serial.printf("Requesting data %u \n", msg_id);
  uint16_t command = 512;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
    this->sys_id,
    this->comp_id,
    &msg, 
    this->tgt_sys,
    this->tgt_comp,
    command,
    0,
    msg_id,
    0, 0, 0, 0, 0,
    2
    );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial2.write(buf, len);
}

void MAVLink::read_data(){
  mavlink_message_t msg;
  mavlink_status_t status;

  if(Serial2.available()){
    uint8_t c = Serial2.read();

    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){

      //Handle new message from autopilot
      switch(msg.msgid){
        case MAVLINK_MSG_ID_HEARTBEAT:
          this->parse_heartbeat(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
          this->parse_mission_request_int(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
          this->parse_mission_request(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_ACK:
          this->parse_mission_ack(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
          this->parse_mission_item_reached(&msg);
          break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
          this->parse_command_ack(&msg);
          break;
        case MAVLINK_MSG_ID_SYS_STATUS:
          this->parse_sys_status(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
          this->parse_mission_count(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
          this->parse_mission_item(&msg);
          break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          this->parse_global_pos(&msg);
          break;
        case MAVLINK_MSG_ID_HOME_POSITION:
          this->parse_home_position(&msg);
          break;
      }
    }
  }
}

void MAVLink::parse_heartbeat(mavlink_message_t* msg){
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(msg, &hb);
  if(this->px_mode != hb.base_mode || this->px_status != hb.system_status){
    this->px_mode = hb.base_mode;
    this->px_status = hb.system_status;
    this->armed = (this->px_mode >= MAV_MODE_FLAG_SAFETY_ARMED);
    Serial.printf("Heartbeat detected\nMode : %u\nSystem : %u\n", this->px_mode, this->px_status);
  }
}

void MAVLink::parse_command_ack(mavlink_message_t* msg){
  mavlink_command_ack_t cmd_ack;
  mavlink_msg_command_ack_decode(msg, &cmd_ack);
  Serial.printf("Command %u result code %u\n", cmd_ack.command, cmd_ack.result);
}

void MAVLink::parse_mission_request_int(mavlink_message_t* msg){
  mavlink_mission_request_int_t mis_req;
  mavlink_msg_mission_request_int_decode(msg, &mis_req);
  this->mis_seq = mis_req.seq;
  Serial.printf("Requesting for mission type %u sequence %u\n", mis_req.mission_type, this->mis_seq);
  if(this->mis_seq == 0){ 
    this->takeoff();
  }
  else if(this->mis_seq == this->mis_count - 1){
    this->return_to_launch();
  }
  else{
    this->send_mission_item();
  }
}

void MAVLink::parse_mission_request(mavlink_message_t* msg){
  mavlink_mission_request_t mis_req;
  mavlink_msg_mission_request_decode(msg, &mis_req);
  this->mis_seq = mis_req.seq;
  Serial.printf("Requesting for mission type %u sequence %u\n", mis_req.mission_type, this->mis_seq);
  if(this->mis_seq == 0){ 
    this->takeoff();
  }
  else if(this->mis_seq == this->mis_count - 1){
    this->return_to_launch();
  }
  else{
    this->send_mission_item();
  }
}

void MAVLink::parse_mission_item_reached(mavlink_message_t* msg){
  mavlink_mission_item_reached_t it;
  mavlink_msg_mission_item_reached_decode(msg, &it);
  if(this->reached != it.seq){
    Serial.printf("Mission sequence %u reached\n", it.seq);
    this->reached = it.seq;
    if(this->reached > 0 && this->global_pos_curr[2] > 2){
      if((this->velocity_curr[0] < 1 && 
          this->velocity_curr[1] < 1 && 
          this->velocity_curr[2] < 1)){
        this->set_servo(4, 2000);
      }else{
        this->set_servo(4, 0);
      }
    }
    if(this->reached == this->mis_count - 2){
      this->waypoints.clear();
    }
  }
}

void MAVLink::parse_mission_ack(mavlink_message_t* msg){
  mavlink_mission_ack_t mis_ack;
  mavlink_msg_mission_ack_decode(msg, &mis_ack);
  if(mis_ack.type == MAV_MISSION_ACCEPTED && mis_ack.mission_type != 255){
    Serial.printf("Mission accepted. Starting mission\n");
    this->reached = NAN;
  }
  else if(mis_ack.type == MAV_MISSION_ACCEPTED && mis_ack.mission_type == 255){
    Serial.printf("Cleared all missions\n");
  }
  else{
    Serial.printf("Mission unaccepted with enum %u\n", mis_ack.type);
  }
}

void MAVLink::parse_sys_status(mavlink_message_t* msg){
  mavlink_sys_status_t sys_status;
  mavlink_msg_sys_status_decode(msg, &sys_status);
  this->battery_status = (int)(sys_status.battery_remaining / 2.55);
}

void MAVLink::parse_global_pos(mavlink_message_t* msg){
  mavlink_global_position_int_t global_pos;
  mavlink_msg_global_position_int_decode(msg, &global_pos);
  this->global_pos_curr[0] = global_pos.lat;
  this->global_pos_curr[1] = global_pos.lon;
  this->global_pos_curr[2] = global_pos.relative_alt;
  this->velocity_curr[0] = global_pos.vx / 1e2;
  this->velocity_curr[1] = global_pos.vy / 1e2;
  this->velocity_curr[2] = global_pos.vz / 1e2;
  this->time_boot_sec = global_pos.time_boot_ms / 1e3;
  this->yaw_curr = global_pos.hdg / 1e2;
}

void MAVLink::parse_mission_status(mavlink_message_t* msg){
  mavlink_mission_current_t mis_stat;
  mavlink_msg_mission_current_decode(msg, &mis_stat);
  switch (mis_stat.mission_state){
    case MISSION_STATE_COMPLETE:
      Serial.printf("Mission completed\n");
      this->return_to_launch();
      break;
    case MISSION_STATE_NO_MISSION:
      Serial.printf("No mission uploaded\n");
      break;
    case MISSION_STATE_NOT_STARTED:
      Serial.printf("Mission uploaded but not started\n");
      break;
    case MISSION_STATE_PAUSED:
      Serial.printf("Mission paused at waypoint %u out of %u\n", mis_stat.seq, mis_stat.total);
      break;
    case MISSION_STATE_ACTIVE:
      Serial.printf("Mission active on the way to waypoint %u out of %u\n", mis_stat.seq, mis_stat.total);
      break;
    default:
      Serial.printf("Unknown mission status\n");
      break;
  }
}

void MAVLink::parse_mission_count(mavlink_message_t* msg){
  mavlink_mission_count_t recv_mis_count;
  mavlink_msg_mission_count_decode(msg, &recv_mis_count);
  if(recv_mis_count.count != this->mis_count){
    Serial.printf("Downloaded mission count is not the same as sent\n");
  }else{
    Serial.printf("Downloading %u missions\n", recv_mis_count.count);
    this->req_mission_item();
  }
}

void MAVLink::parse_mission_item(mavlink_message_t* msg){
  mavlink_mission_item_int_t recv_mis;
  mavlink_msg_mission_item_int_decode(msg, &recv_mis);
  Serial.printf("Downloaded waypoint %u {\n\tlatitude  : %f\n\tlongitude : %f\n\theight    : %f\n}\n\n", recv_mis.seq, recv_mis.x / 1e7, recv_mis.y / 1e7, recv_mis.z);
  if(recv_mis.seq != this->mis_count - 1) this->req_mission_item();
  else{
    this->send_mission_ack();
  } 
}

void MAVLink::parse_home_position(mavlink_message_t* msg){
  mavlink_home_position_t home_pos;
  mavlink_msg_home_position_decode(msg, &home_pos);
  if(home_pos.latitude != this->home_pos[0] || home_pos.longitude != this->home_pos[1]){
    Serial.printf("Home position: %f %f\n", home_pos.latitude / 1e7, home_pos.longitude / 1e7);
    this->home_pos[0] = home_pos.latitude;
    this->home_pos[1] = home_pos.longitude;
  }
}

void MAVLink::run_prearm_checks(){
  Serial.printf("Running prearm checks\n");

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t command = 401;
  uint8_t conf = 0;

  mavlink_msg_command_long_pack(
    this->sys_id, 
    this->comp_id, 
    &msg, 
    this->tgt_sys, 
    this->tgt_comp, 
    command, 
    conf, 
    0, 0, 0, 0, 0, 0, 0
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::arm_disarm(const bool& arm){
  if(arm) Serial.printf("Arming\n"); else Serial.printf("Disarming\n");
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t command = 400; //arm disarm
  uint8_t conf = 0;
  float param1 = (float)arm;

  mavlink_msg_command_long_pack(
    this->sys_id, 
    this->comp_id, 
    &msg, 
    this->tgt_sys, 
    this->tgt_comp, 
    command, 
    conf, 
    param1, 
    0, 0, 0, 0, 0, 0
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::takeoff(){
  this->takeoff(this->fly_alt);
}

void MAVLink::takeoff(const float& height){ 
  Serial.printf("Waypoint %d (takeoff) set as latitude : %f, longitude : %f, height : %f\n", this->mis_seq + 1, this->home_pos[0] / 1e7, this->home_pos[1] / 1e7, height);
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  this->fly_alt = height;

  uint16_t command = 22; //takeoff
  uint8_t conf = 0;
  float param7 = height;

  mavlink_msg_mission_item_int_pack(
    this->sys_id, 
    this->comp_id,
    &msg, 
    this->tgt_sys, 
    this->tgt_comp,
    this->mis_seq,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
    command,
    1,
    1,  
    0, 0, 0, 0,
    this->home_pos[0], // Home position latitude
    this->home_pos[1], // Home position longitude
    param7,
    MAV_MISSION_TYPE_MISSION
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::land(){
  Serial.printf("Landing waypoint sent\n");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t command = 21; //land

  mavlink_msg_mission_item_int_pack(
    this->sys_id, 
    this->comp_id,
    &msg, 
    this->tgt_sys, 
    this->tgt_comp,
    this->mis_seq,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
    command,
    0,
    1,  
    0, 0, 0, 0, 0, 0, 0,
    MAV_MISSION_TYPE_MISSION
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::loiter_time(const uint16_t& time){
  this->loiter_time(time, 
    std::get<0>(this->waypoints.at(this->mis_seq - 1)), 
    std::get<0>(this->waypoints.at(this->mis_seq - 1)), 
    std::get<0>(this->waypoints.at(this->mis_seq - 1))
  );
}

void MAVLink::loiter_time(const uint16_t& time, const float& lat, const float& longitude, const float& alt){
  Serial.printf("Sending loiter mission for %u seconds", time);
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t command = 19; //loiter time
  uint8_t conf = 0;
  float param1 = float(time);

  mavlink_msg_mission_item_int_pack(
    this->sys_id, 
    this->comp_id,
    &msg, 
    this->tgt_sys, 
    this->tgt_comp,
    this->mis_seq,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    command,
    0, 1,
    (float) time,
    0,
    0,
    0,
    lat,
    longitude,
    alt,
    MAV_MISSION_TYPE_MISSION
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::set_servo(const uint8_t& port, const uint16_t& pwm){
  Serial.printf("Setting servo");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t command = 183; //do set serov
  uint8_t conf = 0;
  float param1 = port;
  float param2 = 2000;

  mavlink_msg_command_long_pack(
    this->sys_id, 
    this->comp_id, 
    &msg, 
    this->tgt_sys, 
    this->tgt_comp, 
    command, 
    conf, 
    param1, 
    param2, 0, 0, 0, 0, 0
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial2.write(buf, len);
}

void MAVLink::set_mode(const uint16_t& mode){
  Serial.printf("Setting mode to %u\n", mode);
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t command = 176; //do set mode
  uint8_t conf = 0;
  float param1 = float(mode); //auto disarmed

  mavlink_msg_command_long_pack(
    this->sys_id, 
    this->comp_id, 
    &msg, 
    this->tgt_sys, 
    this->tgt_comp, 
    command, 
    conf, 
    param1, 
    0, 0, 0, 0, 0, 0
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial2.write(buf, len);
}

void MAVLink::return_to_launch(){
  Serial.printf("Waypoint %u (return to launch) set\n");

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint16_t command = 20; //return to launch

  mavlink_msg_mission_item_int_pack(
    this->sys_id, 
    this->comp_id,
    &msg, 
    this->tgt_sys, 
    this->tgt_comp,
    this->mis_seq,
    MAV_FRAME_MISSION, 
    command,
    0,
    1,  
    0, 0, 0, 0, 0, 0, 0,
    MAV_MISSION_TYPE_MISSION
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::send_mission(const uint16_t& num_of_mission){
  // this->clear_all_mission();

  if(num_of_mission != 0){
    this->mis_count = num_of_mission;
  }else{
    this->mis_count = this->waypoints.size() + 2;
  }

  Serial.printf("Sending mission count: %u\n", this->mis_count);
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  this->mis_seq = 0;

  mavlink_msg_mission_count_pack(
    this->sys_id, 
    this->comp_id, 
    &msg, 
    this->tgt_sys, 
    this->tgt_comp, 
    this->mis_count, 
    MAV_MISSION_TYPE_MISSION
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::send_mission_item(const float& hold_time){
  float lat = std::get<0>(this->waypoints.at(this->mis_seq - 1));
  float lng = std::get<1>(this->waypoints.at(this->mis_seq - 1));
  float hgt = std::get<2>(this->waypoints.at(this->mis_seq - 1));

  Serial.printf("Setting waypoint lat : %f, lng : %f, height : %f\n", lat, lng, hgt);

  int32_t lat_send = static_cast<int32_t>(lat * 1e7);
  int32_t lng_send = static_cast<int32_t>(lng * 1e7);

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint8_t command = 16; //waypoint
  uint8_t mission_type = MAV_MISSION_TYPE_MISSION;

  mavlink_msg_mission_item_int_pack(
    this->sys_id, 
    this->comp_id, 
    &msg, 
    this->tgt_sys, 
    this->tgt_comp, 
    this->mis_seq,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
    command, 
    0, 
    1, 
    hold_time,
    0,
    0,
    0, 
    lat_send, 
    lng_send, 
    hgt, 
    mission_type
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);

}

void MAVLink::clear_all_mission(){
  Serial.printf("Clearing all mission\n");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_clear_all_pack(
    this->sys_id,
    this->comp_id,
    &msg,
    this->tgt_sys,
    this->tgt_comp,
    MAV_MISSION_TYPE_ALL
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);
}

void MAVLink::req_mission_list(){
  Serial.printf("Downloading mission from pixhawk\n");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  this->mis_seq = 0;

  mavlink_msg_mission_request_list_pack(
    this->sys_id,
    this->comp_id,
    &msg,
    this->tgt_sys,
    this->tgt_comp,
    MAV_MISSION_TYPE_MISSION
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);

}

void MAVLink::req_mission_item(){
  Serial.printf("Requesting mission item\n");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_request_int_pack(
    this->sys_id,
    this->comp_id,
    &msg,
    this->tgt_sys,
    this->tgt_comp,
    this->mis_seq,
    MAV_MISSION_TYPE_MISSION
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);

  this->mis_seq++;
}

void MAVLink::send_mission_ack(){
  Serial.printf("Mission downloading finished\n");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_ack_pack(
    this->sys_id,
    this->comp_id,
    &msg,
    this->tgt_sys,
    this->tgt_comp,
    MAV_MISSION_ACCEPTED,
    MAV_MISSION_TYPE_MISSION
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial2.write(buf, len);

  Serial.printf("Starting mission\n");

  this->mis_seq = 0;
  this->arm_disarm(true);

  this->timeout(2);

  this->set_mode(MAV_MODE_AUTO_ARMED);
}

void MAVLink::start_mission(){
  this->req_mission_list();
}

void MAVLink::timeout(const uint32_t& duration){
  //time now millisecond
  unsigned long start = millis();
  while(millis() < start + duration * 1000);
}
