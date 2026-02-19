
void request_datastream() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255;       // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2;      // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1;     // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0;  // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x08;  //number of times per second to request the data in hex
  uint8_t _start_stop = 1;            //1 = start, 0 = stop

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial2.write(buf, len);  //Write data to serial port
}




void Heartbeat() {
  //Serial.println("MAVLINK");


  int sysid = 1;
  int compid = 196;
  uint64_t time_usec = 0; /*< Time since system boot*/

  uint8_t frame = 12;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 30;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_GROUND_ROVER;
  // Pack the message

  mavlink_msg_heartbeat_pack(1, 196, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}



void MavLink_receive() {
  //Serial.println("MAVIN");
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial2.available()) {
    uint8_t c = Serial2.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      //Handle new message from autopilot
      switch (msg.msgid) {


        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // #35
          {
            mavlink_rc_channels_raw_t chs;
            mavlink_msg_rc_channels_raw_decode(&msg, &chs);
            Serial.print("Chanel 6 (3-Kanal Schalter): ");
            Serial.println(chs.chan6_raw);
            if (chs.chan8_raw > 1500) { active = 1; }
            if (chs.chan8_raw < 1500) { active = 0; }
          }
          break;

        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:  // #35
          {
            mavlink_named_value_float_t nv;
            mavlink_msg_named_value_float_decode(&msg, &nv);
            if (strcmp(nv.name, "probeX") == 0) remoteProbeX = (int)nv.value;
            if (strcmp(nv.name, "probeY") == 0) remoteProbeY = (int)nv.value;
          }
          break;
      }
    }
  }
}


void MavLink_RC_out() {
  {
    int i;
    int system_type = 250;
    int autopilot_type = MAV_COMP_ID_ALL;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_rc_channels_override_t rcChannels;

    rcChannels.chan4_raw = steering;
    rcChannels.target_system = system_type;
    rcChannels.target_component = autopilot_type;
    mavlink_msg_rc_channels_override_encode(system_type, autopilot_type, &msg, &rcChannels);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf[i]);
  }
}




void Guided() {

  //MAVLINK  MESSAGE
  int sysid = 1;
  int compid = MAV_COMP_ID_PATHPLANNER;
  uint32_t time_boot_ms = 0;
  uint8_t target_system = 1;    /*<System ID of vehicle*/
  uint8_t target_component = 0; /*< Component ID of flight controller or just 0*/
  uint8_t type_mask = 163;      /*  Use Yaw Rate + Throttle : 0b10100011 / 0xA3 / 163 (decimal)   Use Attitude + Throttle: 0b00100111 / 0x27 / 39 (decimal)*/
  float q = (1000);             /*< Attitude quaternion (w, x, y, z order, zero-rotation is {1, 0, 0, 0})Note that zero-rotation causes vehicle to point North. */
  float body_roll_rate = 0;     /*< Body roll rate not supported*/
  float body_pitch_rate = 0;    /*< Body pitch rate not supporte*/
  float thrust = 0.2;           /*< 0=throttle 0%, +1=forward at WP_SPEED, -1=backwards at WP_SPEED*/
  float thrust_body = (000);
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, time_boot_ms, target_system, target_component, type_mask, 0000, body_roll_rate, body_pitch_rate, guidedangle, thrust, 000);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}
