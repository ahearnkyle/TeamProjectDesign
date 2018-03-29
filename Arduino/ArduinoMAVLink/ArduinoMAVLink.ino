#include <checksum.h>
#include <mavlink_types.h>
#include <mavlink.h>
#include <protocol.h>
#include <math.h>
//https://pixhawk.ethz.ch/mavlink/


uint8_t system_id = 100;
uint8_t component_id = MAV_COMP_ID_IMU;
uint8_t type = MAV_TYPE_QUADROTOR; //GCS
uint8_t autopilot = MAV_AUTOPILOT_GENERIC; //generic

uint8_t received_sysid;//51 ;
uint8_t received_compid;// = 68;
uint8_t GCS_UNITS = 0;
int a = 0;

void setup() {
  Serial1.begin(57600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
}

void loop() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Define the system type (see mavlink_types.h for list of possible types)

  // mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, MAV_MODE_GUIDED_DISARMED, 0, MAV_STATE_ACTIVE);
  mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, &msg, system_id, component_id, MAV_FRAME_LOCAL_NED, 100.0, 100.0, 100.0, 0.0);
  //x, y, z are all in meters because we are using the Ned coordinate system.
  //MAV_FRAME_LOCAL_NED give the following Z:down,x:north, y:east
  //info about target_system and target_component at http://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html
  //pack the message for the local movement. the message will contain long, lat, and alt so we will need to update it in the loop.
  //mavlink_msg_set_local_position_setpoint_send(comm,system_id, component_id, &msg, MAV_FRAME_LOCAL_NED, 100.0,100.0,100.0,0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  delay(1000);
  Serial1.write(buf, len);
  receve_msg();
}
////////////////////////////////////////////////////////////////////////////////////////
void receve_msg()
{ //receive data over serial
  // Serial.println(Serial1.available());
  //  Serial.println(" ");
  while (Serial1.available() > 0) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t rec =  Serial1.read();//Show bytes send from the pixhawk
    //Serial.print("len = ");
    //Serial.print(" Data = ");
    //Serial.println(rec);
    if (mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status)) {

      //Serial.println(" ");
      //Serial.print("Mensaje: ");
      //Serial.println(msg.msgid);
      //mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, received_sysid, received_compid, 63 , 1000, 1);
      //mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, &msg, system_id, component_id, MAV_FRAME_LOCAL_NED, 100.0, 100.0, 100.0, 0.0);
      gcs_handleMessage(&msg);
    }
  }
}

void gcs_handleMessage(mavlink_message_t* msg) //read varaible
{

  //  Serial.println();
  Serial.print("Message ID: ");
  Serial.println(msg->msgid);
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT:
      mavlink_set_local_position_setpoint_t locationData;
      Serial.write("I got a local position point set\n");
      mavlink_msg_set_local_position_setpoint_decode(msg, &locationData);
      Serial.println(locationData.x);
      Serial.println(locationData.y);
      Serial.println(locationData.z);
      Serial.println(locationData.yaw);
      Serial.println(locationData.target_system);
      Serial.println(locationData.target_component);
      Serial.println(locationData.coordinate_frame);

      //Im thinking we need to print to the serial the to make sure that we have send the message and it was retrieved.
      //this case will be able to execute local position commands with the long, lat, and alt.
      break;
  }
 

}

