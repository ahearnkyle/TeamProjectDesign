
//#include <checksum.h>
#include <mavlink_types.h>
#include <mavlink.h>
#include <protocol.h>
#include <math.h>

/****IMPORTANT STUFF****/

//TODO probably need a way to read current yaw from drone to align the camera data to the drone's coordinate
//system before sending commands

//TODO need the commands to enable guided mode and enable/disable autonomous mode

/***********************/

/* Controller Specification
* 
* Master States: pickup, drop off
* Sub-States: wait for beacons, position setpoint (0-n), verify pickup or dropoff
* 
* 
*/

/* Variables for Controller State Machine */
const int THRESHOLD_X = 3;//tolerances +/- the given threshold
const int THRESHOLD_Y = 3;
const int THRESHOLD_Z = 1.5;
const int THRESHOLD_Theta = 15;
const int CARRYING_PACKAGE_MIN_CM = 53;
const int CARRYING_PACKAGE_MAX_CM = 57;

const int STATE_INIT = 0;
const int STATE_PU_WAIT = 1;
const int STATE_PU_SETPT = 2;
const int STATE_PU_RAISE = 3;
const int STATE_PU_VERIFY = 4;
const int PU_SETPT_COUNT =  3;
const float PU_X_TOL_CM = 2;// +/- tolerance
const float PU_Y_TOL_CM = 2;
const float PU_Z_TOL_CM = 0.5;
const float PU_YAW_TOL_DEG = 10;
const int PU_SETPTS[PU_SETPT_COUNT][4]={{5,2,70,90},{5,2,53,90},{-9,2,53,90}};

const int STATE_DO_WAIT = 5;
const int STATE_DO_SETPT = 6;
const int STATE_DO_VERIFY = 7;
const int DO_SETPT_COUNT = 4;
const int DO_SETPTS[DO_SETPT_COUNT][4]={{3,4,70,90},{3,4,49,90},{-3,-4,49,90},{-3,-4,70,90}};

int controllerState = STATE_DO_WAIT;
int verifyFailCounter = 0;
int currentSetptIndex = 0;
float objPos[4];
float errs[4];

/*
* Mavlink communication stuff
*/

uint8_t system_id = 100;
uint8_t component_id = MAV_COMP_ID_IMU;
uint8_t type = MAV_TYPE_QUADROTOR; //GCS
uint8_t autopilot = MAV_AUTOPILOT_GENERIC; //generic

uint8_t received_sysid;//51 ;
uint8_t received_compid;// = 68;
uint8_t GCS_UNITS = 0;
int a = 0;

/*
* Camera stuff
*/
//terminology;
//horizontal refers to the wider dimension of the camera (x coord)
//vertical refers to the narrower dimension of the camera (y coord)

//Camera notes:
   //output ranges: x [0-1023], y [0-767] for a 1024 by 768 resolution
    //the blobs merge after about 3m, detectable up to 4 or 5 meters, esp when not near edge of camera frame
    //conversion to angular position:
    // estimate: 18.3 pix/cm at dist of 72.5 cm
    // horizontal fov is about 41 degrees, vertical aobut 31.5 degrees
    // or 23.8 pix/deg vertical
    // and 24.39 horizontal
    // Final Estimate: **24 pixels per degree**
//NOTE: detection range significantly narrower than FOV
//So a beacon at near the edge wont be detected, but can be tracked there if detected earlier

//full frame distance (h) for object to take up full vertical (756 pix) as a function of object length (l)is:
// object must use up 31.5 degrees so l/2 takes up 15.75 degrees
// tan(15.75) = (l/2)/h
// h = l/(2*tan(17.75))
//say l = 7.5 cm, h = 13.3 cm

#include <Wire.h>

const long MIN_LOOP_PERIOD_MILLIS = 2000;
long lastLoopTimeMillis = 0;

/* For IR Camera */ 
const float pixPerDegree = 24;
const float frameSizePix = 768;
float fullFrameDistanceCm = 14.4;//for 8 cm apart beacons

int IRsensorAddress = 0xB0;
//int IRsensorAddress = 0x58;
int slaveAddress;
boolean ledState = false;
byte data_buf[16];
int i;

int Ix[4];
int Iy[4];
int s;

int Ix2[4];
int Iy2[4];

/*
* Helper function used to communicate with the IR camera
*/
void Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

/*
* Reads the connected IR camera by polling.
* Places the beacon locations (in pixels) in the Ix and Iy arrays
*/
void readMasterIRCamera()
{
     //IR sensor read
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);
    Wire.endTransmission();

    Wire.requestFrom(slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
    for (i=0;i<16;i++) { data_buf[i]=0; }
    i=0;
    while(Wire.available() && i < 16) { 
        data_buf[i] = Wire.read();
        i++;
    }

     //read all the coordinates
    Ix[0] = data_buf[1];
    Iy[0] = data_buf[2];
    s   = data_buf[3];
    Ix[0] += (s & 0x30) <<4;
    Iy[0] += (s & 0xC0) <<2;

    Ix[1] = data_buf[4];
    Iy[1] = data_buf[5];
    s   = data_buf[6];
    Ix[1] += (s & 0x30) <<4;
    Iy[1] += (s & 0xC0) <<2;

    Ix[2] = data_buf[7];
    Iy[2] = data_buf[8];
    s   = data_buf[9];
    Ix[2] += (s & 0x30) <<4;
    Iy[2] += (s & 0xC0) <<2;

    Ix[3] = data_buf[10];
    Iy[3] = data_buf[11];
    s   = data_buf[12];
    Ix[3] += (s & 0x30) <<4;
    Iy[3] += (s & 0xC0) <<2;

    //print all the coordinates
   /* Serial.println("coordinates");
    for(i=0; i<4; i++)
    {
      Serial.print( int(Ix[i]) );
      Serial.print(",");
      Serial.print( int(Iy[i]) );
      if (i<3)
        Serial.print(",");
    }
    
    Serial.println(""); */
}

/*
* Reads the connected IR camera by polling from the slave arduino
* connected to serial port 1.
* Places the beacon locations (in pixels) in the Ix and Iy arrays
*/
void readSlaveIRCamera()
{
    //read slave camera
    //while(Serial1.available());
    Serial1.print('0');
    while(Serial1.available() == 0);
    
    for(int i=0;i< 4;i++)
    {
      int value = Serial1.read() | (Serial1.read() << 8);
      Ix2[i] = value;
      value = Serial1.read() | (Serial1.read() << 8);
      Iy2[i] = value;
    }
   /* Serial.println("coordinates2");
    for(i=0; i<4; i++)
    {
      Serial.print( int(Ix2[i]) );
      Serial.print(",");
      Serial.print( int(Iy2[i]) );
      if (i<3)
        Serial.print(",");
    }
    Serial.println();*/
}

/*
* Initializes the IR camera and serial connection to the slave arduino
*/
void setup()
{
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    Serial.begin(19200);  //For user communication
    Serial1.begin(19200); //For slave arduino
    Serial2.begin(57600); //For RXTX from Pixhawk

    Wire.begin();
    // IR sensor initialize
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    
    pinMode(13, OUTPUT);// I have no idea why this line is here...
    analogWrite(13, 150);
    
    delay(100);
    lastLoopTimeMillis = millis();
    Serial.println("Hi");
}
/*
* Returns true on success; false on failure.
* On success, objPos is updated to reflect object location in the order
* [x, y, z, theta] in a right handed coordinate system with z facing down
* and with the origin located at the camera. 
* x, y, z are in centimeters and theta is in degrees.
* Theta is the object's angle (-90, 90] relative to the +x axis.
*/
boolean calculateObjPos(int Ix[], int Iy[])
{
    //use pair of beacons math
    if(Ix[0] != 1023 && Ix[1] != 1023 && Iy[0] != 1023 && Iy[1] != 1023)
    {
      float objDist = fullFrameDistanceCm * frameSizePix/
	sqrt(((long)Iy[0]-Iy[1])*(Iy[0]-Iy[1]) + ((long)Ix[0]-Ix[1])*(Ix[0]-Ix[1]));
      //Serial.print("Object distance (cm): ");Serial.println(objDist);
      //Serial.println(sqrt(((long)Iy[0]-Iy[1])*(Iy[0]-Iy[1]) + ((long)Ix[0]-Ix[1])*(Ix[0]-Ix[1])));
      
      //horizontal distance in frame
      float centerX = (Ix[0]+Ix[1])/2.0f;
      float errXCm = -tan((centerX-1023/2)/pixPerDegree*3.141592/180)*objDist;
      //Serial.print("X Planar Distance (cm): ");Serial.println(errXCm);
       
      //vertical distance in frame
      float centerY = (Iy[0]+Iy[1])/2.0f;
      float errYCm = tan((centerY - 768/2)/pixPerDegree*3.141592/180)*objDist;
      //Serial.print("Y Planar Distance (cm): ");Serial.println(errYCm);
      
      //orientation
      float angX = -(Ix[1] - Ix[0]);
      float angY = Iy[1] - Iy[0];
      //note atan is used instead of atan2
      //this is purposeful since the reported angle should be the same when the 
      //beacons are rotated by 180 degrees
      //The angle is in degrees from the positive x axis
      float angleDeg = atan(angY/angX) * 180/3.141592;
      //Serial.print("Angle: ");Serial.print(angleDeg);Serial.println(" deg.");

      objPos[0] = errXCm;
      objPos[1] = errYCm;
      objPos[2] = objDist;
      objPos[3] = angleDeg;
      return true;    
   }
   return false;
}
/*
* Loop runs through the state machine
*/
void loop()
{
  readMasterIRCamera();
  readSlaveIRCamera();
   mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

   //TODO? Define the system type (see mavlink_types.h for list of possible types)

  // mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, MAV_MODE_GUIDED_DISARMED, 0, MAV_STATE_ACTIVE);
  switch(controllerState)
  {
    ////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////LOOK FOR PACKAGE/////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    case STATE_PU_WAIT:
      Serial.println("Waiting for Package to Pickup");
      //TODO need more sophisticated check??
      if(Ix[0] != 1023 && Ix[1] != 1023 && Iy[0] != 1023 && Iy[1] != 1023)
      {
        Serial.println("Found Package!");
        controllerState = STATE_PU_SETPT;
        currentSetptIndex = 0;
        //TODO need to disable autopilot mission?
        //TODO maybe split this into two states; one where it is acceptable to detect only one
        //beacon because the drone is too high to detect both
        //The first would send a decrease altitude command and the drone goes to the second once both
        //beacons are detected
      }
      break;
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////////PICKUP SEQUENCE///////////////////////////////
    ////////////////////////////////////////////////////////////////////////////    
    case STATE_PU_SETPT:
      //read ir camera
      if(calculateObjPos(Ix, Iy))
      {
        errs[0] = PU_SETPTS[currentSetptIndex][0] - objPos[0];
        errs[1] = PU_SETPTS[currentSetptIndex][1] - objPos[1];
        errs[2] = PU_SETPTS[currentSetptIndex][2] - objPos[2];
        errs[3] = PU_SETPTS[currentSetptIndex][3] - objPos[3];
        if(abs(errs[3] + 180) < abs(errs[3]))//allow angle to wrap between +90/-89.9
          errs[3] = errs[3]+180;
        if(abs(errs[3] - 180) < abs(errs[3]))
          errs[3] = errs[3]-180;

        Serial.print("Setpoint number ");Serial.println(currentSetptIndex);
        Serial.println("Need to move drone:");
        Serial.print("  ");Serial.print(-errs[0]);Serial.println(" cm left");
        Serial.print("  ");Serial.print(-errs[1]);Serial.println(" cm forewards");        
        Serial.print("  ");Serial.print(-errs[2]);Serial.println(" cm downwards");    
        Serial.print("  ");Serial.print(errs[3]);Serial.println(" degrees ccw");  
  
        //if at setpoint (within threshold)
        if(abs(errs[0]) < THRESHOLD_X && abs(errs[1]) < THRESHOLD_Y
          && abs(errs[2]) < THRESHOLD_Z && abs(errs[3]) < THRESHOLD_Theta)
        {
          Serial.print("Arrived at setpoint ");Serial.println(currentSetptIndex);
          currentSetptIndex ++;   
          if(currentSetptIndex == PU_SETPT_COUNT)
            controllerState = STATE_PU_RAISE;
        }
        else
        {
          
          //TODO make sure data is corrected for camera angle relative to compass?
          //mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, &msg, system_id, component_id, MAV_FRAME_LOCAL_NED, -errs[1]/100.0, -errs[0]/100.0, -errs[2]/100.0, (errs[3])*3.1415/180);
          //x, y, z are all in meters because we are using the Ned coordinate system.
          //yaw is in radians
          //MAV_FRAME_LOCAL_NED give the following Z:down,x:north, y:east
          //info about target_system and target_component at http://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html
          //pack the message for the local movement. the message will contain long, lat, and alt so we will need to update it in the loop.
          //mavlink_msg_set_local_position_setpoint_send(comm,system_id, component_id, &msg, MAV_FRAME_LOCAL_NED, 100.0,100.0,100.0,0.0);
          //uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          //Serial2.write(buf, len);
          //receve_msg();
          }
        }
        else
          Serial.println("Camera obstructed");
      //TODO have some sort of watchdog or error check in case calculateObjPos
      //maybe even reset cameera if it doesn't seem to be working right
      //fails too much
      break;
      //TODO insert PU_RAISE state to make drone go up for a few secs before entering PU_VERIFY
    case STATE_PU_RAISE:
    
      Serial.println("Raise drone 30 cm");   
      //mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, &msg, system_id, component_id, MAV_FRAME_LOCAL_NED, 0, 0, -0.3,0);
      controllerState = STATE_PU_VERIFY;
      break;
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////VERIFY PICKUP SUCCESS/////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    case STATE_PU_VERIFY:
      //check whether drone has package at hook height
      if(calculateObjPos(Ix, Iy) && objPos[2] > CARRYING_PACKAGE_MIN_CM 
        && objPos[2] < CARRYING_PACKAGE_MAX_CM)
      {
        //if package
        verifyFailCounter = 0;
        controllerState = STATE_DO_WAIT;
        Serial.println("Pickup Successful!");
      }
      else
      {
        //if no package
        verifyFailCounter+=1;
        controllerState = STATE_PU_WAIT;
        Serial.print("Pickup Failed :( ");Serial.print(verifyFailCounter);Serial.println(" consecutive failures");
      }

      //TODO need to enable autopilot mission?

      break;
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////LOOK FOR LANDING PAD//////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    case STATE_DO_WAIT:
      Serial.println("Waiting for drop off zone");
      //TODO need more sophisticated check??
      if(Ix2[0] != 1023 && Ix2[1] != 1023 && Iy2[0] != 1023 && Iy2[1] != 1023)
      {
        controllerState = STATE_DO_SETPT;
        currentSetptIndex = 0;
        Serial.println("Found drop off zone!");
        //TODO need to disable autopilot mission?
      }
      break;
    ////////////////////////////////////////////////////////////////////////////  
    /////////////////////////////DROP OFF SEQUENCE//////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    case STATE_DO_SETPT:
      //read ir camera
      if(calculateObjPos(Ix2, Iy2))
      {
        errs[0] = DO_SETPTS[currentSetptIndex][0] - objPos[0];
        errs[1] = DO_SETPTS[currentSetptIndex][1] - objPos[1];
        errs[2] = DO_SETPTS[currentSetptIndex][2] - objPos[2];
        errs[3] = DO_SETPTS[currentSetptIndex][3] - objPos[3];
        if(abs(errs[3] + 180) < abs(errs[3]))//allow angle to wrap between +90/-89.9
          errs[3] = errs[3]+180;
        if(abs(errs[3] - 180) < abs(errs[3]))
          errs[3] = errs[3]-180;
          
        Serial.print("Setpoint number ");Serial.println(currentSetptIndex);
        Serial.println("Need to move drone:");
        Serial.print("  ");Serial.print((errs[0]*cos(0.96)+errs[1]*sin(0.96)));Serial.println(" cm left");
        Serial.print("  ");Serial.print(-(errs[0]*sin(.96)-errs[1]*cos(0.96)));Serial.println(" cm forewards");        
        Serial.print("  ");Serial.print(-errs[2]);Serial.println(" cm downwards");    
        Serial.print("  ");Serial.print(errs[3]);Serial.println(" degrees ccw");  

        //if at setpoint (within threshold)
        if(abs(errs[0]) < THRESHOLD_X && abs(errs[1]) < THRESHOLD_Y
          && abs(errs[2]) < THRESHOLD_Z && abs(errs[3]) < THRESHOLD_Theta)
        {
          Serial.print("Arrived at setpoint ");Serial.println(currentSetptIndex);
          currentSetptIndex ++;   
          if(currentSetptIndex == DO_SETPT_COUNT)
            controllerState = STATE_DO_VERIFY;
        }else
        {
          //TODO make sure data is corrected for camera angle relative to compass?
          //mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, &msg, system_id, component_id, MAV_FRAME_LOCAL_NED, errs[0]/100.0, errs[1]/100.0, errs[2]/100.0, errs[3]*3.1415/180);
          //x, y, z are all in meters because we are using the Ned coordinate system.
          //yaw is in radians
          //MAV_FRAME_LOCAL_NED give the following Z:down,x:north, y:east
          //info about target_system and target_component at http://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html
          //pack the message for the local movement. the message will contain long, lat, and alt so we will need to update it in the loop.
          //mavlink_msg_set_local_position_setpoint_send(comm,system_id, component_id, &msg, MAV_FRAME_LOCAL_NED, 100.0,100.0,100.0,0.0);
          //uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          //Serial2.write(buf, len);
          //receve_msg();

        }
      }
      else
        Serial.println("Camera Obstructed");
      //TODO have some sort of watchdog or error check in case calculateObjPos
      //fails too much
      break;
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////VERIFY DROP OFF SUCCESS////////////////////////////
    ////////////////////////////////////////////////////////////////////////////      
    case STATE_DO_VERIFY:
      //check whether drone has package at hook height
      if(calculateObjPos(Ix, Iy) && objPos[2] > CARRYING_PACKAGE_MIN_CM 
        && objPos[2] < CARRYING_PACKAGE_MAX_CM)
      {
        //if package
        verifyFailCounter+=1;
        controllerState = STATE_DO_WAIT;
        Serial.println("Drop off Failed");
      }
      else
      {
        //if no package
        verifyFailCounter = 0;
        controllerState = STATE_PU_WAIT;
        Serial.println("Drop off Successful");
      }

      //TODO need to enable autopilot mission?

      break;
    ////////////////////////////////////////////////////////////////////////////
    ///////////////////////////GO BACK TO THE TOP!//////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    }    

   //delay if needed   
   delay(max(0, MIN_LOOP_PERIOD_MILLIS - (millis()-lastLoopTimeMillis) ));
   lastLoopTimeMillis = millis();
}

////////////////////////////////////////////////////////////////////////////////////////
void receve_msg()
{ //receive data over serial
  // Serial.println(Serial2.available());
  //  Serial.println(" ");
  while (Serial2.available() > 0) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t rec =  Serial2.read();//Show bytes send from the pixhawk
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


