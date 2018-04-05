/* Controller Specification
* 
* Master States: pickup, drop off
* Sub-States: wait for beacons, position setpoint (0-n), verify pickup or dropoff
* 
* 
*/

/* Variables for Controller State Machine */
const int THRESHOLD_X = 2;
const int THRESHOLD_Y = 2;
const int THRESHOLD_Z = 2;
const int THRESHOLD_Theta = 2;
const int CARRYING_PACKAGE_MIN_CM = 30;
const int CARRYING_PACKAGE_MAX_CM = 50;

const int STATE_INIT = 0;

const int STATE_PU_WAIT = 1;
const int STATE_PU_SETPT = 2;
const int STATE_PU_VERIFY = 3;
const int PU_SETPT_COUNT = 4;
const int PU_SETPTS[PU_SETPT_COUNT][4]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

const int STATE_DO_WAIT = 4;
const int STATE_DO_SETPT = 5;
const int STATE_DO_VERIFY = 6;
const int DO_SETPT_COUNT = 4;
const int DO_SETPTS[DO_SETPT_COUNT][4]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};

int controllerState = STATE_PU_WAIT;
int verifyFailCounter = 0;
int currentSetptIndex = 0;
float objPos[4];
float errs[4];

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

const long MIN_LOOP_PERIOD_MILLIS = 20;
long lastLoopTimeMillis = 0;

/* For IR Camera */ 
const float pixPerDegree = 24;
const float frameSizePix = 768;
float fullFrameDistanceCm = 13.3;

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
    Serial.println("coordinates");
    for(i=0; i<4; i++)
    {
      Serial.print( int(Ix[i]) );
      Serial.print(",");
      Serial.print( int(Iy[i]) );
      if (i<3)
        Serial.print(",");
    }
    
    Serial.println(""); 
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
    Serial.println("coordinates2");
    for(i=0; i<4; i++)
    {
      Serial.print( int(Ix2[i]) );
      Serial.print(",");
      Serial.print( int(Iy2[i]) );
      if (i<3)
        Serial.print(",");
    }
    Serial.println();
}

/*
* Initializes the IR camera and serial connection to the slave arduino
*/
void setup()
{
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    Serial.begin(19200);
    Serial1.begin(19200);
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
      Serial.print("Object distance (cm): ");Serial.println(objDist);
      Serial.println(sqrt(((long)Iy[0]-Iy[1])*(Iy[0]-Iy[1]) + ((long)Ix[0]-Ix[1])*(Ix[0]-Ix[1])));
      
      //horizontal distance in frame
      float centerX = (Ix[0]+Ix[1])/2.0f;
      float errXCm = -tan((centerX-1023/2)/pixPerDegree*3.141592/180)*objDist;
      Serial.print("X Planar Distance (cm): ");Serial.println(errXCm);
       
      //vertical distance in frame
      float centerY = (Iy[0]+Iy[1])/2.0f;
      float errYCm = tan((centerY - 768/2)/pixPerDegree*3.141592/180)*objDist;
      Serial.print("Y Planar Distance (cm): ");Serial.println(errYCm);
      
      //orientation
      float angX = -(Ix[1] - Ix[0]);
      float angY = Iy[1] - Iy[0];
      //note atan is used instead of atan2
      //this is purposeful since the reported angle should be the same when the 
      //beacons are rotated by 180 degrees
      //The angle is in degrees from the positive x axis
      float angleDeg = atan(angY/angX) * 180/3.141592;
      Serial.print("Angle: ");Serial.print(angleDeg);Serial.println(" deg.");

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
  
  switch(controllerState)
  {
    ////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////LOOK FOR PACKAGE/////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    case STATE_PU_WAIT:
      //TODO need more sophisticated check??
      if(Ix[0] != 1023 && Ix[1] != 1023 && Iy[0] != 1023 && Iy[1] != 1023)
      {
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

        //TODO construct and send command to pixhawk
        //make sure units are right
        //need to command position or velocity??
        
        //if at setpoint (within threshold)
        if(abs(errs[0]) < THRESHOLD_X && abs(errs[1]) < THRESHOLD_Y
          && abs(errs[2]) < THRESHOLD_Z && abs(errs[3]) < THRESHOLD_Theta)
        {
          currentSetptIndex ++;   
          if(currentSetptIndex == PU_SETPT_COUNT)
            controllerState = STATE_PU_VERIFY;
        }
      }
      //TODO have some sort of watchdog or error check in case calculateObjPos
      //maybe even reset cameera if it doesn't seem to be working right
      //fails too much
      break;
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////VERIFY PICKUP SUCCESS/////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    case STATE_PU_VERIFY:
      //check whether drone has package at hook height
      if(calculateObjPos(Ix, Iy) && objPos[2] > CARRYING_PACKAGE_MIN_CM 
        && objPos[2] > CARRYING_PACKAGE_MAX_CM)
      {
        //if package
        verifyFailCounter = 0;
        controllerState = STATE_DO_WAIT;
      }
      else
      {
        //if no package
        verifyFailCounter+=1;
        controllerState = STATE_PU_WAIT;
      }
      break;
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////LOOK FOR LANDING PAD//////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    case STATE_DO_WAIT:
      //TODO need more sophisticated check??
      if(Ix2[0] != 1023 && Ix2[1] != 1023 && Iy2[0] != 1023 && Iy2[1] != 1023)
      {
        controllerState = STATE_DO_SETPT;
        currentSetptIndex = 0;
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
        errs[0] = PU_SETPTS[currentSetptIndex][0] - objPos[0];
        errs[1] = PU_SETPTS[currentSetptIndex][1] - objPos[1];
        errs[2] = PU_SETPTS[currentSetptIndex][2] - objPos[2];
        errs[3] = PU_SETPTS[currentSetptIndex][3] - objPos[3];

        //TODO send command to pixhawk
        
         //if at setpoint (within threshold)
        if(abs(errs[0]) < THRESHOLD_X && abs(errs[1]) < THRESHOLD_Y
          && abs(errs[2]) < THRESHOLD_Z && abs(errs[3]) < THRESHOLD_Theta)
        {
          currentSetptIndex ++;   
          if(currentSetptIndex == DO_SETPT_COUNT)
            controllerState = STATE_DO_VERIFY;
        }
      }
      //TODO have some sort of watchdog or error check in case calculateObjPos
      //fails too much

      break;
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////VERIFY DROP OFF SUCCESS////////////////////////////
    ////////////////////////////////////////////////////////////////////////////      
    case STATE_DO_VERIFY:
      //check whether drone has package at hook height
      if(calculateObjPos(Ix, Iy) && objPos[2] > CARRYING_PACKAGE_MIN_CM 
        && objPos[2] > CARRYING_PACKAGE_MAX_CM)
      {
        //if package
        verifyFailCounter+=1;
        controllerState = STATE_DO_WAIT;
      }
      else
      {
        //if no package
        verifyFailCounter = 0;
        controllerState = STATE_PU_WAIT;
      }
      break;
    ////////////////////////////////////////////////////////////////////////////
    ///////////////////////////GO BACK TO THE TOP!//////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    }    

   //delay if needed   
   delay(max(0, MIN_LOOP_PERIOD_MILLIS - (millis()-lastLoopTimeMillis) ));
   lastLoopTimeMillis = millis();
}


