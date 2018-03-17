// Wii Remote IR sensor  test sample code  by kako http://www.kako.com
// modified output for Wii-BlobTrack program by RobotFreak http://www.letsmakerobots.com/user/1433
// modified for http://DFRobot.com by Lumi, Jan. 2014

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

const float frameSizePix = 768;
float fullFrameDistanceCm = 13.3;

int IRsensorAddress = 0xB0;
//int IRsensorAddress = 0x58;
int slaveAddress;
int ledPin = 13;
boolean ledState = false;
byte data_buf[16];
int i;

int Ix[4];
int Iy[4];
int s;

void Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1); Wire.write(d2);
    Wire.endTransmission();
}

void setup()
{
    slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    Serial.begin(19200);
    pinMode(ledPin, OUTPUT);      // Set the LED pin as output
    Wire.begin();
    // IR sensor initialize
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    
    pinMode(13, OUTPUT);
    analogWrite(13, 150);
    
    delay(100);
}
void loop()
{
    ledState = !ledState;
    if (ledState) { digitalWrite(ledPin,HIGH); } else { digitalWrite(ledPin,LOW); }


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
      if (Ix[i] < 1000)
        Serial.print("");
      if (Ix[i] < 100)  
        Serial.print("");
      if (Ix[i] < 10)  
        Serial.print("");
      Serial.print( int(Ix[i]) );
      Serial.print(",");
      if (Iy[i] < 1000)
        Serial.print("");
      if (Iy[i] < 100)  
        Serial.print("");
      if (Iy[i] < 10)  
        Serial.print("");
      Serial.print( int(Iy[i]) );
      if (i<3)
        Serial.print(",");
    }
    Serial.println("");

    //use pair of beacons math
    if(Ix[0] != 1023 && Ix[1] != 1023 && Iy[0] != 1023 && Iy[1] != 1023)
    {
      float objDist = fullFrameDistanceCm * frameSizePix/sqrt(((long)Iy[0]-Iy[1])*(Iy[0]-Iy[1]) + ((long)Ix[0]-Ix[1])*(Ix[0]-Ix[1]));
      Serial.print("Object distance (cm): ");Serial.println(objDist);
      Serial.println(sqrt(((long)Iy[0]-Iy[1])*(Iy[0]-Iy[1]) + ((long)Ix[0]-Ix[1])*(Ix[0]-Ix[1])));
      
      float centerX = (Ix[0]+Ix[1])/2.0f;
      float centerY = (Iy[0]+Iy[1])/2.0f;
      //todo compute angle, horizontal distance, vertical distance from object center to camera center
      
   }
    
    delay(500);
}
