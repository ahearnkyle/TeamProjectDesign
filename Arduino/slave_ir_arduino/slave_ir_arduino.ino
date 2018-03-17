// Wii Remote IR sensor  test sample code  by kako http://www.kako.com
// modified output for Wii-BlobTrack program by RobotFreak http://www.letsmakerobots.com/user/1433
// modified for http://DFRobot.com by Lumi, Jan. 2014

//reads data from serial camera and relays along 
//master sends 0xAA and slave responds with 8 16-bit integers
//coordinates are written in the order x0 y0 x1 y1 x2 y2 x3 y3

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

unsigned int Ix[4];
unsigned int Iy[4];
unsigned int s;

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
    Serial.println("setup");
}
void loop()
{
  //master sends '0' to request 
  if(Serial.available() > 0)
  {  
    char b = Serial.read();
    if(b == 0x30)
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
      for(i=0; i<4; i++)
      {
        Serial.write((char)(Ix[i]));
        Serial.write((char)(Ix[i] >> 8));
        Serial.write((char)(Iy[i]));
        Serial.write(Iy[i] >> 8);
      }
   }
  }
}
