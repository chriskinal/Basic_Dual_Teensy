/*
 * This is a simple GPS code for AgOpen GPS
 * It can used as single antenna with IMU and send PANDA to AgOpen
 * Or use two F9P and send PAOGI to AgOpen
 */
 
#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

/************************* User Settings *************************/
#define deBugPin   22
bool deBug = false;

//Serial Ports

#define SerialAOG Serial    //AgOpen / USB
HardwareSerial* SerialGPS = &Serial2;   //Main postion receiver (GGA) (Serial2 must be used here with T4.0 / Basic Panda boards - Should auto swap)
HardwareSerial* SerialGPS2 = &Serial7;  //Dual heading receiver 

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;
 
constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];   //Extra serial tx buffer

//is the GGA the second sentence?
const bool isLastSentenceGGA = true;

//I2C pins, SDA = 18, SCL = 19
//Note - Pullup resistors will be needed on both SDA & SCL pins

//Ethernet
byte Eth_myip[4] = { 192, 168, 137, 80 }; //IP address for this GPS module
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xB3, 0x1B}; // original
byte Eth_ipDest_ending = 255;        //ending of IP address to send UDP data to
unsigned int portMy = 5544;          //this is port of this module
unsigned int AOGNtripPort = 2233;    //port NTRIP data from AOG comes in
unsigned int portDestination = 9999; //Port of AOG that listens
bool Ethernet_running = false;
char Eth_NTRIP_packetBuffer[512];    // buffer for receiving and sending data

IPAddress Eth_ipDestination;
EthernetUDP Eth_udpPAOGI;
EthernetUDP Eth_udpNtrip;

//Swap BNO08x roll & pitch?
const bool swapRollPitch = false;
//const bool swapRollPitch = true;

//BNO08x, time after last GPS to load up IMU ready data for the next Panda takeoff
const uint16_t IMU_DELAY_TIME = 90; //Best results seem to be 90-95ms
uint32_t IMU_lastTime = IMU_DELAY_TIME;
uint32_t IMU_currentTime = IMU_DELAY_TIME;

//BNO08x, how offen should we get data from IMU (The above will just grab this data without reading IMU)
const uint16_t GYRO_LOOP_TIME = 10;  
uint32_t lastGyroTime = GYRO_LOOP_TIME;

//CMPS14, how long should we wait with GPS before reading data from IMU then takeoff with Panda
const uint16_t CMPS_DELAY_TIME = 4;  //Best results seem to be around 5ms
uint32_t gpsReadyTime = CMPS_DELAY_TIME;

// Booleans to see if we are using CMPS or BNO08x or Dual
bool useCMPS = false;
bool useBNO08x = false;
bool useDual = false;
bool GGAReady = false;
bool relPosReady = false;

//CMPS always x60
#define CMPS14_ADDRESS 0x60 

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A,0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

//Dual 
double headingcorr = 900;  //90deg heading correction (90deg*10)

float baseline;
double baseline2;
float rollDual;
float rollDualRaw;
double relPosD;
double relPosDH;
double heading = 0;

byte CK_A = 0, CK_B = 0;
byte incoming_char;
boolean headerReceived = false;
unsigned long ackWait = millis();
byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int i = 0;

 /* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false, blink;

//100hz summing of gyro
float gyro, gyroSum;
float lastHeading;

float roll, rollSum;
float pitch, pitchSum;

float bno08xHeading = 0;
int16_t bno08xHeading10x = 0;

// Define functions
void errorHandler();
void GGA_Handler();
void VTG_Handler();
void imuHandler();
void BuildPANDA();
void GyroHandler( uint32_t delta );
void relPosDecode();

// Status LED's
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green

void setup()
{   
    pinMode(Power_on_LED, OUTPUT);
    pinMode(Ethernet_Active_LED, OUTPUT);
    pinMode(GPSRED_LED, OUTPUT);
    pinMode(GPSGREEN_LED, OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);
    digitalWrite(Power_on_LED, 1);

    SerialAOG.begin(baudAOG);
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    //the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);

    Wire.begin();
    delay(500);
    pinMode(13, OUTPUT);
    pinMode(deBugPin, INPUT_PULLUP);
    deBug = !digitalRead(deBugPin);
    deBug = true;
    Serial.println();
    
    //test if CMPS working
    uint8_t error;
    if(deBug) Serial.println("Checking for CMPS14");
    Wire.beginTransmission(CMPS14_ADDRESS);
    error = Wire.endTransmission();

    if (error == 0)
    {
        if(deBug) {
          Serial.println("Error = 0");
          Serial.print("CMPS14 ADDRESs: 0x");
          Serial.println(CMPS14_ADDRESS, HEX);
          Serial.println("CMPS14 Ok.");
        } 
        useCMPS = true;
    }
    else
    {
        if(deBug) {
          Serial.println("Error = 4");
          Serial.println("CMPS not Connected or Found"); 
        }
    }

    if (!useCMPS)
    {
        for (int16_t i = 0; i < nrBNO08xAdresses; i++)
        {
            bno08xAddress = bno08xAddresses[i];
          if(deBug) {
            Serial.print("\r\nChecking for BNO08X on ");
            Serial.println(bno08xAddress, HEX);
          }
            Wire.beginTransmission(bno08xAddress);
            error = Wire.endTransmission();

            if (error == 0)
            {
              if(deBug) {
                Serial.println("Error = 0");
                Serial.print("BNO08X ADDRESs: 0x");
                Serial.println(bno08xAddress, HEX);
                Serial.println("BNO08X Ok.");
              }
                          // Initialize BNO080 lib        
                if (bno08x.begin(bno08xAddress))
                {
                    Wire.setClock(400000); //Increase I2C data rate to 400kHz

            // Use gameRotationVector
            bno08x.enableGyro(GYRO_LOOP_TIME);
            bno08x.enableGameRotationVector(GYRO_LOOP_TIME-1);
                       
            // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
            if (bno08x.getFeatureResponseAvailable() == true)
            {
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, (GYRO_LOOP_TIME-1)) == false) bno08x.printGetFeatureResponse();
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, (GYRO_LOOP_TIME-1)) == false) bno08x.printGetFeatureResponse();
              digitalWrite(GPSRED_LED, 1);

              // Break out of loop
              useBNO08x = true;
              break;
            }
                    else
                    {
                        if(deBug) Serial.println("BNO08x init fails!!");
                    }
                }
                else
                {
                    if(deBug) Serial.println("BNO080 not detected at given I2C address.");
                }
            }
            else
            {
                if(deBug) {
                  Serial.println("Error = 4");
                  Serial.println("BNO08X not Connected or Found"); 
                }    
            }
        }
    }
    
   if (!useCMPS && !useBNO08x)
    {
      SerialGPS2->begin(baudGPS);
      SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
      SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);
      //useDual = true;
    }

//Ethernet

  Serial.println("Starting Ethernet");
  Ethernet.begin(mac, Eth_myip);
  delay(3000);
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found, sending data via USB only.");
    Ethernet_running = false;
    return;
  }
  else {
    Serial.println("Ethernet hardware found, checking for connection");
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected yet, sending data via USB & Ethernet UDP anyway.");
      //digitalWrite(Power_on_LED, 1);
      digitalWrite(Ethernet_Active_LED, 0);
    }
    else {
      Serial.println("Ethernet connected, sending data via USB & Ethernet UDP");
      if (Ethernet.linkStatus() == LinkON) {
        //digitalWrite(Power_on_LED, 0);
        digitalWrite(Ethernet_Active_LED, 1);
       }
    }       
      for (byte n = 0; n < 3; n++) {
          Eth_ipDestination[n] = Eth_myip[n];
        }
        Eth_ipDestination[3] = Eth_ipDest_ending;
        
      Ethernet_running = true;
      Serial.println();
      Serial.print("Ethernet IP of roof module: "); Serial.println(Ethernet.localIP());
      Serial.print("Ethernet sending to IP: "); Serial.println(Eth_ipDestination);
      //init UDP Port sending to AOG
      if (Eth_udpPAOGI.begin(portMy)) // portMy  portDestination
      {
        Serial.print("Ethernet UDP sending from port: ");
        Serial.println(portMy);
        Serial.print("Ethernet UDP sending to port: ");
        Serial.println(portDestination);
      }
      //init UDP Port getting NTRIP from AOG
      if (Eth_udpNtrip.begin(AOGNtripPort)) // AOGNtripPort
      {
        Serial.print("Ethernet NTRIP UDP listening to port: ");
        Serial.println(AOGNtripPort);
      }

  }
  
    Serial.println();
    Serial.println("Basic Dual or Single GPS for AgOpenGPS"); 
    Serial.println("Setup done, waiting for GPS Data....."); 
    if (Ethernet_running) Serial.println("Sending Data Via Ethernet and USB"); 
    else Serial.println("Sending Data Via USB Only"); 
    Serial.println();
    delay(2000);
}

void loop()
{
    //Serial.println("TOP OF LOOP");
    //Read incoming nmea from GPS
    if (SerialGPS->available())
        parser << SerialGPS->read();

    //Pass NTRIP etc to GPS
    if (SerialAOG.available())
        SerialGPS->write(SerialAOG.read());

    if (Ethernet_running) doEthUDPNtrip();

    deBug = !digitalRead(deBugPin);
    IMU_currentTime = millis();

if(!useDual){
  
  if (useBNO08x)
    {  
      if (isTriggered && IMU_currentTime - IMU_lastTime >= IMU_DELAY_TIME)
      {
        //Load up BNO08x data from gyro loop ready for takeoff
        imuHandler();

        //reset the timer 
        isTriggered = false;
      }      
    }

  if (useCMPS)
    { 
      if (isTriggered && IMU_currentTime - gpsReadyTime >= CMPS_DELAY_TIME)
      {
        imuHandler(); //Get data from CMPS (Heading, Roll, Pitch) and load up ready for takeoff
        BuildPANDA(); //Send Panda

        //reset the timer 
        isTriggered = false;
      }
    }  

  IMU_currentTime = millis();    

  if (IMU_currentTime - lastGyroTime >= GYRO_LOOP_TIME)
    {
        GyroHandler(IMU_currentTime - lastGyroTime);
    }
    
}//End Not Dual

if(!useCMPS && !useBNO08x){

if(GGAReady == true && relPosReady == true) {
  BuildPANDA();
  GGAReady = false;
  relPosReady = false;
}
  
    if (SerialGPS2->available()) {
    incoming_char = SerialGPS2->read();
    if (i < 4 && incoming_char == ackPacket[i]) {
      i++;
    }
    else if (i > 3) {
      ackPacket[i] = incoming_char;
      i++;
    }
  }
  if (i > 71) {
    checksum();
    i = 0;
  }
 } //Dual

} //Loop

//**************************************************************************

void checksum() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 70 ; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  if (CK_A == ackPacket[70] && CK_B == ackPacket[71]) {
  if(deBug) Serial.println("ACK Received! ");
    useDual = true;
    relPosDecode();
  }
  else {
  if(deBug) Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

//**************************************************************************

void doEthUDPNtrip() {
  unsigned int packetLenght = Eth_udpNtrip.parsePacket();
  if (packetLenght > 0) {
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLenght);
    SerialGPS->write(Eth_NTRIP_packetBuffer, packetLenght);
  }  
} 
