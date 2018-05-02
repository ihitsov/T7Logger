
#define FREQUENCY    160                  // valid 80, 160
#include "ESP8266WiFi.h"
#include <EEPROM.h>
extern "C" {
#include "user_interface.h"
}
void pp_soft_wdt_stop();    // close software watchdog

#include <Wire.h>
#include <TimeLib.h>


const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t Accell[4] = {0};


byte PWMPin0 = D1;//46
byte PWMPin1 = D2;//46
byte InjPin0 = 10;//46
// N.B. D3 and D4 seems problematic if interrupt is active because it goes to GPIO0  and GPIO15 which needs to be high during boot

unsigned int InjTime = 0;
unsigned int HighTime = 0;
unsigned int LowTime = 0;
const byte PWMSamples = 3;
const byte InjSamples = 3;
byte DutyCycle = 0;
byte EngStatus=1;
unsigned int EngRPM;
unsigned int TempRPM;
unsigned int EngP;
unsigned int TempP;


volatile unsigned long PWMTimePulses0[PWMSamples];
volatile boolean PWMStatePulses0[PWMSamples];
byte PWM0Index=0;

volatile unsigned long PWMTimePulses1[PWMSamples];
volatile boolean PWMStatePulses1[PWMSamples];
byte PWM1Index=1;


volatile unsigned long InjTimePulses0[InjSamples];
volatile boolean InjStatePulses0[InjSamples];
byte Inj0Index=2;


volatile boolean PWMReady[3] = {0, 0, 0};


volatile boolean PWMRecording[3] = {1,1,1};


byte DutyCycle0 = 0;
byte DutyCycle1 = 0;
unsigned long InjT0 = 0;
#define FIRST 0 //Auth methods
#define SECOND 1
/* initialize */
int method = SECOND;
/* toggle */

unsigned int TempVal;


//const char* ssid = "";
//const char* password = "";

float tic = 0;
float toc = 0;
float Cal[3];
long unsigned canID = 0;
unsigned int Empty;
unsigned int CorrectFrame;
unsigned int n;
unsigned int len_DinitAck;
unsigned int len_Data;
unsigned int len_buf;
unsigned char Value[6] = {0, 0, 0, 0};

unsigned char Dinit[7] = {63, 129,  0,  17, 2,  64, 0};
unsigned int  DinitID = 544;

unsigned char DinitAck[8] = {64, 191,  33, 193,  0,  17, 2,  88};
unsigned int  DinitAckID = 568;

unsigned char SecAccReq[8] = {64,  161,  2,  39, 5,  0,  0,  0}; // Req security access
unsigned int  SecAccReqID = 576;

unsigned char SeedRcv[8] = {192,  191,  4,  103,  5,  201,  39, 0}; // receive seed 201 39
unsigned int  SeedRcvID = 600;

unsigned char ACK[4] = {64, 161,  63, 128}; // Acknowledge seed
unsigned int  ACKID = 614;

unsigned char SeedAns[8] = {64,  161,  4,  39, 6,  145,  72, 0}; // Send calculated answer
unsigned int  SeedAnsID = 576;

unsigned char Req[8] = {65, 161, 7, 35,  240,  71,  122, 2};
unsigned int  ReqID = 576;

unsigned char Req2[2] = {0, 161};
//unsigned char Req2A[2] = {0, 161};

unsigned int  ReqID2 = 576;


unsigned char Data[8] = {192, 191, 5, 99, 240,  94, 248, 12};
unsigned int  DataID = 600;

const unsigned int MaxSizeLog = 32; // preallocate the biggest possible log size, do not change this one. Larger values use more memory
unsigned int SizeLog; // initiallize how many variables we are logging, will set a value at the end of setup()

// Change parameters here
//
const String         Name[MaxSizeLog]   =      {"mReq",  "mAir",    "BCV",    "Pinl",   "O2Sf",   "Tair",   "Tinj",   "Amul",   "Lamb",   "LSwi",   "ECMS",   "Ioff"};// Name of the variable, only to go in the header of the log
unsigned long        Addr[MaxSizeLog]   =      {15746224, 15746938, 15752804, 15752290, 15752446, 15752422, 15752874, 15749636, 15750536, 15750440, 15752952, 15750906}; // Address of the variable in decimal, find the value in t7suite
unsigned int         Length[MaxSizeLog] =      {2,        2,        2,        2,        2,        2,        4,        2,        2,        2,        1,        2}; // Address length, find the value in t7suite
float              PollFreq[MaxSizeLog] =      {5,        5,       0.001,    10  ,    10   ,    0.001,      2,        10,     20,       10,       0.001,      10  ,     10}; //How often to poll the values in seconds, smaller number means more frequent polling 
//
// End of parameter changing block


long serialdata[MaxSizeLog];
int inbyte;

int ReceiveFilters[5]={0};


signed long          Values[MaxSizeLog] = {0};

const unsigned int SSizeLog = 24;
String         SName[SSizeLog]  =  {"RPM", "TPS", "SPD", "Teng", "Patm", "FL", "FR", "RL", "RR", "BrkPress", "StAng", "180_3", "180_4", "120_0", "120_1", "120_2", "120_3", "120_5", "120_6", "1a0_3", "1a0_4", "1a0_6", "1a0_7"};
long  unsigned         SValues[SSizeLog];

const unsigned int AnLog = 5;
String         AnName[AnLog]  =  {"O2Sens", "T1Pot", "T2Pot", "P1Pot", "P2Pot"};
//const int      AnScale[AnLog] = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3};
float         AnValues[AnLog];

const unsigned int DigFreq = 4;
String         DigFreqName[DigFreq]  =  {"PinlFreq", "PDelivFreq", "ABSSpeedFreq", "AMMFreq"};
long  unsigned         DigFreqVal[DigFreq];

const unsigned int DigPWM = 2;
String         DigPWMName[DigFreq]  =  {"PWM", "Purge"};
long  unsigned         DigPWMVal[DigFreq];

String         NameDigInjTime  =  {"DigInjTime"};
long  unsigned         DigInjTime;




//int16_t MaxAcX, MinAcX, MaxAcY, MinAcY = 0;
//int16_t Accell[4] = {0};

unsigned char len = 0;
unsigned char buf[8];


unsigned long ElapsedTime[MaxSizeLog];
unsigned long ElapsedPrintTime = millis();
//unsigned long ElapsedDispTime = millis();
unsigned long ElapsedGyroTime = millis();

unsigned long cyctime = millis();



unsigned int pos = 0;

//WiFiServer server(80);

#include <SPI.h>
#include "mcp_can.h"


MCP_CAN CAN(D8);                                    // D8 for esp8266 setup plus cheap MCP board, 10 for the elecfreaks on mega

void setup()
{
  Serial.begin(230400);
  delay(0);
  //setTime(hr,min,sec,day,month,yr); //Make a pseudo time stamp so t7suite can make use of the log as well
  setTime(12, 0, 0, random(1, 31), random(1, 12), random(1970, 2020));

  //  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  //  adcOne.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  adcTwo.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //
  //  adcOne.begin();
  //  adcTwo.begin();

  //  float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */

  
  WiFi.forceSleepBegin();                  // turn off ESP8266 RF
  delay(1);                                // give RF section time to shutdown
  system_update_cpu_freq(FREQUENCY);
  //   wifi_status_led_uninstall();

    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

  

  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_16MHz))              // init can bus : baudrate = 500k // Go to C:\Program Files (x86)\Arduino\libraries\CAN_BUS_Shield-master\mcp_can_dfs.h and change line:   byte begin(byte speedset, const byte clockset = MCP_16MHz);   to      byte begin(byte speedset, const byte clockset);     // init can

  {
    Serial.println("CAN BUS no es bueno!"); // Ref to AvE
    delay(1000);
  }

  Serial.println("CAN BUS Shield init ok!");

  for (int i=0;i<MaxSizeLog;i++){ // Determine the size of the log 
    if (Addr[i]==0){break;}
    SizeLog++;
  }
}



void printDigits(int digits) {
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
   
union u_tag {
  byte b[3];
  unsigned long ival;
} u;


uint16_t CalcKey(uint8_t method, unsigned char ReceivedSeed[8]) {

  uint16_t seed;
  seed = ReceivedSeed[5] << 8 | ReceivedSeed[6];
  seed  = seed << 2 & 0xffff;
  seed ^= (method ? 0x4081 : 0x8142);
  seed -= (method ? 0x1f6f : 0x2356);
  return seed;
}

void SendACK(int row) { //Make an ACK frame based on the receved frame
  if (row == 26) {
    ACK[3] = 195;
  }
  if (row == 195) {
    ACK[3] = 131;
  }
  if (row == 130) {
    ACK[3] = 130;
  }
  if (row == 129 || row == 193) {
    ACK[3] = 129;
  }
  if (row == 192 || row == 128) {
    ACK[3] = 128;
  }
  CAN.sendMsgBuf(ACKID, 0, 4, ACK);
  return;
}

void bufprint(unsigned char buf[8], byte l) {
  for (int i = 0; i < l; i++) // print the data (for debugging)
  {
    Serial.print(buf[i]);
    if (i < l - 1) {
      Serial.print(",");
      Serial.print("\t");
    }
  }
  Serial.print("\n");
  return;
}

int isCorrectFrame(unsigned char buf[8], unsigned char Comp[8], int n1, int n2) {
  int CorrectFrame = 1;
  for (int n = n1; n < n2; n++) {
    if (Comp[n] != buf[n]) {
      CorrectFrame = 0; // test each element to be the same. if not, return false
    }
  }
  return CorrectFrame;
}

void Gyro() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // request a total of 14 registers(now 6)
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  if (AcX < Accell[0]) {
    Accell[0] = AcX;
  }
  if (AcY < Accell[1]) {
    Accell[1] = AcY;
  }
  if (AcX > Accell[2]) {
    Accell[2] = AcX;
  }
    if (AcX > Accell[3]) {
    Accell[3] = AcX;
  }
}

void SerialPrint(boolean Header) {

  if (Header==1){Serial.print("Timestamp");}
  if (Header==0){
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(".");
  Serial.print(millis() % 1000);}


  for (int i = 0; i < SizeLog; i++) // print the data
  {
    Serial.print(",");
    if (Header==1){Serial.print(Name[i]);}
    if (Header==0){Serial.print(Values[i]);}
  }
  for (int i = 0; i < SSizeLog; i++) // print the data
  {
    Serial.print(",");
    if (Header==1){Serial.print(SName[i]);}
    if (i<12){
    if (Header==0){Serial.print(SValues[i]);}
    }
    if (i>=12){ // Print the unknown values in HEX for easier reverse engineering in case its more than a byte long variable
    if (Header==0){Serial.print(SValues[i],HEX);}      
    }
  }
  Serial.print(","); 
  if (Header==1){Serial.print("AccX");} 
  if (Header==0){Serial.print((AcX / 16.384), 0);}
  Serial.print(","); 
  if (Header==1){Serial.print("AccY");} 
  if (Header==0){Serial.print((AcY / 16.384), 0);}
  Serial.print(",");
  if (Header==1){Serial.print("AccZ");} 
  if (Header==0){Serial.print((AcZ / 16.384), 0);}

  Serial.print(","); 
  if (Header==1){Serial.print("FPS");}
  int FPS=1000000 / (micros() - cyctime);
  if (Header==0){Serial.print(FPS);}
  cyctime = micros();

  Serial.print("\n"); 

}

void getSerial()
{
  memset(serialdata,0,MaxSizeLog*sizeof(serialdata[0]));
  int indx=0;
  //serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    if (inbyte==','){indx++; serialdata[indx]=0;}
    if (inbyte > 0 && inbyte != '/' && inbyte!=',')
    {     
      serialdata[indx] = serialdata[indx] * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return ;
}

void SendCAN()
{
byte Canbuf[8];
int CanID=0;
int messagebuf[9];
int bufidx=0;
String inString = "";    // string to hold input
int inChar;

  memset(Canbuf,0,8*sizeof(byte));
  memset(messagebuf,0,9*sizeof(int));
  while (inChar != '/')
  { 


     int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    if (inChar == ',' || inChar == '/') {
      messagebuf[bufidx]=inString.toInt();
      inString = "";
      bufidx++;
    }
  if   (inChar == '/'){
    CanID=messagebuf[0];
    for (int i=1; i<9; i++){
    Canbuf[i-1]=messagebuf[i];  
    }
    break;
  }
  }
  CAN.sendMsgBuf(CanID, 0, bufidx-1, Canbuf);
  return ;
}

void ReceiveCAN()
{
int messagebuf[5];
int bufidx=0;
String inString = "";    // string to hold input
int inChar;
  memset(ReceiveFilters,0,5*sizeof(int));
  while (inChar != '/')
  { 
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    if (inChar == ',' || inChar == '/') {
      ReceiveFilters[bufidx]=inString.toInt();
      inString = "";
      bufidx++;
    }
    if (inChar == '/') {break;}
  }
  return;
}

void ReceivePrint(int canID, int len, byte buf[8]){
  Serial.print("RX,");
  Serial.print(canID);
  Serial.print(",");
  for (int i=0;i<len;i++){
    Serial.print(buf[i]);
    if (i!=len-1){
    Serial.print(",");
    }
  } 
  Serial.println("");
}
void ServiceSerial(){

      switch ((int)serialdata[0])
      {
      case 1:
        {
          //SizeLog
          getSerial();
          SizeLog = (int)serialdata[0];
          break;
        }
      case 2:
        {
          //Addr
          getSerial();
          for (int i=0;i<MaxSizeLog;i++){Addr[i]=serialdata[i];}
          break;
        }
      case 3:
        {
          //Length
          getSerial();
          for (int i=0;i<MaxSizeLog;i++){Length[i]=serialdata[i];}
          break;
        }
      case 4:
        {
          //PollFreq
          getSerial();
          for (int i=0;i<MaxSizeLog;i++){PollFreq[i]=(float)serialdata[i]/1000.;}
          break;
        }
      case 5:
        {
          //Send CAN message e.g. 5/ID,1,2,3,4,5,6,7,8/
          SendCAN();
          break;
        }        
      case 6:
        {
          //Receive CAN messages in HEX, set receive filter e.g. 6/ID1,ID2,ID3,.../  , 6/0 disables receiving messages
          ReceiveCAN();
          break;
        }
        }
}

void WaitForEngStatus(){
        unsigned long breakOutTime=millis();
 while (true){
        delay(0);
        if (CAN_MSGAVAIL == CAN.checkReceive()){           // check if data is coming
        CAN.readMsgBufID(&canID, &len, buf);   // read data,  len: data length, buf: data buf
        for (int i=0;i<5;i++){ // print messages that are interesting according to the filter
         if (canID==ReceiveFilters[i]){ReceivePrint(canID,len,buf);}
        }
        TempRPM = buf[1] << 8 | buf[2] << 0;//Latest broadcasted RPM
        TempP =  buf[3];
          if (canID==416 && TempRPM!=EngRPM){
          EngStatus=1;    // Eng status changed
          EngRPM=TempRPM;// update last recorded RPM
          EngP=TempP;
          SValues[0]=TempRPM;
          SValues[1] = buf[5] ; //TPS
          break;
          }
      }
 if ((millis()-breakOutTime)>200){ // prevent getting stuck in the while loop
  break;
 }
      else{EngStatus=0;}
 }
}

void ReceiveCheck(){
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data is coming
    {
      CAN.readMsgBufID(&canID, &len, buf);   // read data,  len: data length, buf: data buf
      for (int i=0;i<5;i++){ // print messages that are interesting according to the filter
        if (canID==ReceiveFilters[i]){ReceivePrint(canID,len,buf);}
      }
    }
}
unsigned char MessageWait(int ID)
{
  tic = millis();
  toc = tic;
  while (true)           // check if data coming
  {

    tic = millis();
    if ((tic - toc) > 5) // give up waiting if DATA is not received in time
    {
      memset(buf, 0, 8 * sizeof(buf[0]));
      Empty=1;
      return buf[8];
    }
    if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data is coming
    {
      CAN.readMsgBufID(&canID, &len, buf);   // read data,  len: data length, buf: data buf
    for (int i=0;i<5;i++){ // print messages that are interesting according to the filter
      if (canID==ReceiveFilters[i]){ReceivePrint(canID,len,buf);}
    }
      if (canID == 416) {
        EngStatus=buf[0];
        SValues[0] = buf[1] << 8 | buf[2] << 0;
        SValues[20] = buf[3];
        SValues[21] = buf[4];
        SValues[22] = buf[6];
        SValues[23] = buf[7];
        TempRPM = SValues[0];//
        TempP= buf[3];
        if (EngRPM!=TempRPM && EngP!=TempP){
          EngRPM=TempRPM;
          EngP=TempP;
          EngStatus=1;}
        if (EngRPM==TempRPM && EngP==TempP){EngStatus=0;}
        //      SValues[1] = buf[3] ;
        
        SValues[1] = buf[5] ;

      }
      if (canID == 768) {
        SValues[6] = (buf[0] << 8 | buf[1] << 0) ; //Speed FL
        SValues[7] = (buf[2] << 8 | buf[3] << 0) ; //Speed FR
        SValues[8] = (buf[4] << 8 | buf[5] << 0) ; //Speed RL
        SValues[9] = (buf[6] << 8 | buf[7] << 0) ; //Speed RR
      }
      if (canID == 288) {
        SValues[10] = buf[7] ; //Brake pressure
        SValues[11] = buf[4] ; // Steering angle

        SValues[14] = buf[0] ; // Unknown
        SValues[15] = buf[1] ; // Unknown
        SValues[16] = buf[2] ; // Unknown
        SValues[17] = buf[3] ; // Unknown
        SValues[18] = buf[5] ; // Unknown
        SValues[19] = buf[6] ; // Unknown
      }
      if (canID == 384) {
        SValues[12] = buf[3] ; //180_3 Unknown
        SValues[13] = buf[4] ; // 180_4 Unknown
      }
            
      if (canID == 928) {
        SValues[2] = (buf[3] << 8 | buf[4] << 0) / 10; //Speed
      }


      if (canID == 288) {
        SValues[10] = buf[7] ; //Brake pressure
        SValues[11] = buf[4] ; // Steering angle

        SValues[14] = buf[0] ; // Unknown
        SValues[15] = buf[1] ; // Unknown
        SValues[16] = buf[2] ; // Unknown
        SValues[17] = buf[3] ; // Unknown
        SValues[18] = buf[5] ; // Unknown
        SValues[19] = buf[6] ; // Unknown
      }
      

      if (canID == 384) {
        SValues[12] = buf[3] ; //180_3 Unknown
        SValues[13] = buf[4] ; // 180_4 Unknown
      }

      if (canID == 801) {
        DigFreqVal[0] = (buf[1] << 8) | buf[0];
        DigFreqVal[1] = (buf[3] << 8) | buf[2];
        DigFreqVal[2] = (buf[5] << 8) | buf[4];
        DigFreqVal[3] = (buf[7] << 8) | buf[6];
      }
      if (canID == 802) {
        AnValues[1] = (buf[1] << 8) | buf[0];
        AnValues[2] = (buf[3] << 8) | buf[2];
        AnValues[3] = (buf[5] << 8) | buf[4];
        AnValues[4] = (buf[7] << 8) | buf[6];
      }
//      if (canID == 803) {
//        AnValues[0] = (buf[1] << 8) | buf[0];
//        AcX = (buf[3] << 8) | buf[2];
//        AcY = (buf[5] << 8) | buf[4];
//        AcZ = (buf[7] << 8) | buf[6];
//      }

      if (canID == 1472) {
        SValues[3] = (buf[1] - 40) ;
        SValues[4] = buf[3] << 8 | buf[4] << 0;

      
      }

      if (ID == canID)  {
        SendACK(buf[0]);
        Empty=0;
        return buf[8];
      }
    }
  }
}


String result;


void datainit(int printed) {
  while (true) {
    CAN.sendMsgBuf(DinitID, 0, 7, Dinit);  // Initialize the T7 data communication
    buf[8] = MessageWait(DinitAckID);
    
    delay(0);
    int Correct = isCorrectFrame(buf, DinitAck, 0, 3);

    if (Empty == 1 || Correct == 0) {
      if (printed==1){ Serial.println(String("Reinitializing ECU"));}
      CAN.sendMsgBuf(DinitID, 0, 8, Dinit);
      delay(500);
    }
    else {
      if (printed==1){Serial.println("ECU is ready to communicate");}
      break;
    }
  }
}

void reqsecacc(int printed) {
  while (true) {
    if (printed==1){Serial.println(String("Requesting security access"));}
    CAN.sendMsgBuf(SecAccReqID, 0, 7, SecAccReq);  // Request security access
    buf[8] = MessageWait(DataID);
    int Correct = isCorrectFrame(buf, SeedRcv, 0, 3);

    if (Empty == 1 || Correct == 0) {
      if (printed==1){Serial.println(String("ECU is not replying to security access request, trying again"));}
      delay(500);
      CAN.sendMsgBuf(SecAccReqID, 0, 7, SecAccReq);  // Request security access
    }
    else {
      if (printed==1){Serial.println("Seed received");}
      SeedRcv[5] = buf[5];
      SeedRcv[6] = buf[6];
      break;
    }
  }
}

void authentication(int printed) {
  while (true) {

    method = FIRST + SECOND - method; //toggle method between 0 and 1
    uint16_t seed = CalcKey(method, SeedRcv);
    SeedAns[5] = seed >> 8 & 0xff;
    SeedAns[6] = seed & 0xff;
    CAN.sendMsgBuf(SeedAnsID, 0, 8, SeedAns);  // Send back the calculated authentication
    buf[8] = MessageWait(DataID); // Acknowledge and return data frame
    int Correct = isCorrectFrame(buf, SeedRcv, 0, 3);
    if (Correct == 1) {
      SeedRcv[5] = buf[5];
      SeedRcv[6] = buf[6];
    }


    delay(0);

    if (Empty == 1 ) {
      if (printed==1){Serial.println(String("Cannot get access, trying again"));}
      delay(500);
      break;
    }
    if (buf[3] == 103 && buf[5] == 52) {
      if (printed==1){Serial.println("Security access OK");}
      if (printed==1){Serial.println("We are requesting data");}
      break;
    }
  }
}
unsigned int dropped = 0;

void polling() {
     SerialPrint(1); // (1) means print headers
     int DroppedCount=0;
while (true) {
    delay(0);
    if (SizeLog==0){ReceiveCheck();}
    if (dropped==1){DroppedCount++;}    
    else{DroppedCount=0;}
    if (DroppedCount>3){datainit(0);reqsecacc(0);authentication(0);}
    
    if (EngStatus==0 && pos==0 && SizeLog!=0){
      WaitForEngStatus();
    }
    EngStatus=0; 

    int polled = 1;
    if ((millis() - ElapsedTime[pos]) < (1000 * PollFreq[pos]) && dropped == 0)
    {
      polled = 0;
      Empty = 0;
      CorrectFrame = 1;
    }
    dropped = 0;
    if (Serial.available()>0){
      getSerial();
      ServiceSerial();
      //reset the polling cycle
      pos=0;
      polled=0;
      dropped=0;
      CorrectFrame=1;
      Empty=0;
      //
    }

    if (polled == 1 && SizeLog!=0) {

    //update the request and the data for the current position
    u.ival = Addr[pos];
    Req[5] = u.b[1];
    Req[6] = u.b[0];
    Req[7] = Length[pos];
    Data[5] = u.b[1];
    Data[6] = u.b[0];

    
        ElapsedTime[pos] = millis();      
      if (PWMReady[0] == true) {
        PWMReady[0] = false;
        PWMEval(PWMTimePulses0, PWMStatePulses0, PWMSamples, PWM0Index);
        DigPWMVal[0] = DutyCycle;
      }

      if (PWMReady[1] == true) {
        PWMReady[1] = false;
        PWMEval(PWMTimePulses1, PWMStatePulses1, PWMSamples, PWM1Index);
        DigPWMVal[1] = DutyCycle;
      }

      if (PWMReady[2] == true) {
        PWMReady[2] = false;
        PWMEval(InjTimePulses0, InjStatePulses0, InjSamples, Inj0Index);
        DigInjTime = HighTime;
      }

      CAN.sendMsgBuf(ReqID, 0, 8, Req);  // Request data with the current address and length
      CAN.sendMsgBuf(ReqID2, 0, 2, Req2);  // End of request, kind of silly that this cannot be incorporated into the first request, is it possible? Will speed things up. Anoter possibility is to try to request all the values at the same time and see how T7 reacts. This will be super fast if possible
      
      buf[8] = MessageWait(DataID);
      delay(0);
      CorrectFrame = isCorrectFrame(buf, Data, 5, 7); // check if the address of the frame is correct

      if (Empty == 1 || CorrectFrame == 0) {
        dropped = 1;
//        Serial.println(String("Dropped frame 1, requesting again"));
        SendACK(buf[0]);
        delay(0);
      }
      if (Empty == 0 && CorrectFrame == 1 && dropped == 0) {
        if (buf[0] == 192) {//Single frame received
          TempVal = buf[7];
        }
      }

      if (buf[0] == 193) {//Two part frame received
        Value[0] = buf[7];
        buf[8] = MessageWait(DataID);
        if (buf[0] != 128) {
          CorrectFrame = 0; // if the second frame doesnt start with 128 then its incorrect
          SendACK(buf[0]);
        }
        if (Empty == 1 || CorrectFrame == 0 || dropped == 1) {
//          Serial.println(String("Dropped frame 2, requesting again"));
          SendACK(buf[0]);
          delay(0);
        }
        if (Empty == 0 && CorrectFrame == 1 && dropped == 0) {
          Value[1] = buf[2];
          Value[2] = buf[3];
          Value[3] = buf[4];
          //                  bufprint(buf);
          if (Length[pos] == 6) {
            TempVal = Value[0] << 40 | Value[1] << 32 | Value[2] << 24 | Value[3] << 16 | Value[4] << 8 | Value[5] << 0;
          }          
          if (Length[pos] == 5) {
            TempVal = Value[0] << 32 | Value[1] << 24 | Value[2] << 16 | Value[3] << 8 | Value[4] << 0;
          }
          if (Length[pos] == 4) {
            TempVal = Value[0] << 24 | Value[1] << 16 | Value[2] << 8 | Value[3] << 0;
          }
          if (Length[pos] == 3) {
            TempVal = Value[0] << 16 | Value[1] << 8 | Value[2] << 0;
          }
          if (Length[pos] == 2) {
            TempVal = Value[0] << 8 | Value[1] << 0;
          }
        }
      }
          Values[pos] = TempVal;
      //    ADMeasure(); // Measure the analogue values
    }
    unsigned int Problem = 0;
    if (CorrectFrame == 0 || Empty == 1 || dropped == 1) {
      Problem = 1;
    }
    if (Values[pos] > (65535 - 2500) && Length[pos] < 4) {
      Values[pos] = Values[pos] - 65535; //take care of negative values
    }
    
    if (pos == SizeLog - 1 && Problem == 0) {
    wdt_reset();    
        if ((millis() - ElapsedGyroTime) > 0) {
          Gyro();
          ElapsedGyroTime = millis(); // if its time to poll the accelerometer do it
        }
           
      // Serial printing
      SerialPrint(0);
    }

    if (Problem == 0 && SizeLog!=0)
    {
      pos++;
      if (pos == SizeLog) {
        pos = 0; //restart the counter when all the values are polled
      }
    }
  }
}



void PWMCounter0() {
  static  byte i = 0;
  //  volatile static  unsigned long TimePulses[PWMSamples] = {0};
  //  volatile static  boolean StatePulses[PWMSamples] = {0};

  if (PWMRecording[0] == 1) {
    if (i < PWMSamples) {
      PWMStatePulses0[i] = digitalRead(PWMPin0); // Record state high/low
      PWMTimePulses0[i] = micros(); // Record time of pulse
      i++; // increment index
    }
    if (i == PWMSamples) {
      i = 0;
      PWMReady[0] = true;
      PWMRecording[0] = false;
    }
  }

}

void PWMCounter1() {
  static  byte i = 0;
  //  volatile static  unsigned long TimePulses[PWMSamples] = {0};
  //  volatile static  boolean StatePulses[PWMSamples] = {0};

  if (PWMRecording[1] == 1) {
    if (i < PWMSamples) {
      PWMStatePulses1[i] = digitalRead(PWMPin1); // Record state high/low
      PWMTimePulses1[i] = micros(); // Record time of pulse
      i++; // increment index
    }
    if (i == PWMSamples) {
      i = 0;
      PWMReady[1] = true;
      PWMRecording[1] = false;
    }
  }

}

void InjCounter0() {
  static  byte i = 0;

  if (PWMRecording[2] == 1) {
    if (i < InjSamples) {
      InjStatePulses0[i] = digitalRead(InjPin0); // Record state high/low
      InjTimePulses0[i] = micros(); // Record time of pulse
      i++; // increment index
    }
    if (i == InjSamples) {
      i = 0;
      PWMReady[2] = true;
      PWMRecording[2] = false;
    }
  }

}

void PWMEval(volatile unsigned long PWMTimePulses[PWMSamples], volatile boolean PWMStatePulses[PWMSamples],byte Samples, byte Index) {


  unsigned  long TotalLowTime = 0;
  unsigned  long TotalHighTime = 0;
  int TotalLowPulses = 0;
  int TotalHighPulses = 0;


  for (byte j = 0; j < (Samples - 1); j++) {

    if (PWMStatePulses[j] == 0) {
      TotalLowTime = TotalLowTime + PWMTimePulses[j + 1] - PWMTimePulses[j];
      TotalLowPulses = TotalLowPulses + 1;
//                  Serial.println("TotalLowTime");
//            Serial.println(TotalLowTime);
    }

    if (PWMStatePulses[j] == 1) {
      TotalHighTime = TotalHighTime + PWMTimePulses[j + 1] - PWMTimePulses[j];
      TotalHighPulses = TotalHighPulses + 1;
//                        Serial.println("TotalHighTime");
//            Serial.println(TotalHighTime);
    }
  }

  PWMRecording[Index] = true; // Allow recording of pulses again

  LowTime = TotalLowTime / (TotalLowPulses + 1e-5);
  HighTime = TotalHighTime / (TotalHighPulses + 1e-5);
  DutyCycle = 100 * LowTime / (HighTime + LowTime);
//  Serial.println("LowTime");
//  Serial.println(LowTime);
//  Serial.println(HighTime);
//  Serial.println(DutyCycle);

}
///--------------------------------------------------------------------------------------------------------------------------------------------------
///-------------------LOOP-------------------------------------------------------------------------------------------------------------------------------
///--------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{
    pinMode(PWMPin0, INPUT);
  pinMode(PWMPin1, INPUT);
  pinMode(InjPin0, INPUT);

  attachInterrupt(digitalPinToInterrupt(PWMPin0), PWMCounter0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWMPin1), PWMCounter1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(InjPin0), InjCounter0, CHANGE);
  
  datainit(1);
  reqsecacc(1);
  authentication(1);
  polling();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
