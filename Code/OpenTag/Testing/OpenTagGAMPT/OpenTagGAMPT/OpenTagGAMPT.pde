/* Loggerhead Instruments OpenTagGAMPT v1.1
   Release 22 November 2012
   Copyright 2012 by David Mann

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Additional software and schematics available at http://www.loggerheadinstruments.com

// Some code derived from SparkFun SF9DOF_AHRS by Doug Weibel and Jose Julio
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel

//Notes
// 0. Written with Arduino v0022.  Note Arduino v1.0 will not work because of issues with changes to Wire.h that produce compile errors.  Waiting for next update which will fix this.
// 1. OpenTag (Windows software) is used to create the Default.txt file that is read to set the time and sample rates.
// 2. The current time will only be updated from the schedule if it is more recent than the current time held by the RTC, unless Force Reset.
// 3. Orientation of x,y,z follows that of accelerometer on board (readings of magnetometer correct to same orientation when read).
// 4. A7 analog channel has been commented out of this version.
// 5. Calculations of Pressure involve 64-bit math which chews up a lot of sketch size.  So, just saving raw 24-bit pressure and temp reads along with coefficients to calc P and T in post-processing. 
// 6. Burn time is not implemented
// 7. For programming with Sparkfun FTDI Basic GND and CTL must be shorted.  There is no corresponding pin for CTL on OpenTag.
// 8. This code is close to filling code space.  So Serial.Prints are commented.

// 8. Data are streamed to one file at a time.  The file structure follows the datafile.h header, and is generally as follows:
// DF_HEAD: At start of each file. Contains start time, code version number, and values for lat, lon, depth (altitude)
// SID_SPEC: Follows DF_HEAD.  May be more than one SID_SPEC depending on which sensors are being saved.  Contains info on sample period, sensor types, number of channels
// SID_REC: As data buffers are filled, they are written to the file.  Each data chunk starts with a SID_REC which indicates which SID_SPEC type it is, sensors stored, and the number of bytes recorded since the start of recording.


// Revision History
//  beta dfh.Version=10000;
//  dfh.Version=10001; 13 June 2012
//      Added code so nbuffers not incremented in pressure or ADC loops if IMU being saved.  Keeps things aligned.
// dfh.Version=10003; 22 November v1.1; changed accelerometer from +/2g max scale to +/16g max scale

// Sid_SPEC.SensorType Binary flag structure for sensors (sensor = bit shift)
// Analog ADC0 = 0
// temperature = 1
// Pressure = 2
// gyro = 3
// magnetometer = 4
// accelerometer internal = 5
// accelerometer external = 6

// Order Data are written to file (if present)
// "HYD1" HYDRO1, Accel Ext
// "PTMP" Pressure, Temp
// "INER" Accel Int, Comp, Gyro

#include <stdint.h>
#include <SdFat.h> // http://code.google.com/p/sdfatlib/  //note that the SD library that comes with Arduino does not support file timestamps, so not using it
#include <SdFatUtil.h>
#include <Wire.h>
#include <MsTimer2.h>  // Note: MsTimer2.cpp modified so it does not disable interrupts
#include <datafile.h>

// SD chip select pin
const uint8_t CS = SS_PIN;

int BURN=8;
int I2CPOW=9;
int SALTFLIP=7;
int SALT_ADC=A1;
int SD_POW=5;
int LED_GRN=4;
int LED_RED=A3;
int HYDRO1=A0;
//int HYDRO2=A7;
int PRESS=A6;

// I2C Addresses

int AccelAddressInt = 0x53;  //with pin 12 grounded; board accel
int AccelAddressExt = 0x1D;   //with pin 12 tied to Vcc; external accelerometer

// Default Flags...These can be changed by script
boolean accelflagint=0;  //flag to enable accelerometer; 
boolean accelflagext=0;  //flag to enable external accelerometer
boolean compflag=0; //flag to enable 3d magnetometer (compass)
boolean gyroflag=0; //flag to enable gyro
boolean pressflag=0; //flag to enable pressure
boolean tempflag=0; //flag to enable temperature
boolean HYDRO1flag=0; // flag to record HYDRO1
boolean printflag=0; //flag to enable printing of sensor data to serial output

//boolean A7flag=0;
boolean compcalflag=0;
boolean saltflag=0;  //flag to enable salt switch to control recording
boolean stopflag=0;

boolean updatepressflag=0;  //=1 call UpdatePress();  takes 9 ms for conversion
boolean updatetempflag=0;  //=1 call UpdateTemp();  takes 9 ms for conversion

SdFat card;
SdFile file;

char filename[12];
static unsigned long count=0;
#define BUFFERSIZE 252 // used this length because it is divisible by 36 bytes (e.g. Aint,M,G)  //default 144  //IMU only; no ADC  288
byte buffer[BUFFERSIZE]; //Double buffer used to store IMU sensor data before writes in bytes

#define PTBUFFERSIZE 24
byte PTbuffer[PTBUFFERSIZE];  //Double buffer used to store Pressure and Temp at low rate. Note Press and Temp each 3 bytes.

#define HYDRO1BUFFERSIZE 1  //stores hydrophone (2 bytes) and/or external accelerometer (6 bytes)  //default 120
byte HYDRO1buffer[HYDRO1BUFFERSIZE];  //Double buffer used to store HYDRO1 data or external accelerometer

byte time2write=0;  //=0 no write; =1 write first half; =2 write second half
byte time2writePT=0; 
byte time2writeHYDRO1=0;

byte halfbufPT=PTBUFFERSIZE/2;
byte halfbufHYDRO1=HYDRO1BUFFERSIZE/2;
int halfbuf=BUFFERSIZE/2;
int bufferpos=0; //current position in double buffer
byte bufferposPT=0;
byte bufferposHYDRO1=0;

boolean firstwritten;
boolean firstwrittenPT;
boolean firstwrittenHYDRO1;

int speriod=10; //default master sample rate interrupt ms
int iperiod=20; // sample period for motion sensors ms
int mscale_period=iperiod/speriod;  //number of speriods before sample motion sensors
int mscale_counter=0;

int PTperiod=1000; //Press Temp Sample Period in ms
int mscale_PTperiod=PTperiod/speriod;  //sample period for pressure/temperature sensor; number of speriods before sample pressure and temp
int mscale_PTcounter;

int HYDRO1period=10;  //HYDRO1 Sample period in ms
int mscale_HYDRO1period=HYDRO1period/speriod;  //sample period for pressure/temperature sensor; number of speriods before sample pressure and temp
int mscale_HYDRO1counter;

/*
int A7period=10;  //A7 Sample period in ms
int mscale_A7period=A7period/speriod;  //sample period for pressure/temperature sensor; number of speriods before sample pressure and temp
int mscale_A7counter;
*/
unsigned int RTCcounter=0;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400
byte hour;
byte minute;
byte second;
byte year;
byte month;
byte date;

TIME_HEAD burntime;

// interval between timer interrupts in microseconds
const uint16_t TICK_TIME_USEC = 1000;
byte LEDSON=1;
unsigned int counter=0;

int accel_x_int;
int accel_y_int;
int accel_z_int;
int accel_x_ext;
int accel_y_ext;
int accel_z_ext;
int magnetom_x;
int magnetom_y;
int magnetom_z;
int gyro_x;
int gyro_y;
int gyro_z;
int gyro_temp;

byte Tbuff[3];
byte Pbuff[3];

//Pressure and temp calibration coefficients
unsigned int PSENS; //pressure sensitivity
unsigned int POFF;  //Pressure offset
unsigned int TCSENS; //Temp coefficient of pressure sensitivity
unsigned int TCOFF; //Temp coefficient of pressure offset
unsigned int TREF;  //Ref temperature
unsigned int TEMPSENS; //Temperature sensitivity coefficient

// Header for dsg files
DF_HEAD dfh;
SID_SPEC SidSpec[SID_MAX];
SID_REC SidRec[SID_MAX];

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//
// ------  Setup   --------
//
void setup() {                
  pinMode(BURN, OUTPUT);    
  pinMode(I2CPOW, OUTPUT);    
  pinMode(SALTFLIP, OUTPUT);    
  pinMode(SD_POW, OUTPUT);    
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(SALT_ADC, INPUT); 
  digitalWrite(SALT_ADC, HIGH); //set pullup resistor on Salt ADC
  pinMode(A2, INPUT);  //used to detect stop
  digitalWrite(A2, HIGH);
  
  // set prescale to 8 for ADC to allow faster reads
  cbi(ADCSRA,ADPS2) ;
  sbi(ADCSRA,ADPS1) ;
  sbi(ADCSRA,ADPS0) ;
  
  // wait in sleeping loop until salt switch ON
  int salty;
  if(saltflag)
  {
     do{
        salty=amiwet();
       // Serial.println(salty);
     } while(salty>1000); 
  }
  
  digitalWrite(SD_POW, HIGH);  //turn on power to SD card.  High for OpenTag.
  digitalWrite(BURN,LOW);
  digitalWrite(SALTFLIP, LOW);
  digitalWrite(I2CPOW, LOW);
    
  //Serial.begin(57600); 
  //PgmPrint("LHI Open Tag");
  //PgmPrint("Ram ");
  //Serial.println(FreeRam());

  // initialize the SD card at SPI_FULL_SPEED for best performance.
  if (!card.init(SPI_FULL_SPEED, CS)) card.initErrorHalt();
  
  LoadScript(file);
  mscale_period=iperiod/speriod;
  mscale_PTperiod=PTperiod/speriod/(pressflag+tempflag);
  mscale_HYDRO1period=HYDRO1period/speriod;
  
  I2C_Init();

  if (pressflag|tempflag) 
  {
    Press_Init(); 
    Update_Press();
  }
  if (gyroflag) 
  {
    Gyro_Init();
  }
  if (compflag)
  {
    Compass_Init();
    Read_Compass();
  }
  if(accelflagext)
  {
  //   do
   //  {
        Accel_Init(AccelAddressExt,(float) 1000/iperiod,0);
        Read_Accel(AccelAddressExt);
    
 //   } while (accel_x_ext==0);
  }
  if(accelflagint)
  {
  //   do
  //   {
       Accel_Init(AccelAddressInt,(float) 1000/iperiod,0);
       Read_Accel(AccelAddressInt);
    
  //   } while (accel_x_int==0);
  }

  for (int n=0; n<SID_MAX; n++)
  {
    SidSpec[n].SID=0;
    SidRec[n].nbytes=0;
    SidRec[n].nbytes_2=0;
  }

  dfh.Version=10003;
  dfh.UserID=3333;
  
  if(pressflag|tempflag)
  {
    // Write out pressure and temperature calibration values to a file
   if(file.open("presstmp.cal", O_CREAT | O_EXCL | O_WRITE))
   {
      Read_RTC();  //update time

      file.timestamp(T_WRITE,(uint16_t) year+2000,month,date,hour,minute,second);

      file.write(&PSENS,2);    
      file.write(&POFF,2);     
      file.write(&TCSENS,2);  
      file.write(&TCOFF,2);  
      file.write(&TREF,2);  
      file.write(&TEMPSENS,2);     
      file.close();
    }
  }
    // turn off LEDS
   digitalWrite(LED_RED, LOW);
   digitalWrite(LED_GRN, LOW);
   
   //Start Timer for Sensor Reading
   MsTimer2::set(speriod, flash); // 5ms period=200 Hz
   MsTimer2::start();
   count=0;
}

//
// ------  Main Loop   --------
//
void loop() {

 int nSid, nSid0, nSid1, nSid2; 
 int bytes_read=0; //Keeps track of how many bytes are read when accessing a file on the SD card.
 unsigned int buffersperfile=2880;
 unsigned int nbuffers=0;
 int bufsync=0;
 
 //check if still wet everytime a new file is going to be started
  if(saltflag)
  {
     int salty=amiwet();
     if(salty>1000) softreset();  //dry if salty is >1000
  }
 
 sprintf(filename,"m%d.dsg",count);
  //Create a file. If the file already exists, increment counter and try again.
  while(!file.open(filename, O_CREAT | O_EXCL | O_WRITE))
  {
   count++; 
   sprintf(filename,"m%d.dsg",count);
   if(LEDSON)  digitalWrite(LED_RED,HIGH);
  }
  
  //Read_RTC();  //update time
  file.timestamp(T_CREATE,(uint16_t) year+2000,month,date,hour,minute,second);
    
  //Write file header
  boolean rv;
  updatetime(&dfh.RecStartTime);
  file.write(&dfh, sizeof(DF_HEAD));
  nSid=0;  //reset Sid ID counter   
  if(accelflagint|gyroflag|compflag)
  {
        SidRec[0].nSID=AddSid(nSid,"INER", halfbuf, EVTYPE_STREAM, DFORM_SHORT, iperiod,(accelflagint*3)+(gyroflag*3)+(compflag*3),accelflagint<<5 | compflag<<4 | gyroflag<<3);
        SidRec[0].Chan = (accelflagint<<5 | compflag<<4 | gyroflag<<3);
        nSid++;  
  }
   if(HYDRO1flag | accelflagext)
  {
	SidRec[1].nSID=AddSid(nSid,"HYD1", halfbufHYDRO1, EVTYPE_STREAM, DFORM_SHORT, HYDRO1period,HYDRO1flag+(accelflagext*3),accelflagext<<6 | HYDRO1flag);  //text description, #bytes,data type, data format (used to calculate #samples from bytes), speriod*256 in us, nchan,sensor bitcode
        SidRec[1].Chan=(accelflagext<<6 | HYDRO1flag);
        nSid++;
  }
    if(tempflag|pressflag)
  {
        SidRec[2].nSID=AddSid(nSid,"PTMP", halfbufPT, EVTYPE_STREAM, DFORM_I24, PTperiod,(tempflag)+(pressflag), tempflag<<1 | pressflag<<2);
        SidRec[2].Chan = (tempflag<<1 | pressflag<<2);
        nSid++;  
  }

  /*
  if(A7flag)
  {
	SidRec[3].nSid=AddSid(nSid,"ADC7", halfbuf, EVTYPE_STREAM, DFORM_SHORT, A7period*256000*mscale_A7period,1,0);  //text description, #bytes,data type, data format (used to calculate #samples from bytes), speriod*256 in us, nchan,sensor bitcode
        nSid++;
  }
  */
  AddSid(nSid, "DONE", 0, EVTYPE_STREAM, DFORM_SHORT, 1, 0, 0);  //last SID_SPEC with 0 for nbytes to indicate end of SID_SPEC headers
    

  
  if(LEDSON) digitalWrite(LED_RED,LOW);

  while (nbuffers<buffersperfile)
  {

     if(time2write==1)
     {
        if(LEDSON) digitalWrite(LED_GRN,HIGH);   
               
        file.write(&SidRec[0],sizeof(SID_REC));
        file.write(buffer, halfbuf);
        
        UpdateSID_REC(0,halfbuf);  // update SID_REC with number of bytes written
        
        time2write=0;
        if(LEDSON) digitalWrite(LED_GRN,LOW);
      }
        
      if(time2write==2)
      {
        if(LEDSON) digitalWrite(LED_GRN,HIGH);
                
        file.write(&SidRec[0],sizeof(SID_REC));
        file.write((const void*)&buffer[halfbuf], halfbuf);
        
        UpdateSID_REC(0,halfbuf);  // update SID_REC with number of bytes written      
        
        time2write=0;
        if(LEDSON) digitalWrite(LED_GRN,LOW);
        nbuffers++;
        bufsync++;
       }    
        
      if(time2writePT==1)
      {
        if(LEDSON) digitalWrite(LED_GRN,HIGH);
  
        file.write(&SidRec[2],sizeof(SID_REC));
        file.write(PTbuffer, halfbufPT);
        
        UpdateSID_REC(2,halfbufPT);  // update SID_REC with number of bytes written
                
        time2writePT=0;
        if(LEDSON) digitalWrite(LED_GRN,LOW);
      }
      
      if(time2writePT==2)
      {
        if(LEDSON) digitalWrite(LED_GRN,HIGH);

        file.write(&SidRec[2],sizeof(SID_REC));
        file.write((const void*)&PTbuffer[halfbufPT], halfbufPT);
        
        UpdateSID_REC(2,halfbufPT);  // update SID_REC with number of bytes written       
        
        time2writePT=0;
        if(LEDSON) digitalWrite(LED_GRN,LOW);
        if(!(accelflagint|gyroflag|compflag))  //only increment if no IMU
        {
          nbuffers++;
          bufsync++;
        }
      }    
     
      if(time2writeHYDRO1==1)
      {
        if(LEDSON) digitalWrite(LED_GRN,HIGH);
      
        file.write(&SidRec[1],sizeof(SID_REC));
        file.write(HYDRO1buffer, halfbufHYDRO1);
        
        UpdateSID_REC(1,halfbufHYDRO1);  // update SID_REC with number of bytes written
        
        time2writeHYDRO1=0;
        if(LEDSON) digitalWrite(LED_GRN,LOW);
      }
      
      if(time2writeHYDRO1==2)
      {
        if(LEDSON) digitalWrite(LED_GRN,HIGH);

        file.write(&SidRec[1],sizeof(SID_REC));
        file.write((const void*)&HYDRO1buffer[halfbufHYDRO1], halfbufHYDRO1);
        
        UpdateSID_REC(1,halfbufHYDRO1);  // update SID_REC with number of bytes written
        
        time2writeHYDRO1=0;
        if(LEDSON) digitalWrite(LED_GRN,LOW);
        if(!(
        accelflagint|gyroflag|compflag))  //only increment if no IMU
        {
          nbuffers++;
          bufsync++;
        }
      }    
      
     // sync every 2048 bytes
     if(bufsync>=20)
      {
        bufsync=0;
       // file.sync();
        
        //check to see if time to stop
        int stopval=analogRead(A2);
        if(stopval<20) 
          stopflag=1;
      }
      if(stopflag) break;  //end recording
  }
  file.timestamp(T_WRITE,(uint16_t) year+2000,month,date,hour,minute,second);
  file.close();
  if(stopflag) 
  {
     digitalWrite(LED_RED, HIGH);
     delay(30000);  //give 30 seconds to turn off power, before continuing to record
     digitalWrite(LED_RED, HIGH);
     stopflag=0;
  }
  count++;
  nbuffers=0;
}

// increment buffer position by 1 byte.  check for end of buffer
void incrementbufpos(){
   boolean overflow;
   bufferpos++;
   if(bufferpos==BUFFERSIZE)
   {
     bufferpos=0; 
     if(time2write==1) //check for overflow--convert this to lighting LED
     { 
       overflow=1;
      if(LEDSON) digitalWrite(LED_RED,HIGH);
      //  delay(1);  
     }
     else
     {
       overflow=0;
        if(LEDSON) digitalWrite(LED_RED,LOW);
      }
    time2write=2;  // set flag to write second half
    firstwritten=0; 
 }
 
  if((bufferpos>=halfbuf) & !firstwritten)  //at end of first buffer
  {
    if(time2write==2)
      {  
        overflow=1;
        if(LEDSON) digitalWrite(LED_RED,HIGH);
      }
    else
    {
        overflow=0;
        if(LEDSON) digitalWrite(LED_RED,LOW);
    }
    time2write=1; 
    firstwritten=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

// increment buffer position by 1 byte.  check for end of buffer
void incrementHYDRO1bufpos(){
   boolean overflow;
   bufferposHYDRO1++;
   if(bufferposHYDRO1==HYDRO1BUFFERSIZE)
   {
     bufferposHYDRO1=0; 
     if(time2writeHYDRO1==1) //check for overflow--convert this to lighting LED
     { 
       overflow=1;
      if(LEDSON) digitalWrite(LED_RED,HIGH);
      //  delay(1);  
     }
     else
     {
       overflow=0;
        if(LEDSON) digitalWrite(LED_RED,LOW);
      }
    time2writeHYDRO1=2;  // set flag to write second half
    firstwrittenHYDRO1=0; 
 }
 
  if((bufferposHYDRO1>=halfbufHYDRO1) & !firstwrittenHYDRO1)  //at end of first buffer
  {
    if(time2writeHYDRO1==2)
      {  
        overflow=1;
        if(LEDSON) digitalWrite(LED_RED,HIGH);
      }
    else
    {
        overflow=0;
        if(LEDSON) digitalWrite(LED_RED,LOW);
    }
    time2writeHYDRO1=1; 
    firstwrittenHYDRO1=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

// increment PTbuffer position by 1 byte. This does not check for overflow, because collected at a slow rate
void incrementPTbufpos(){
  boolean overflow;
  bufferposPT++;
   if(bufferposPT==PTBUFFERSIZE)
   {
     bufferposPT=0;
     time2writePT=2;  // set flag to write second half
     firstwrittenPT=0; 
   }
 
  if((bufferposPT>=halfbufPT) & !firstwrittenPT)  //at end of first buffer
  {
    time2writePT=1; 
    firstwrittenPT=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

/********************************************************************
***        Master Interrupt Routine to Read Sensors               ***
********************************************************************/
void flash(){  
  mscale_counter++;
  mscale_PTcounter++;
  mscale_HYDRO1counter++;
 // mscale_A7counter++;
  RTCcounter++;

  if(RTCcounter>speriod*2000)  //reading here because interrupt blocks reads outside this loop sometimes
  {
    Read_RTC();
    RTCcounter=0;
  }

  if(HYDRO1flag | accelflagext)
  {
    if(mscale_HYDRO1counter>=mscale_HYDRO1period)
    {
        if (HYDRO1flag)
        {
          int sensorValue = analogRead(HYDRO1);
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)sensorValue;  
          incrementHYDRO1bufpos();
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)sensorValue>>8;  
          incrementHYDRO1bufpos();
        }
        if(accelflagext)  //Write external acceleromter values to buffer
        {      
         // Read_IMU(AccelAddressExt, 0x32, 6, 0);
          
          Read_Accel(AccelAddressExt);
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)accel_x_ext;  
          incrementHYDRO1bufpos();
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)accel_x_ext>>8;  
          incrementHYDRO1bufpos();
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)accel_y_ext;
          incrementHYDRO1bufpos();
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)accel_y_ext>>8;
          incrementHYDRO1bufpos(); 
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)accel_z_ext;
          incrementHYDRO1bufpos();
          HYDRO1buffer[bufferposHYDRO1]=(unsigned int)accel_z_ext>>8;
          incrementHYDRO1bufpos();   
        
        }
        mscale_HYDRO1counter=0;
    } 
  }

  if(mscale_PTcounter>=mscale_PTperiod)  //alternate reading temperature and pressure
  {
    updatepressflag=~updatepressflag;
    if(pressflag & updatepressflag)
    {
      Read_Press();  //read current value of pressure and temperature
      PTbuffer[bufferposPT]=Pbuff[0];
      incrementPTbufpos();
      PTbuffer[bufferposPT]=Pbuff[1];
      incrementPTbufpos();
      PTbuffer[bufferposPT]=Pbuff[2];
      incrementPTbufpos();
      Update_Temp();
    }
    else
    {
      if(tempflag )
      {
        Read_Temp();
        PTbuffer[bufferposPT]=Tbuff[0];
        incrementPTbufpos();
        PTbuffer[bufferposPT]=Tbuff[1];
        incrementPTbufpos();
        PTbuffer[bufferposPT]=Tbuff[2];
        incrementPTbufpos();
        Update_Press();
      }
    }
    mscale_PTcounter=0;
  }
 
    if(mscale_counter>=mscale_period)
    {
      
        //Write acceleromter values to buffer
        if(accelflagint)
        {      
         Read_Accel(AccelAddressInt);
         // Read_IMU(AccelAddressInt, 0x32, 6, 0);
          buffer[bufferpos]=(unsigned int)accel_x_int;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_x_int>>8;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_y_int;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_y_int>>8;
          incrementbufpos(); 
          buffer[bufferpos]=(unsigned int)accel_z_int;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)accel_z_int>>8;
          incrementbufpos();
          
        }
        
        // Write Magnetometer to buffer
        if(compflag)
        {
          //Read_IMU(CompassAddress, 0x03, 6, 1);
          
          Read_Compass();
          buffer[bufferpos]=(unsigned int)magnetom_x;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_x>>8;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_y;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_y>>8;
          incrementbufpos(); 
          buffer[bufferpos]=(unsigned int)magnetom_z;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)magnetom_z>>8;
          incrementbufpos();
          
        }
      
        // Write Gyros to buffer
        if(gyroflag)
        {
          //Read_IMU(GyroAddress, 0x1D, 6, 1);
          Read_Gyro();
          buffer[bufferpos]=(unsigned int)gyro_x;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_x>>8;  
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_y;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_y>>8;
          incrementbufpos(); 
          buffer[bufferpos]=(unsigned int)gyro_z;
          incrementbufpos();
          buffer[bufferpos]=(unsigned int)gyro_z>>8;
          incrementbufpos();
        }
      mscale_counter=0;
  }
}

void updatetime(TIME_HEAD *tm)
{ 
   	tm->sec=second;
	tm->minute=minute;  
	tm->hour=hour;  
	tm->day=1;  
	tm->mday=date;  
	tm->month=month;  
	tm->year=year;  
	tm->timezone=0;  
  
}

// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(TIME_HEAD *tm){
	int i;
	unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
	unsigned long Ticks = 0;

	long yearsSince = tm->year+30; // Same as tm->year + 2000 - 1970
	long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated
	
	// Calculate Year Ticks
	Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
	Ticks += numLeaps * SECONDS_IN_LEAP;

	// Calculate Month Ticks
	for(i=0; i < tm->month-1; i++){
	     Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
	}

	// Calculate Day Ticks
	Ticks += tm->mday * SECONDS_IN_DAY;
	
	// Calculate Time Ticks CHANGES ARE HERE
	Ticks += (ULONG)tm->hour * SECONDS_IN_HOUR;
	Ticks += (ULONG)tm->minute * SECONDS_IN_MINUTE;
	Ticks += tm->sec;

	return Ticks;
}

int AddSid(int i, char* sid, unsigned long nbytes, unsigned long storetype, unsigned long dform, unsigned long SPus, unsigned long numchan, unsigned long sensors)
{
	unsigned long _sid, nelements;
	memcpy(&_sid, sid, 4);

	memset(&SidSpec[i], 0, sizeof(SID_SPEC));
        nbytes<<1;  //multiply by two because halfbuf

	switch(dform)
	{
		case DFORM_SHORT:
			nelements = nbytes>>1;
			break;
             
		case DFORM_LONG:
			nelements = nbytes/4;  //32 bit values
			break;
             
		case DFORM_I24:
			nelements = nbytes/3;  //24 bit values
			break;
	}

	SidSpec[i].SID = _sid;
	SidSpec[i].nBytes = nbytes;
	SidSpec[i].StoreType = storetype;
	SidSpec[i].DForm = dform;
	SidSpec[i].SPus = SPus*1000;  //convert from ms to us
	SidSpec[i].RECPTS = nelements;
	SidSpec[i].RECINT = nelements;
	SidSpec[i].NumChan = numchan;
        SidSpec[i].SensorType = sensors;	
	
	file.write(&SidSpec[i], sizeof(SID_SPEC));
       	return i;
}

void UpdateSID_REC(int buf, int nbytes)
{
   unsigned long newcount=SidRec[buf].nbytes + nbytes; 
   if(newcount<SidRec[buf].nbytes) //check for rollover and use TS256_2 to count rollovers
       SidRec[buf].nbytes_2++;
   SidRec[buf].nbytes=newcount;  //this is total sample points from when recorder started (not reset to 0 when new files started)
}

int amiwet()
{
    digitalWrite(SALTFLIP, HIGH);  //send out pulse
    int salty=analogRead(SALT_ADC);
    digitalWrite(SALTFLIP, LOW);
    return salty;
}

void softreset()
{
}

