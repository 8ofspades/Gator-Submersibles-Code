
// m11 Sensor Test
// Disabled writing to disk

#include <stdint.h>
// http://code.google.com/p/sdfatlib/
#include <SdFat.h>
#include <SdFatUtil.h>

#include <Wire.h>
#include <MsTimer2.h>
// Note: MsTimer2.cpp  add sei(); to ISR(TIMER2_OVF_vect) at end of code, so doesn't disable interrupts 
// Otherwise, will mess up I2C calls from within interrupt routine
//#include <TimerTwo.h>

// SD chip select pin
const uint8_t CS = SS_PIN;

int BURN=8;
int I2CPOW=9;
int SALTFLIP=7;
int SALT_ADC=A1;
int SD_POW=5;
int LED_GRN=A3;
int LED_RED=4;
int HYDRO1=A0;
int HYDRO2=A7;
int PRESS=A6;

boolean READFLAG;  //set to 1 by interrupt service routine

SdFat card;
//SdVolume volume;
//SdFile root;
SdFile file;
//This is the double buffer used to store sensor data before writes in bytes
#define BUFFERSIZE	512

byte buffer[BUFFERSIZE];
byte time2write=0;  //=0 no write; =1 write first half; =2 write second half
int halfbuf=BUFFERSIZE/2;
int bufferpos=0; //current position in double buffer
boolean firstwritten;
int speriod=2; //sample period for ADC0
int mscale_period=100;  //sample period for motion sensors; number of speriods before sample motion sensors
int mscale_counter;

byte hour;
byte minute;
byte second;
byte year;
byte month;
byte date;

// interval between timer interrupts in microseconds
const uint16_t TICK_TIME_USEC = 1000;
byte LEDSON=1;
unsigned int counter=0;

int AccelAddressInt = 0x53;  //with pin 12 grounded; board accel
int AccelAddressExt = 0x1D;   //with pin 12 tied to Vcc; external accelerometer

int SENSOR_SIGN[9] = {1,1,1,1,1,1,-1,-1,-1};  //Correct directions x,y,z - gyros, accels, magnetormeter

int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int ACC[3];          //array that store the accelerometers data
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

//Pressure and temp setup
unsigned int PSENS; //pressure sensitivity
unsigned int POFF;  //Pressure offset
unsigned int TCSENS; //Temp coefficient of pressure sensitivity
unsigned int TCOFF; //Temp coefficient of pressure offset
unsigned int TREF;  //Ref temperature
unsigned int TEMPSENS; //Temperature sensitivity coefficient


void setup() {                
  pinMode(BURN, OUTPUT);    
  pinMode(I2CPOW, OUTPUT);    
  pinMode(SALTFLIP, OUTPUT);    
  pinMode(SD_POW, OUTPUT);    
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(SD_POW, HIGH);  //turn on power to SD card
  digitalWrite(BURN,LOW);
  
  digitalWrite(LED_GRN,HIGH);
  digitalWrite(LED_RED,HIGH);
  
  //analogReference(INTERNAL);
  
  Serial.begin(57600); 
  Serial.println("LHI m11");
    PgmPrint("FreeRam: ");
  Serial.println(FreeRam());
  Serial.println("I2C init");
  I2C_Init();
  Serial.println("Compass init");
  Read_Compass();
  Serial.println("Press init");
  Press_Init(); 
  Serial.println("Gyro init");  
  Gyro_Init();
  Compass_Init();

    Serial.println("RTC init"); 
  // RTC_Init();
 setTime2(0,0,0,1,1,2012); //1 Jan 2012 00:00:00
  Read_RTC();
  
  
    Accel_Init(AccelAddressInt, 500, 0); //initialize at 500 Hz
    Read_Accel(AccelAddressInt);

/*
  //Set up the pins for the microSD shield
  pinMode(CS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
*/
  
  // initialize the SD card at SPI_FULL_SPEED for best performance.
  // try SPI_HALF_SPEED if bus errors occur.
//  if (!card.init(SPI_FULL_SPEED,CS)) error("card.init fail");
    if (!card.init(SPI_FULL_SPEED, CS)) 
      Serial.println("Card init fail");
  // initialize a FAT volume
//  if (!volume.init(&card)) error("volume.init fail");
  
 //  if (!root.openRoot(&volume)) error("openRoot fail");
  
 //   Serial.println("SD card initialized.");
    
    // Find last filename
    
    
  // set tick time
/*  if (!TimerTwo::init(TICK_TIME_USEC)
    || TICK_TIME_USEC != TimerTwo::period()) {
    // TICK_TIME_USEC is too large or period rounds to a different value
    error("TimerTwo::init");
  }
  */

  
  //Start Timer for Sensor Reading
//   TimerTwo::start();
   mscale_counter=0;
   MsTimer2::set(speriod, flash); // 5ms period=200 Hz
   MsTimer2::start();
   READFLAG=1;
   //   Serial.println("Sampling Started");
}

void loop() {

 char filename[12];
 int bytes_read=0; //Keeps track of how many bytes are read when accessing a file on the SD card.
 static unsigned long count=0;
 unsigned int nbuffers=0;
  unsigned long time;
  unsigned long oldtime;
  int bufsync=0;

 //sprintf(filename,"m%d.i16",count);
   
 Read_RTC();  //update time
 /* //Create a file. If the file already exists, increment counter and try again.
  while(!file.open(filename, O_CREAT | O_EXCL | O_WRITE))
  {
   count++; 
   sprintf(filename,"m%d.i16",count);
    digitalWrite(LED_RED,HIGH);
  }
  file.timestamp(T_CREATE,
		(uint16_t) year+2000,
		month,
		date,
		hour,
		minute,
		second 
	);
 */
  digitalWrite(LED_RED,LOW);
 
  //file.write(&count, 1);
  //collect data for 1000 buffers

  while (nbuffers<10)  //keep this a multiple of 3
  {
    if(READFLAG)
    {
       Read_Compass();    // Read I2C magnetometer
       Read_Gyro();
       Read_Press();
       Read_Accel(AccelAddressInt);
       
       print_sensors();
       Update_Press();  //this will start next conversion which takes ~9 ms
    
       READFLAG=0;
     }
       if(time2write==1)
        {
          digitalWrite(LED_GRN,HIGH);
         // fat_write_file(file_handle, (const uint8_t*)buffer, halfbuf);
       //  file.write(buffer, halfbuf);
          time2write=0;
          digitalWrite(LED_GRN,LOW);
        }
        
        if(time2write==2)
        {
          digitalWrite(LED_GRN,HIGH);
        //  fat_write_file(file_handle, (const uint8_t*)buffer+halfbuf, halfbuf);
        //  file.write((const void*)&buffer[halfbuf], halfbuf);
          time2write=0;
          digitalWrite(LED_GRN,LOW);
          nbuffers++;
          bufsync++;
        }    
        
        // sync every 2048 bytes
       if(bufsync>=10)
        {
          bufsync=0;
       //   file.sync();
          
        }

  }
 
  file.close();
  count++;
  nbuffers=0;

  digitalWrite(LED_GRN, HIGH);  
  digitalWrite(LED_RED,LOW);
  digitalWrite(BURN,HIGH);
  delay(200);             
    
   //READ ADC Values
  readandpost(SALT_ADC);
  readandpost(PRESS);
  readandpost(HYDRO1);
  readandpost(HYDRO2);
  
  digitalWrite(LED_GRN, LOW);   
  digitalWrite(LED_RED,HIGH);
  digitalWrite(BURN,LOW);
  delay(200);             

  //READ ADC Values
  readandpost(SALT_ADC);
  readandpost(PRESS);
  readandpost(HYDRO1);
  readandpost(HYDRO2);

}


void readandpost(int ch)
{
   int sensorValue = analogRead(ch);
   Serial.print("Ch ");
   Serial.print(ch);
   Serial.print(": ");
   Serial.println(sensorValue);
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
        digitalWrite(LED_RED,HIGH);
      //  delay(1);

 
      }
        else
        {
        overflow=0;
        digitalWrite(LED_RED,LOW);
        }

    time2write=2;  // set flag to write second half
    firstwritten=0; 
 }
 
  if((bufferpos>=halfbuf) & !firstwritten)  //at end of first buffer
  {
    if(time2write==2)
      {  
        overflow=1;
        digitalWrite(LED_RED,HIGH);
      //  delay(1);
      }
    else
    {
        overflow=0;
        digitalWrite(LED_RED,LOW);
    }
    time2write=1; 
    firstwritten=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
 
}


//ISR(TIMER2_COMPA_vect){
void flash(){  
/*  
  static int testcounter=-10000;
  testcounter+=10;
  buffer[bufferpos]=(unsigned int)testcounter;  
  incrementbufpos();
  buffer[bufferpos]=(unsigned int)testcounter>>8;  
  incrementbufpos();
  */


  int sensorValue = analogRead(HYDRO1);
  buffer[bufferpos]=(unsigned int)sensorValue;  
  incrementbufpos();
  buffer[bufferpos]=(unsigned int)sensorValue>>8;  
  incrementbufpos();
  
//  Serial.println(sensorValue);
  
  mscale_counter++;
  
  if(mscale_counter>=mscale_period/speriod)
  {

      // Write Accelerometer
      // note I2C originally returns these as bytes.  so could run a little faster if left as bytes
        
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
    
      // Write Magnetometer
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
    
      // Write Gyros
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
    
      mscale_counter=0;
      READFLAG=1;
  }
}

void print_sensors()
{
  Serial.print("Mag ");
  Serial.print(magnetom_x);
  Serial.print(" ");
  Serial.print(magnetom_y);
  Serial.print(" ");
  Serial.print(magnetom_z);
  Serial.print(" Com ");
  
 float heading;
 heading=atan2((float)magnetom_y,(float)magnetom_x);
 heading=heading*57.2957795;
 Serial.print(heading);

  Serial.print("  G ");
  Serial.print(gyro_x);
  Serial.print(" ");
  Serial.print(gyro_y);
  Serial.print(" ");
  Serial.print(gyro_z);
  Serial.print("  ");
  
    Serial.print("A ");
  Serial.print(accel_x_int);
  Serial.print(" ");
  Serial.print(accel_y_int);
  Serial.print(" ");
  Serial.print(accel_z_int);
  Serial.print("\n");
}
