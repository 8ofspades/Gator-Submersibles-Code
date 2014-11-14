/* ******************************************************* */
/* I2C code for external sensors                           */
/* ******************************************************* */

int CompassAddress = 0x1E;  //0x3C //0x3D;  //(0x42>>1);
int GyroAddress = 0x69;
int RTCAddress = 0x68;
int PressAddress = 0x77;

void I2C_Init()
{
 Wire.begin();
}

void Press_Init()
{
  int i = 0;
  byte buff[2];
  int bytesread;
  
  // Reset so PROM is loaded
  Wire.beginTransmission(PressAddress);
  Wire.send(0x1E);  //Reset Command
  bytesread=Wire.endTransmission();
  //Serial.println("Pressure Reset 0=success");
  //Serial.println(bytesread);  
    
  delay(5);  //reset needs at least 2.8 ms
  
  // Read and store calibration coefficients
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA2);  //PROM Read Pressure Sensitivity
  Wire.endTransmission();
  
  bytesread=Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
//  Serial.println("PROM Bytes Available");
//  Serial.println(bytesread);
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
 // Wire.endTransmission();
  PSENS=(unsigned int) (buff[0]<<8)|buff[1]; //pressure sensitivity  MSB first
    
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA4);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
 // Wire.endTransmission();
  POFF=(buff[0]<<8)|buff[1];  //Pressure offset
 
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA6);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
//  Wire.endTransmission();
  TCSENS=(buff[0]<<8)|buff[1];  //

  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xA8);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
//  Wire.endTransmission();
  TCOFF=(buff[0]<<8)|buff[1];  //Temp coefficient of pressure offset
  
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xAA);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
 // Wire.endTransmission();
  TREF=(buff[0]<<8)|buff[1];  //Ref temperature
  
  i=0;
  Wire.beginTransmission(PressAddress);
  Wire.send(0xAC);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(PressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
 // Wire.endTransmission();
  TEMPSENS =(buff[0]<<8)|buff[1];  //Temperature sensitivity coefficient  

/*
  Serial.println("P Sensitivity");
  Serial.println((unsigned int)PSENS);
  Serial.println("P Offset");
  Serial.println((unsigned int)POFF);
  Serial.println("T Coeff P Sens");
  Serial.println((unsigned int)TCSENS);
  Serial.println("T Coeff of Press Off");
  Serial.println((unsigned int)TCOFF);
  Serial.println("Ref T");
  Serial.println((unsigned int)TREF);
  Serial.println("T Sens");
  Serial.println((unsigned int)TEMPSENS);
*/
}

void Update_Press()
{
    Wire.beginTransmission(PressAddress);
    Wire.send(0x48);  //Initiate pressure conversion OSR-4096
    Wire.endTransmission();
}

void Read_Press()
{
  int i = 0;

  
  Wire.beginTransmission(PressAddress);
  Wire.send(0x00);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(PressAddress, 3);    // request 3 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Pbuff[i] = Wire.receive();  // receive one byte
    i++;
  }
  //Wire.endTransmission();
 // Serial.println((buff[0]<<16)|(buff[1]<<8)|buff[2]);
  
  Wire.beginTransmission(PressAddress);
  Wire.send(0x58); //Initiate Temperature conversion OSR=4096
  Wire.endTransmission();
  delay(9);
  Wire.beginTransmission(PressAddress);
  Wire.send(0x00);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(PressAddress, 3);    // request 3 bytes from device
  i=0;
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Tbuff[i] = Wire.receive();  // receive one byte
    i++;
  }

  float D1=(float)((((unsigned long)Pbuff[0]<<16) | ((unsigned long)Pbuff[1]<<8) | ((unsigned long)Pbuff[2])));
  float D2=(float)((((unsigned long)Tbuff[0]<<16) | ((unsigned long)Tbuff[1]<<8) | ((unsigned long)Tbuff[2])));
 
  float T16;
  float P16;

  float dT=D2-(float)TREF*256;
  T16= (2000+dT*(float)TEMPSENS/(float)8388608); //round to 16-bit
	  
  float OFF=(float)POFF*65536+((float)TCOFF*dT)/128;  //floating point approximation of calculation
  float SENS=(float)PSENS*32768+(dT*(float)TCSENS)/256;

  P16=(D1*SENS/2097152-OFF)/81920;  // mbar (i.e. a value of 1 = 1 mbar = ~1 cm depth resolution) 

 Serial.println(T16);
 Serial.println(P16);
 // Wire.endTransmission();
}

void Accel_Init(int AccelAddress, float srate, boolean lowpower)
{
 Wire.beginTransmission(AccelAddress);
  Wire.send(0x2D);  // power register
  Wire.send(0x08);  // measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x31);  // Data format register
  Wire.send(0x08);  // set to full resolution
  Wire.endTransmission();
  delay(5);	

// Rate codes
// 800 Hz: 1101 0x0D 13
// 400 Hz: 1100 0x0C 12
// 200 Hz: 1011 0x0B 11
// 100 Hz: 1010 0x0A 10
// 50 Hz: 1001  0x09  9
// 25 Hz: 1000  0x08  8

// Bit 5 low power mode (lower power, but higher noise) 0=normal
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2C);  // Rate
  // set to next higher sample rate on compass
 int ratecode=8;  //default to 25 Hz
 if (srate>25) ratecode=9;
 if (srate>50) ratecode=10;
 if (srate>100) ratecode=11;
 if (srate>200) ratecode=12;
 if (srate>400) ratecode=13;
 ratecode = ratecode | (lowpower<<4);
  Wire.send(ratecode );  // set to 800Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x,y and z accelerometer registers
void Read_Accel(int AccelAddress)
{
 int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(AccelAddress); 
  Wire.send(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
//  Wire.beginTransmission(AccelAddress); //start transmission to device
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
//  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
      if(AccelAddress==0x53)
      {
        accel_x_int = (((int)buff[1]) << 8) | buff[0];    // X axis (internal sensor x axis)
        accel_y_int = (((int)buff[3]) << 8) | buff[2];    // Y axis (internal sensor y axis)
        accel_z_int = (((int)buff[5]) << 8) | buff[4];    // Z axis
      }
      else
      {
        accel_x_ext = (((int)buff[1]) << 8) | buff[0];    // X axis (internal sensor x axis)
        accel_y_ext = (((int)buff[3]) << 8) | buff[2];    // Y axis (internal sensor y axis)
        accel_z_ext = (((int)buff[5]) << 8) | buff[4];    // Z axis
      }
    }
  //else
   // Serial.println("ERR:Acc");
}

void Compass_Init()
{
  
  // default Sensor Range +/- 1.3 Ga.  LSB/Gauss = 1090
  
  Wire.beginTransmission(CompassAddress);
  // Configuration Register A (Register address 0x00)
  // Bit 7: 0
  // Bit 6-5: Samples Averaged b00=1; b01=2; b10=4; b11=8 (default)
  // Bit 4-2: Data Output Rate Bits b100=15 Hz (default); b101=30 Hz; b110=75 Hz.  
  //          Higher output rate (160 Hz) can be achieved by checking DRDY interrupt
  // Bit 1-0: Measurement configuration
  //          b00=Normal (default); b01=Positive bias for cal (use 0x11); b10=Negative bias for cal (use 0x12)
  Wire.send(0x00);  //Register address 0
  Wire.send(0x78);   // average 8 samples, 75 Hz rate, no bias


 // Configuration Register B (Register address 0x01)
 // Set Gain
  Wire.send(0x20); //set to +/- 1.3 Ga (1090 LSB/Gauss)

  // Set Measurement Mode
  // Register address 0x02
  // 0x00 : Continuous measurement mode (~100 uA draw)
  // 0x01 : Single Measurement mode
  // 0x02 : Idle Mode (2 uA draw)
  Wire.send(0x00);   // Set continuous mode (default to 15Hz)
  Wire.endTransmission(); //end transmission
 
}
  
void Read_Compass()
{
 // HMC5883L stores X, Z, Y
 // Compass data will be -4096 if there is an overflow
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
 // Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device

  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
 // Wire.endTransmission(); //end transmission
  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Z,Y (order of reads from compass)
    magnetom_z = ((((int)buff[2]) << 8) | buff[3]);    
    magnetom_x = ((((int)buff[0]) << 8) | buff[1]);    
    magnetom_y = ((((int)buff[4]) << 8) | buff[5]);    
    }
 // else
  //  Serial.println("ERR:Mag");
   
}
/*
void Compass_Cal()
{
    
  Wire.beginTransmission(CompassAddress);
  // Configuration Register A (Register address 0x00)
  // Bit 7: 0
  // Bit 6-5: Samples Averaged b00=1; b01=2; b10=4; b11=8 (default)
  // Bit 4-2: Data Output Rate Bits b100=15 Hz (default); b101=30 Hz; b110=75 Hz.  
  //          Higher output rate (160 Hz) can be achieved by checking DRDY interrupt
  // Bit 1-0: Measurement configuration
  //          b00=Normal (default); b01=Positive bias for cal (use 0x11); b10=Negative bias for cal (use 0x12)
  Wire.send(0x00);
  Wire.send(0x11);   // positive bias, read twice

  
   // Configuration Register B (Register address 0x01)
 // Set Gain
  Wire.send(0x60); //set to +/- 2.5 Ga (660 LSB/Gauss)
  
  // Set Measurement Mode
  // Register address 0x02
  // 0x00 : Continuous measurement mode (~100 uA draw)
  // 0x01 : Single Measurement mode
  // 0x02 : Idle Mode (2 uA draw)

  Wire.send(0x02);
  Wire.send(0x01);   // Single measurement mode
  Wire.endTransmission(); //end transmission
  Read_Compass();
  magnetom_cal[0]=(unsigned int)magnetom_x;  
  magnetom_cal[1]=(unsigned int)magnetom_x>>8;  
  magnetom_cal[2]=(unsigned int)magnetom_y;
  magnetom_cal[3]=(unsigned int)magnetom_y>>8;
  magnetom_cal[4]=(unsigned int)magnetom_z;
  magnetom_cal[5]=(unsigned int)magnetom_z>>8;
  Compass_Init(); //return to original settings
}
*/

void Gyro_Init()
{

  int ecode;
 //  Serial.print("Gyro Init\n");
 Wire.beginTransmission(GyroAddress);
   
  Wire.send(0x3E); //power management
  Wire.send(0x01); //everything awake; clock from X gyro reference
 //  ecode=Wire.endTransmission(); //end transmission
  delay(5);

 //  Serial.print(ecode);
 //  Serial.print("\n");

// 
// bits 3-4: 
// 0 +/- 250 degrees/s
// 1 +/- 500 degrees/s
// 2 +/- 1000 degrees/s
// 3 +/- 2000 degrees/s

// Bits 0-2. Low pass filter setting
// 0 256 Hz
// 1 188 Hz
// 2 98 Hz
// 3 42 Hz
// 4 20 Hz
// 5 10 Hz
// 6 5 Hz
  Wire.beginTransmission(GyroAddress);  
  Wire.send(0x16);  // gyro scale, sample rate and LPF
  Wire.send(0x0B);   // 500 degrees/sec; 1kHz; 20 Hz LP
  ecode=Wire.endTransmission(); //end transmission
  delay(5);
 //  Serial.print("Gyro Setup; 0=good\n");
 //  Serial.print(ecode);
 //  Serial.print("\n");
 
 // Wire.beginTransmission(GyroAddress);  
//  Wire.send(0x15);  //sample rate divider
 // Wire.send(0x07);   // divide down to 125 Hz
  //Wire.endTransmission(); //end transmission
 // delay(5);
 
  Wire.beginTransmission(GyroAddress); 
  Wire.send(0x3D);        //sends address to write
  Wire.send(0x01);  //reset gyro
  ecode=Wire.endTransmission(); //end transmission
 /*
 
   Wire.beginTransmission(GyroAddress); 
  Wire.send(0x0C);        //sends address to read from
  ecode=Wire.endTransmission(); //end transmission
  
  Wire.requestFrom(GyroAddress, 2);    // request 2 bytes from device
  int numbytesavailable=Wire.available();

  byte ga1 = Wire.receive();  // receive one byte
  byte ga2 = Wire.receive();
 
  Serial.print("Gyro xoffset\n");
    Serial.print(((int)ga1<<8)|ga2);
    Serial.print("\n");


   Wire.beginTransmission(GyroAddress); 
  Wire.send(0x0C);        //set offset to 3
  Wire.send(0x03);
  ecode=Wire.endTransmission(); //end transmission
  
     Wire.beginTransmission(GyroAddress); 
  Wire.send(0x0C);        //sends address to read from
  ecode=Wire.endTransmission(); //end transmission
  
  Wire.requestFrom(GyroAddress, 2);   

ga1 = Wire.receive();  // receive one byte
ga2 = Wire.receive();
 
  Serial.print("Gyro xoffset read back after setting to three\n");
    Serial.print(((int)ga1<<8)|ga2);
    Serial.print("\n");
*/

}

void Read_Gyro()
{
  int i = 0;
  byte buff[6];
  int ecode;
  int numbytestoread=6;
 
  Wire.beginTransmission(GyroAddress); 
  Wire.send(0x1D);        //sends address to read from
  ecode=Wire.endTransmission(); //end transmission
  
  //Wire.beginTransmission(GyroAddress); 
  Wire.requestFrom(GyroAddress, numbytestoread);    // request 6 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  //Wire.endTransmission(); //end transmission
  
  if (i==numbytestoread)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
   // gyro_temp = ((((int)buff[0]) << 8) | buff[1]); 
    gyro_x = ((((int)buff[0]) << 8) | buff[1]);    // X axis (roll to right)
    gyro_y = ((((int)buff[2]) << 8) | buff[3]);    // Y axis (pitch forward)
    gyro_z = -1*((((int)buff[4]) << 8) | buff[5]);    // Z axis
    
    }
 // else
 //   Serial.println("ERR:Gyro");
}



void setTime2(int hour,int minute,int second,int day,int month,int year)
{
/*  second=(10*(buff[0]>>4)) + ((buff[0]&0x0F)) ;//lower 4 bits is seconds, upper 3 bits are 10 seconds
  minute=(10*(buff[1]>>4)) + ((buff[1]&0x0F)) ;//lower 4 bits is minutes, upper 3 bits are 10 minutes
  hour=(10*((buff[2]>>4)&0x03))+((buff[2]&0x0F));  //bits 4 and 5 are 10 hours; bit 0-3 are hour
  date=(10*(buff[4]>>4))+((buff[4]&0x0F)); 
  month=(10*(buff[5]>>4))+((buff[5]&0x0F)); 
  year=(10*(buff[6]>>4))+((buff[6]&0x0F));   
  */
  Wire.beginTransmission(RTCAddress);
  Wire.send(0x00);  //starting address is 00
  Wire.send(((second/10)<<4) | ((second%10)&0x0F));  //seconds, 
  Wire.send(((minute/10)<<4) | ((minute%10)&0x0F));  //min
  Wire.send((((hour/10)&0x03)<<4) | ((hour)%10&0x0F)); //hour  0x20 will set to 12 hour AM/PM mode
  Wire.send(0x01); // day of week(1-7)
  Wire.send(((day/10)<<4) | ((day%10)&0x0F)); // date (1-31)
  Wire.send((month/10)<<4 | ((month%10)&0x0F)); //  month
  Wire.send((year/10)<<4 | ((year%10)&0x0F)); //  year 2010=0x0A
  int ecode=Wire.endTransmission(); //end transmission
//  Serial.println("RTC Initialization: ");
//  Serial.println(ecode);
}

void Read_RTC()
{
  int i = 0;
  byte buff[7];
 
  Wire.beginTransmission(RTCAddress); 
  Wire.send(0x00);        //sends address to read from starting with seconds
  Wire.endTransmission(); //end transmission
  
  Wire.requestFrom(RTCAddress, 7);    // request 7 bytes from clock (seconds through Year)
 //   Serial.println("Bytes available: ");

  int bytesavail=Wire.available();
//   Serial.println(bytesavail); 
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  
  if (i==7)  // All bytes received?
    {
      second=(10*(buff[0]>>4)) + ((buff[0]&0x0F)) ;//lower 4 bits is seconds, upper 3 bits are 10 seconds
      minute=(10*(buff[1]>>4)) + ((buff[1]&0x0F)) ;//lower 4 bits is minutes, upper 3 bits are 10 minutes
      hour=(10*((buff[2]>>4)&0x03))+((buff[2]&0x0F));  //bits 4 and 5 are 10 hours; bit 0-3 are hour
      date=(10*(buff[4]>>4))+((buff[4]&0x0F)); 
      month=(10*(buff[5]>>4))+((buff[5]&0x0F)); 
      year=(10*(buff[6]>>4))+((buff[6]&0x0F));   
      /*
         if(printflag)
      {
     
        Serial.println("HH:MM:SS Date-Month-Year");
        Serial.print((int)hour);
        Serial.print(":");
        Serial.print((int)minute);
        Serial.print(":");
        Serial.print((int)second);
        Serial.print("  ");
        Serial.print((int)date);
        Serial.print("-");
        Serial.print((int)month);
        Serial.print("-");
        Serial.print((int)year);
        Serial.print("\n");
       
      }
       */
    }
  //else
  //  Serial.println("!ERR: RTC data");
 }
 
 
