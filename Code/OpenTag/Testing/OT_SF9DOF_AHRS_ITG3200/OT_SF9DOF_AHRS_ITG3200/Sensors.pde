/* ******************************************************* */
/* Code for ADXL345 accelerometer, ITG3200 gyro            */
/* and HMC5883 magnetometer                                */
/* ******************************************************* */

int AccelAddress = 0x53;
int CompassAddress = 0x1E;

// ITG3200 definitions
#define ITG3200_ADDRESS 0x69  // I2C address 0x68 or 0x69 (depends on AD0) Sparkfun Razor 9DOF & ITG3200=>0x68
// ITG3200 Register map (from datasheet)
#define WHO_AM_I        0x00
#define	SMPLRT_DIV	0x15
#define DLPF_FS         0x16
#define INT_CFG         0x17
#define INT_STATUS	0x1A
#define	GYROX_H	        0x1D
#define PWR_MGM	        0x3E


void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
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
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(AccelAddress);
  Wire.send(0x2C);  // Rate
  Wire.send(0x09);  // set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(AccelAddress); 
  Wire.send(0x32);        //sends address to read from
  Wire.endTransmission(); //end transmission

    Wire.beginTransmission(AccelAddress); //start transmission to device
  Wire.requestFrom(AccelAddress, 6);    // request 6 bytes from device

  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission

    if (i==6)  // All bytes received?
  {
    ACC[1] = (((int)buff[1]) << 8) | buff[0];    // X axis (internal sensor x axis)
    ACC[0] = (((int)buff[3]) << 8) | buff[2];    // Y axis (internal sensor y axis)
    ACC[2] = (((int)buff[5]) << 8) | buff[4];    // Z axis
    AN[3] = ACC[0];
    AN[4] = ACC[1];
    AN[5] = ACC[2];
    accel_x = SENSOR_SIGN[3]*(ACC[0]-AN_OFFSET[3]);
    accel_y = SENSOR_SIGN[4]*(ACC[1]-AN_OFFSET[4]);
    accel_z = SENSOR_SIGN[5]*(ACC[2]-AN_OFFSET[5]);
  }
  else
    Serial.println("!ERR: Acc data");
}

void Compass_Init()
{
  Wire.beginTransmission(CompassAddress);
  Wire.send(0x02); 
  Wire.send(0x00);   // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission
}

void Read_Compass()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission

    //Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission

    if (i==6)  // All bytes received?
  {
    // MSB byte first, then LSB, Reads X, Z, Y
    magnetom_x = SENSOR_SIGN[6]*((((int)buff[4]) << 8) | buff[5]);    // X axis   Internal Y
    magnetom_y = SENSOR_SIGN[7]*((((int)buff[0]) << 8) | buff[1]);    // Y axis   Internal x
    magnetom_z = SENSOR_SIGN[8]*((((int)buff[2]) << 8) | buff[3]);    // Z axis   internal z
  
 
  }
  else
    Serial.println("!ERR: Mag data");
}

int ITG3200_test(void)
{
  int temp;
  
  Wire.beginTransmission(ITG3200_ADDRESS); 
  Wire.send(WHO_AM_I);        //sends address to read from
  Wire.endTransmission(); 
  
  Wire.requestFrom(ITG3200_ADDRESS, 1);    // request 1 bytes from device
  while(Wire.available())     
    temp = Wire.receive();  // receive one byte
    
  Wire.endTransmission(); //end transmission
  
  return(temp);
}

// ITG3200 Gyro initialization and configuration
void ITG3200_Init(void)
{
  Wire.begin();  
  
  Wire.beginTransmission(ITG3200_ADDRESS);
  Wire.send(PWR_MGM);       
  Wire.send(0x80);        // Chip reset
  Wire.endTransmission();
  delay(1);
  /* ********************************************* */
  Wire.beginTransmission(ITG3200_ADDRESS);
  Wire.send(SMPLRT_DIV);
  Wire.send(0x04);        // sample rate divider 4=200Hz   9=100Hz
  Wire.endTransmission();
  Wire.beginTransmission(ITG3200_ADDRESS);
  Wire.send(DLPF_FS);       
  Wire.send(0x1B);        //0x1A (2000º/s)+DLPF_CFG=2(98Hz)  //0x1B (2000º/s)+DLPF_CFG=3(42Hz)
  Wire.endTransmission();  
  delay(1);
  //Wire.beginTransmission(ITG3200_ADDRESS);
  //Wire.send(INT_CFG);
  //Wire.send(0x11);        // Interrupt on raw data ready
  //Wire.endTransmission();
  //delay(1);
  Wire.beginTransmission(ITG3200_ADDRESS);
  Wire.send(PWR_MGM);       
  Wire.send(0x01);        // Oscillator : PLL with X gyro reference (recommended in datasheet)
  Wire.endTransmission(); //end transmission
 
  
}

// Read Sensor data
void ITG3200_Read()
{
  byte i = 0;
  byte buff[6];
 
  Wire.beginTransmission(ITG3200_ADDRESS); 
  Wire.send(GYROX_H);        //sends address to read from
  Wire.endTransmission(); 
  
  //Wire.beginTransmission(ITG3200_ADDRESS); 
  Wire.requestFrom(ITG3200_ADDRESS, 6);    // request 6 bytes from device
  while(Wire.available())     
    buff[i++] = Wire.receive();  // receive one byte
    
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
    GYRO[0] = ((((int)buff[0]) << 8) | buff[1]);    // X axis
    GYRO[1] = ((((int)buff[2]) << 8) | buff[3]);    // Y axis
    GYRO[2] = ((((int)buff[4]) << 8) | buff[5]);    // Z axis
    AN[0] = GYRO[0];
    AN[1] = GYRO[1];
    AN[2] = GYRO[2];
    
    /*
    // For m11, before rotated gyro relative to accelerometer
    gyro_x = SENSOR_SIGN[0]*(GYRO[0]-AN_OFFSET[0]);
    gyro_y = SENSOR_SIGN[1]*(GYRO[1]-AN_OFFSET[1]);
    gyro_z = SENSOR_SIGN[2]*(GYRO[2]-AN_OFFSET[2]);
    */
    gyro_y = SENSOR_SIGN[0]*(GYRO[0]-AN_OFFSET[0]);
    gyro_x = SENSOR_SIGN[1]*(GYRO[1]-AN_OFFSET[1]);
    gyro_z = SENSOR_SIGN[2]*(GYRO[2]-AN_OFFSET[2]);
    
    }
}


