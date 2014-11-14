
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Measurement Attitude and Heading Reference System
// Firmware v1.2
// This code is for new Sparkfun board with ITG3200 gyros and 5883 magnetometer
//
// Modified for OpenTag M11
// No Offsets at beginning


// Released under Creative Commons License 
// Code by Doug Weibel and Jose Julio
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel

// Axis definition: 
   // X axis pointing forward (to the FTDI connector)
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

/* 
    Hardware version - 
	
	ATMega328@3.3V w/ external 8MHz resonator
	High Fuse DA
        Low Fuse FF
	
	ADXL345: Accelerometer
	HMC5883: Magnetometer
	ITG3200 : Gyro

        Programmer : 3.3v FTDI
        Arduino IDE : Select board  "Arduino Pro or Pro Mini (3.3V, 8MHz) w/ ATmega 328"
*/


#include <Wire.h>

// ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
// Tested value : 248
#define GRAVITY 248  //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// ITG3200 sensibility  (theorical 0.069 => 1/14.375LSB/º/s )
#define Gyro_Gain_X 0.069
#define Gyro_Gain_Y 0.069
#define Gyro_Gain_Z 0.069
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.01
#define Ki_ROLLPITCH 0.00001
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define ADC_WARM_CYCLES 50
#define STATUS_LED 13 

// int SENSOR_SIGN[9] ={1,-1,-1,1,1,1,-1,-1,1};  //{1,-1,-1,1,1,1,-1,-1,-1};  //original m11
int SENSOR_SIGN[9] ={-1,-1,-1,1,1,1,1,1,1};  

//Correct directions x,y,z - gyros, accels, magnetometer

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that store the 3 ADC filtered data (gyros)
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int ACC[3];          //array that store the raw accelerometers data
int GYRO[3];

int accel_x;
int accel_y;
int accel_z;
int gyro_x;
int gyro_y;
int gyro_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
 

void setup()
{ 
  float aux_float[6];
  
  Serial.begin(57600);
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  
  I2C_Init();
  ITG3200_Init();
  Accel_Init();
  Read_Accel();

  Serial.println();
  Serial.println("OpenTag");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);
 
  // Magnetometer initialization
  Compass_Init();
  
  // Initialze ADC readings and buffers
  delay(20);
 
  for(int i=0;i<100;i++)    // We take some readings...
    {
    ITG3200_Read();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
       //aux_float[y] += AN[y];
       aux_float[y] += 0;
    delay(2);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = aux_float[y]/100;
    
 // AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  AN_OFFSET[5]=0;
  Serial.println("Offsets:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);
  
  delay(200);
  digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);

  counter=0;
}

void loop() //Main Loop
{
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    ITG3200_Read();     // This read gyro data (ITG3200)
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
   
    printdata();
    
    //Turn off the LED when you saturate any of the gyros.
    if((abs(Gyro_Vector[0])>=ToRad(300))||(abs(Gyro_Vector[1])>=ToRad(300))||(abs(Gyro_Vector[2])>=ToRad(300)))
      {
      if (gyro_sat<50)
        gyro_sat+=10;
      }
    else
      {
      if (gyro_sat>0)
        gyro_sat--;
      }
  
    if (gyro_sat>0)
      digitalWrite(STATUS_LED,LOW);  
    else
      digitalWrite(STATUS_LED,HIGH);  
  
  }
   
}
