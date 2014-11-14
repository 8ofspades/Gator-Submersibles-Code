#define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
	short *pCV;
	short n;
	unsigned int lv1;
	char s[22];
        unsigned int tday;
        unsigned int tmonth;
        unsigned int tyear;
        unsigned int thour;
        unsigned int tmin;
        unsigned int tsec;

	pCV = (short*)pCmd;

	n = strlen(pCmd);
	if(n<2) 
          return TRUE;

	switch(*pCV)
	{              
                case ('V' + ('1'<<8)):  
                {
                   sscanf(&pCmd[3],"%d",&lv1);
                   speriod=lv1/1000;  // Master interrupt rate in ms. 
                   break;
                }
  
                //SC is IMU sample period in ms
                case ('S' + ('C'<<8)):
                {
                   sscanf(&pCmd[3],"%d",&lv1);
                   iperiod=lv1;
                   break; 
                  
                }
		// Sets real time clock
		case ('T' + ('M'<<8)):
		{
                   //set time
                   sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tmonth,&tday,&tyear,&thour,&tmin,&tsec);
                   
                   // if earlier than current time...do not reset
                   TIME_HEAD NewTime;
                   second=tmonth;
                   minute=tmin;
                   hour=thour;
                   date=tday;
                   month=tmonth;
                   year=tyear;
                   updatetime(&NewTime);  //load into structure
                   ULONG newtime=RTCToUNIXTime(&NewTime);  //get new time in seconds
                   Read_RTC();  //update time with current clock time
                   updatetime(&dfh.RecStartTime);  //load current clock time into DF_HEAD time structure
                   ULONG curtime=RTCToUNIXTime(&dfh.RecStartTime);
                   
                   if(newtime>curtime)
                     setTime2(thour,tmin,tsec,tday,tmonth,tyear);    
     		  break;	
		}

                // Force set of Real Time Clock
		case ('F' + ('T'<<8)):
		{
                   //set time
                   sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tmonth,&tday,&tyear,&thour,&tmin,&tsec);
                   setTime2(thour,tmin,tsec,tday,tmonth,tyear); 
                   break;
                }
                
                 // Burn Time
		case ('B' + ('W'<<8)):
		{
                   //set time
                   sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&burntime.month,&burntime.mday,&burntime.year,&burntime.hour,&burntime.minute,&burntime.sec);
                   break;
                }               

		// Disable LEDS
		case ('L' + ('D'<<8)):
		{
		    LEDSON=0;
		    break;
		}

		// ADC0
		case ('A' + ('0'<<8)):
		{
		   sscanf(&pCmd[3],"%d",&lv1);
                   HYDRO1period=lv1;
                   HYDRO1flag=1;
		    break;
		}

/*
		// ADC7
		case ('A' + ('7'<<8)):
		{
		    A7flag=1;
		    break;
		}
*/
		// External Accelerometer
		case ('A' + ('E'<<8)):
		{
		   sscanf(&pCmd[3],"%d",&lv1);
                   HYDRO1period=lv1;  //uses same sample clock as ADC0
                   
                    accelflagext=1;
		    break;
		}
		// Board Accelerometer
		case ('A' + ('I'<<8)):
		{
		    accelflagint=1;
		    break;
		}
		// Magnetometer
		case ('M' + ('G'<<8)):
		{
		    compflag=1;
		    break;
		}
		// Gyroscope
		case ('G' + ('Y'<<8)):
		{
		    gyroflag=1;
		    break;
		}
		// PT Period
		case ('P' + ('T'<<8)):
		{
		     sscanf(&pCmd[3],"%d",&lv1);
                     PTperiod=lv1;  // Pressure/temperature period in ms
		    break;
		}
		// Temperature
		case ('T' + ('P'<<8)):
		{
		   tempflag=1;
		    break;
		}
		// Pressure
		case ('P' + ('R'<<8)):
		{
		   pressflag=1;
		    break;
		}
		// Salt Switch
		case ('S' + ('W'<<8)):
		{
		    saltflag=1;
		    break;
		}
		// Serial Print 
        	case ('S' + ('P'<<8)):
		{
		    printflag=1;
		    break;
		}
	}	
	return TRUE;
}

void LoadScript(SdFile file)
{
	char s[22];
	char c;
	short i;

     int16_t n;

 // Read card setup.txt file to set date and time, sample rate, recording interval
 card.chdir("Script");
  if(file.open("Default.txt", O_READ))
  {
    do
	{
		i = 0;
		s[i] = 0;
		do
		{
		    n=file.read(&c, 1);
		    if(c!='\r')
		      s[i++] = c;
		    if(i>20)
		      break;
		}while(c!='\n');
		s[--i] = 0;
			
		if(s[0] != '/' && i>1)
		{
		  ProcCmd(s);
		}
	}while(n);

	file.close();
      
  }
 else
 {
    // Serial.println("No default.txt");
    //enable defaults if no script present
    accelflagint=1;  //flag to enable accelerometer; 
    compflag=1; //flag to enable 3d magnetometer (compass)
    gyroflag=1; //flag to enable gyro
    pressflag=1; //flag to enable pressure
    tempflag=1; //flag to enable temperature
 }

 card.chdir();  //change to root
	
  return;	


}


