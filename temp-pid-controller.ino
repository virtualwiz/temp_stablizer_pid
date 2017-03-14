#include <OneWire.h>
#include "LCD12864RSPI.h"
#define AR_SIZE( a ) sizeof( a ) * sizeof( a[0] )

#define GPIO_KEY 2
#define GPIO_LED 13
#define GPIO_BUZZER 12
#define GPIO_RELAY A0
#define GPIO_DS A2
#define PID_ILIMIT_P 20
#define PID_ILIMIT_N 0

OneWire ds(GPIO_DS);

unsigned char show0[]={0xB4,0xAB,0xB8,0xD0,0xC6,0xF7,0xC0,0xEB,0xCF,0xDF,0xC4,0xBF,0xB1,0xEA,'-','-'};
unsigned char show1[]={0xCB,0xE3,0xB7,0xA8,0xA3,0xBA};
unsigned char show2[]={'P','I','D',' ',0xBD,0xE1,0xB9,0xFB,0xA3,0xBA};
unsigned char show3[]={0xCF,0xDF,0xC8,0xA6,0xD7,0xB4,0xCC,0xAC,'C',':',' ',' ','P',':'};

volatile int target_temp = 35;
boolean mode = 1;

int PID_Exec(float PID_In, int PID_Target , float kP,float kI)
{
	float POut = 0;
	static float IVal = 0;
	int PID_Out;
	
	POut = (PID_Target - PID_In) * kP;
	IVal += (PID_Target - PID_In) * kI;
	
	if(IVal >= PID_ILIMIT_P)
		IVal = PID_ILIMIT_P;
	if(IVal <= -PID_ILIMIT_N) 
		IVal = -PID_ILIMIT_N;
		
	PID_Out = (int)(POut + IVal);
	return PID_Out;
}

void Buzzer_PlayStartupSound()
{
  tone(GPIO_BUZZER,2093,150);
  delay(150);
  tone(GPIO_BUZZER,2349,150);
  delay(150);
  tone(GPIO_BUZZER,2637,200);
  delay(200);
  noTone(GPIO_BUZZER);
}

void Hardware_Init()
{
  pinMode(GPIO_BUZZER,OUTPUT);
  pinMode(GPIO_KEY,INPUT_PULLUP);
  pinMode(GPIO_LED,OUTPUT);
  pinMode(GPIO_RELAY,OUTPUT);
  LCDA.Initialise(); //@Initialize
  Buzzer_PlayStartupSound();
  Serial.begin(2400);
}

void LCD_RefreshData(int sensorx10,int target,boolean arith,int pidresultx10,boolean coilc,boolean coilp)
{
  unsigned char sensor_str[4];
  unsigned char target_str[2];
  unsigned char arith_1_str[]={0xCA,0xFD,0xD6,0xB5,0xB1,0xC8,0xBD,0xCF};
  unsigned char arith_2_str[]={0xB1,0xC8,0xC0,0xFD,0xBB,0xFD,0xB7,0xD6};
  unsigned char pidresult_str[6];
  unsigned char close_str[]={0xBA,0xCF};
  unsigned char open_str[]={0xB7,0xD6};
  
  sensor_str[0]=sensorx10/100+0x30;
  sensor_str[1]=sensorx10%100/10+0x30;
  sensor_str[2]='.';
  sensor_str[3]=sensorx10%10+0x30;
  LCDA.DisplayString(0,3,sensor_str,4);

  target_str[0]=target/10+0x30;
  target_str[1]=target%10+0x30;
  LCDA.DisplayString(0,7,target_str,2);
  
  if(arith)
	LCDA.DisplayString(1,3,arith_2_str,8);
  else 
	LCDA.DisplayString(1,3,arith_1_str,8);
	
  pidresult_str[0]=pidresultx10>=0?'+':'-';
  if(pidresultx10<0) pidresultx10=-pidresultx10;
  pidresult_str[1]=pidresultx10/1000+0x30;
  pidresult_str[2]=pidresultx10%1000/100+0x30;
  pidresult_str[3]=pidresultx10%100/10+0x30;
  pidresult_str[4]='.';
  pidresult_str[5]=pidresultx10%10+0x30;
  LCDA.DisplayString(2,5,pidresult_str,6);
  
  if(coilc)
	LCDA.DisplayString(3,5,close_str,2);
  else
	LCDA.DisplayString(3,5,open_str,2);
  if(coilp)
	LCDA.DisplayString(3,7,close_str,2);
  else
	LCDA.DisplayString(3,7,open_str,2);
}


void setup()
{
  Hardware_Init();
  //attachInterrupt(0,KeyPress,FALLING);
  LCDA.CLEAR();//@Clear Screen
  delay(100);
  LCDA.DisplayString(0,0,show0,AR_SIZE(show0));
  LCDA.DisplayString(1,0,show1,AR_SIZE(show1));
  LCDA.DisplayString(2,0,show2,AR_SIZE(show2));
  LCDA.DisplayString(3,0,show3,AR_SIZE(show3));
  if(!digitalRead(GPIO_KEY))
  mode = !mode;
}
 
byte ScanPeriodCounter = 0;
boolean AlarmEnabled = 1;

void loop()
{
  float output_buffer = 0;
  int output_value = 0;
  boolean relay_status_c = 0;
  boolean relay_status_p = 0;
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
   
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(10);
    return;
  }
 
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  
  digitalWrite(GPIO_LED,HIGH);
  
  delay(10);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
   
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  digitalWrite(GPIO_LED,LOW);
  // convert the data to actual temperature
 
  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  Serial.println(celsius);
  
  output_buffer = PID_Exec(celsius,target_temp,7,0.05);
  
  output_value=(int)output_buffer;
  if(output_value>=100)output_value = 100;
  if(output_value<=0)output_value=0;
  
  if(celsius<target_temp)
  relay_status_c = 1;
  else relay_status_c = 0;  
  
  if(ScanPeriodCounter++ <= output_buffer)
  relay_status_p = 1;
  else relay_status_p = 0;
  if(ScanPeriodCounter >= 100)
  ScanPeriodCounter = 0;
	
  if(!mode)
  digitalWrite(GPIO_RELAY,relay_status_c);
  else
  digitalWrite(GPIO_RELAY,relay_status_p);

  if(!digitalRead(GPIO_KEY))
  {
    tone(GPIO_BUZZER,2093,20);
    for(unsigned int cyc = 0;cyc<60000;cyc++)
	{
	  if(digitalRead(GPIO_KEY))
	  break;
	}
	target_temp--;
	AlarmEnabled = 1;
	while(!digitalRead(GPIO_KEY))
	{
	   tone(GPIO_BUZZER,2093,20);
	   delay(30);
	   tone(GPIO_BUZZER,1568,20);
	   delay(30);
	   target_temp++;
	   AlarmEnabled = 1;
	   LCD_RefreshData((int)(celsius*10),target_temp,mode,(int)((output_buffer)*10),relay_status_c,relay_status_p);
	}
  }
    
  LCD_RefreshData((int)(celsius*10),target_temp,mode,(int)((output_buffer)*10),relay_status_c,relay_status_p);
  if(AlarmEnabled)
  {
  if(target_temp == (int)celsius)
	{
	   tone(GPIO_BUZZER,1568,500);
	   delay(500);
	   AlarmEnabled = 0;
	} 
  }
}
