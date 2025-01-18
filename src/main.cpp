/*
AUTHOR: ENG. RIYAD AL-ALI 27/10/2024 
USE EXTERNAL CRYSTAL 16.0 MHZ 



*/
//-------------------------------------MEMORY MAP----------------------------------------------------
/*
hours_lcd_1 : 0 
minutes_lcd_1 : 1
hours_lcd_2: 2 
minutes_lcd_2 : 3
//-----------------------
hours_lcd_timer_2_start: 4 
minutes_lcd_timer_start: 5
hours_lcd_timer_2_stop: 6
minutes_lcd_timer_2_stop:7
Mini_battery_voltage: 8
mini_battery_voltage_t2: 12
startup_voltage: 16
startloadvoltage_t2: 20
startuptimer_1 : 24
startuptimer_2: 26
//----------------------------
Runonbatteryvoltagemode: 28
upsmode: 29 
addError=30
VinBatteryDifference=31~34 
batteryTypeLiPo4=50
*/
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Wire.h>
#include <EEPROM.h>


//----------------------------LCD--------------------
//const int rs = 8, en =9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
const int rs = A3, en =A2, d4 = 13, d5 = 12, d6 = 11, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//---------------------------Defines------------------------------
RTC_DS3231 rtc;
char t[32];
 //--------------------------Defines-------------------------------------
 #define Relay_L_Solar 6
 #define Relay_L_Solar_2 7
 #define Set 2
 #define Decrement 8  
 #define Increment 5
 #define AC_Available 3
 #define Backlight 9
 #define Exit A0
 #define Flash 4
 #define LONG_PRESS_DURATION 2000
//-----------------------------------------Variables----------------------------
//unsigned short old_time_compare_pv,old_time_update_pv,old_time_screen_1=0,old_time_screen_2=0; // to async
char set_status=0;    //variable for the set button state
char txt[32];
char seconds_lcd_1=0,minutes_lcd_1=0,hours_lcd_1=0;
char seconds_lcd_2=0,minutes_lcd_2=0,hours_lcd_2=0;
char hours_lcd_timer2_start=0,hours_lcd_timer2_stop=0,seconds_lcd_timer2_start=0;
char minutes_lcd_timer2_start=0,minutes_lcd_timer2_stop=0,seconds_lcd_timer2_stop=0;
char Relay_State; // variable for toggling relay
char set_ds1307_minutes=0,set_ds1307_hours=0,set_ds1307_seconds=0,set_ds1307_day=1,set_ds1307_month=1;
uint16_t set_ds1307_year=2024;
char ByPassState=0;    //enabled is default 0 is enabled and 1 is disabled
float Battery_Voltage,PV_Voltage,Vin_PV,Vin_PV_Old=0,Vin_PV_Present=0;
char BatteryVoltageSystem=0; // to save the battery voltage system if it is 12v/24v/48v
unsigned int ADC_Value;   // adc value for battery voltage
unsigned int ADC_Value_PV;
double Vin_Battery;      //voltage of battery
float Mini_Battery_Voltage=0,Mini_Battery_Voltage_T2=0;     // for timer 1 and timer 2
char Timer_Enable=1;   // timer 1
char Timer_2_Enable=1; // timer 2
char Timer_3_Enable=1; //timer 3
char CorrectionTime_State=0;  // this function to solve the error when battery is low and timer didn't start because of the low battery
unsigned int High_Voltage=245;      //ac high voltag`e
unsigned int Low_Voltage=175;       // ad low voltage
char VoltageProtectorGood;
char BatteryGuardEnable=1;   // enabled is default
char VoltageProtectionEnable; // enable voltage protection on grid
char Error_Voltage=0;       //difference between voltage and reading voltage
float v; // ac voltage as global variable
char Saved_Voltage;     // volatge when user hits set
char Adjusted_Voltage; // voltage saved by user
char AcBuzzerActiveTimes=0;  //for not making buzzer always on
char AcBuzzerActive=0;  //  for controlling buzzer activation just for one time
char matched_timer_1_start,matched_timer_1_stop, matched_timer_2_start,matched_timer_2_stop;
char Old_Reg=0;
char SolarOnGridOff=0,SolarOffGridOn=0;
char SolarOnGridOff_2=0,SolarOffGridOn_2=0;
char Timer_isOn=0,Timer_2_isOn=0;
unsigned int Timer_Counter_2=0, Timer_Counter_3=0,Timer_Counter_4=0;
unsigned int Low_PV_Voltage=50;        // PV panels low voltage
bool Grid_Already_On=false;            // to not enter conditions as the grid is available
unsigned short old_timer_1=0,old_timer_2=0,temp=0;
int startupTIme_1=0,startupTIme_2=0;  // 25 seconds for load one to start up and 50 seconds for load 2 to startup
int offDelay_1=0,offDelay_2=0;
char updateScreen=0;
float arrayBatt[21];
float StartLoadsVoltage=0,StartLoadsVoltage_T2=0;
float BuzzerVoltage=0.1; // voltage added to mini voltage to start giving the alarm before loads switches off
unsigned short ReadMinutesMinusOldTimer_1=0;
unsigned short ReadMinutesMinusOldTimer_2=0;
unsigned int Timer_Counter_For_Grid_Turn_Off=0;
char RunTimersNowState=0;
unsigned int SecondsRealTime=0,CutSecondsRealTime_T1=0 , CutSecondsRealTime_T2=0 ;   // for holding reading seconds in real time for ac grid and startup timers
unsigned int SecondsRealTimePv_ReConnect_T1=0,SecondsRealTimePv_ReConnect_T2=0; // for reactive timers in sequence when timer switch off because off battery and wants to reload
unsigned int realTimeLoop=0;
bool RunWithOutBattery=false;
char const ButtonDelay=200;
char RunLoadsByBass=0;
char TurnOffLoadsByPass=0; // to turn off for error
char VoltageProtectorEnableFlag=1;
char every30MinutesInitScreen=0;
char initedScreenOnce=0;
unsigned int UpdateScreenTime=0,TimeToExitSetupProgram=0;
char SystemBatteryMode=0;
char EnterSetupProgram=0; // variable to detect if mcu in loop of setup program
unsigned int  ReadBatteryTime=0;
char RunOnBatteryVoltageMode=0;
char UPSMode=0;       // i made ups mode and upo mode in same variable
char LoadsAlreadySwitchedOFF=0;
uint16_t Full_Seconds;
unsigned long currentTime = 0;
unsigned int CountSecondsRealTime=0;   // for secondsrealtime
unsigned int CountSecondsRealTimePv_ReConnect_T2=0,CountSecondsRealTimePv_ReConnect_T1=0;
unsigned int CountCutSecondsRealTime_T1=0,CountCutSecondsRealTime_T2=0; // time for cutting loads off

uint32_t currentMillis_1=0,previousMiliis_1=0; // for flashing display 
uint32_t currentMillis_2=0,previousMiliis_2=0; // for flashing display 

//-> for battery sampling 
uint32_t samplesReading=0;
float sum=0 , Battery[500];
char i=0; 
char programNumber=0; // for setup program 
//-> FOR RELAY STATES 
char relayState[32];
char relayState_1=0,relayState_2=0,relayState_3=0; // for saving relay state
double VinBatteryDifference=0.0;
char addError=0,MinusError=0;
float Vin_Battery_=0.0;
double VinBatteryError=0;

//-variables for the LiPo4
    String receivedData = "";
    bool endOfResponse = false;  // Flag to indicate when end of response is found
    String batteryVoltage="";
    String batteryCapacity="";
    char batteryTypeLiPo4=1; 
    unsigned long startTime_inverter ; // Record the time when we started waiting
    const unsigned long timeout_inverter = 1000;  // Set timeout to 2 seconds (2000 milliseconds)
    String formattedBatteryVoltage="",formattedBatteryCapacity="";
    char bmsErrorFlag=0;
    String loadKW;
    String batteryDischargeCurrent="";


//- variable for long press 
bool insideSetup=false;
char exitProgrampress=500;
    
//-----------------------------------------------------------------------------
struct pipCommands_t{
  unsigned char qpigs[5];
} pipCommands = {'Q', 'P', 'I', 'G', 'S'};


//-----------------------------------Functions---------------------------------
void EEPROM_Load();
void Gpio_Init();
void Write_Time();
void Config();
void Config_Interrupts();
void LCD_Clear(unsigned short Row, unsigned short Start, unsigned short End);
void SetUpProgram();
void Timer_Delay_Config();
void SetTimerOn_1();
void SetTimerOff_1();
void SetTimerOn_2();
void SetTimerOff_2();
void SetDS1307_Time();
void SetDS1307Minutes_Program();
void SetDS1307Seconds_Program();
void TimerDelay();
void Read_Battery();
void SetLowBatteryVoltage();
void StoreBytesIntoEEprom();
void ReadBytesFromEEprom();
void SetTimer();   // set timer to be activated or not activated
void LowBatteryVoltageAlarm();
unsigned int ReadAC();  //read ac voltage
void CalculateAC();   //calculate ac voltage
void DisplayTimerActivation(); // to display if timer is enabled or disabled on LCD
void SetHighVoltage();
void SetLowVoltage();
void VoltageProtector(unsigned long voltage);
void EnableBatteryGuard();         // this function is for enabling or disable the battery protection
void EnableVoltageGuard();   // AC voltage Protection
void SetACVoltageError() ; // this function to adjust differents in the reading volt because of the error in the resistors
void GetVoltageNow(); // get AC voltage at time
void ToggleBuzzer();
void Start_Timer();
void Stop_Timer();
void ReadPV_Voltage();
void SetLowPV_Voltage();
void RestoreFactorySettings();
void EEPROM_FactorySettings(char period);
void Start_Timer_2_B();   // timer for updating screen
void Start_Timer_0_A();  // timer for pv voltage to shutdown
void Stop_Timer_0();
void Read_PV_Continues();  // to eep updating pv
void Startup_Timers();
void SetStartUpLoadsVoltage();
void RunTimersNow();
void TurnACLoadsByPassOn();
void RunTimersNowCheck();
void Watch_Dog_Timer_Enable();
void Watch_Dog_Timer_Disable();
void Write_Date(); // to set date of ds1307
void CheckForParams();
void LCD_ReConfig();
void Interrupt_INT0();
void Interrupt_INT1();
void Read_Time();
unsigned short ReadMinutes();
unsigned short ReadHours();
void AutoRunWithOutBatteryProtection();
void Timer_Seconds(); // timer for counting seconds 
void CheckWireTimeout();
void WDT_Disable();
int analogNoiseReducedRead(int pinNumber); 
void TimerBattery();
void SetVoltageMode(); 
void SetUPSMode();
void SetDS1307_Date();
void DelayOff();
void SetBatteryType();
bool isLongPress();
//-------------------------LIPO4 Functions-----------------------------------------------
void pipSend(unsigned char *cmd, int len);
uint16_t crc16(const uint8_t* data, uint8_t length);
String getValue(String data, char separator, int index);
bool matchesFormat(const String& value, const String& expected);
bool isValidResponse(const String& data);
void Read_LiPo4();
//---------------------------------------------------------------------------------------
void Gpio_Init()
{
pinMode(Relay_L_Solar,OUTPUT);
pinMode(Relay_L_Solar_2,OUTPUT);  // Relay_L_Solar_2 set as output
pinMode(Set,INPUT);  //Set as input  
pinMode(Increment,INPUT);  //Decrement as input   
pinMode(Decrement,INPUT);  //Increment as input  
pinMode(AC_Available,INPUT);  //ac_available as input  
pinMode(Backlight,OUTPUT);  // backlight
pinMode(Exit,INPUT);  // set Exit as input 
pinMode(Flash,OUTPUT);
}

//-------------------------------------Config---------------------------------------------------
void Config()
{
Gpio_Init();
analogReference(EXTERNAL);
TimerBattery();  // for starting reading 
digitalWrite(Backlight,1);
lcd.begin(16,2);
lcd.clear();
lcd.noCursor();
lcd.setCursor(0,0);
lcd.print(" SLC LiPo4 V1.2 ");
delay(1500);
lcd.clear();
Wire.begin();
rtc.begin();
Wire.setWireTimeout(3000,true);   //refe : https://www.fpaynter.com/2020/07/i2c-hangup-bug-cured-miracle-of-miracles-film-at-11/
Wire.clearWireTimeoutFlag();
Serial.begin(2400);
}
//----------------------------------Config Interrupts-----------------------------------
void Config_Interrupts()
{
attachInterrupt(digitalPinToInterrupt(2), Interrupt_INT0, RISING);   //Enter is pressed 
attachInterrupt(digitalPinToInterrupt(3), Interrupt_INT1, RISING);   // ac_avaiable
}
//---------------------------------Enter or Set Interrupt to turn backlight on---------------------------------
void Interrupt_INT0()
{
UpdateScreenTime=0; // if user pressed the button zero counter of dipslay backlight
digitalWrite(Backlight,1);
if(programNumber==0)
{
// lcd.begin(16,2);
// lcd.clear();
// lcd.noCursor();
// lcd.setCursor(0,0);
// lcd.print("Loading");
}

EIFR |= (1 << INTF0);  // Clear INTF0 by writing a 1 to it
delay(100); // give some time to inverter to respond very important 

} 
//-------------------------------When Grid is Turned Off---------------------------------------------------------
void Interrupt_INT1()
{
//delay(1000) ; 
 //-> functions for shutting down loads if there is no timers and grid is off
if(digitalRead(AC_Available)==1 && Timer_isOn==0  && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)
{

SecondsRealTime=0;
CountSecondsRealTime=0;
digitalWrite(Relay_L_Solar,0);
relayState_1=0;

}

if (digitalRead(AC_Available)==1 && Timer_2_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
{
CountSecondsRealTime=0;
SecondsRealTime=0;
digitalWrite(Relay_L_Solar_2,0);
relayState_2=0;

}

if (digitalRead(AC_Available)==1 &&  RunLoadsByBass==0 && UPSMode==1 && LoadsAlreadySwitchedOFF==1)
{
LoadsAlreadySwitchedOFF=0;
SecondsRealTime=0;
SecondsRealTimePv_ReConnect_T1=0;
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTime=0;
CountSecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar,0);
digitalWrite(Relay_L_Solar_2,0);
relayState_1=0;
relayState_2=0;

}

delay(500);

}
//-----------------------------------------Screen 1-------------------------------------------------
void Screen_1()
{
  //noInterrupts();
if (RunLoadsByBass==0) 
{
  lcd.setCursor(15,0);
  lcd.print("  ");
}   else
{
  lcd.setCursor(15,0);
  lcd.print("B");
}  
if (RunOnBatteryVoltageMode==0)
{
Read_Time();
}
else
{
 
  lcd.setCursor(0,0);
  lcd.print("V-MODE ");
  delay(1000);
  
}

if (batteryTypeLiPo4==0)
{
Read_Battery();
}
else 
{

//-> display on LCD for lithium 
Read_LiPo4(); 

// sprintf(txt,"%sV  %s%%  %dA      ",formattedBatteryVoltage.c_str(),formattedBatteryCapacity.c_str(),batteryDischargeCurrent.toInt());
// lcd.setCursor(0,1);
// lcd.print(txt);
if (bmsErrorFlag==0) 
{
  lcd.setCursor(7,0);
  lcd.print("BMS");
}
 else 
 {
   lcd.setCursor(7,0);
   lcd.print("   "); 
 }
}

//CalculateAC();  //for displaying grid is available 
//-> DISPLAY RELAY STATES 
sprintf((char*)relayState,"R=%01d%01d",relayState_1, relayState_2);
lcd.setCursor(11,0);
lcd.print(relayState);
}
//--------------------------------------Read Time-----------------------------------
void Read_Time()
{
DateTime now = rtc.now();
sprintf(t, "%02d:%02d  ", now.hour(), now.minute());
lcd.setCursor(0,0);
lcd.print(t);

}
//------------------------------------Read Battery------------------------------------
//--------------------------Read Battery Voltage--------------------------------
void Read_Battery()
{

//lcd.setCursor(0,1);
//lcd.print("V=");
dtostrf(Vin_Battery,4,1,txt);
lcd.setCursor(0,1);
lcd.print(txt);
}

//-------------------------------Check For Set Program-----------------------------------------------
void CheckForSet()
{

if (digitalRead(Set)==0 ) 
{
delay(1000);
if (digitalRead(Set)==0)
{
programNumber=1;
SetUpProgram();
}
}
}
//--------------------------------------Setup Program---------------------------------------------------
void SetUpProgram()
{
//Delay_ms(500);
digitalWrite(Backlight,HIGH);
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Setup Program");
programNumber=1;
insideSetup=true;
digitalWrite(Flash,0); // turn off flash
delay(1500);
//---------------------------------Enter Programs ------------------------------
//-> enter setup mode and don't exit it until the user hit set button
while (insideSetup)
{

switch (programNumber)
{
case 1: 
   sprintf(txt,"[1] H:%02d-M:%02d   ",hours_lcd_1,minutes_lcd_1);
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
              SetTimerOn_1();

              //  unsigned long pressTime = millis();
              //   while (digitalRead(Set) == 0) 
              //   {
              //       if (millis() - pressTime > exitProgrampress)  
              //       {
              //           delay(200);  // Debouncing

              //           insideSetup=false;
              //           lcd.setCursor(0,0);
              //           lcd.print("Exiting Program");
              //           delay(1000);
              //           return;   // to exit setup 
              //       }
              //   }
    }
   
   break ; 
case 2: 
   sprintf(txt,"[2] H:%02d-M:%02d   ",hours_lcd_2,minutes_lcd_2);
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetTimerOff_1();

                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   
   }
   break ; 
case 3: 
   sprintf(txt,"[3] H:%02d-M:%02d   ",hours_lcd_timer2_start,minutes_lcd_timer2_start);
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetTimerOn_2();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 

case 4 :
   sprintf(txt,"[4] H:%02d-M:%02d   ",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetTimerOff_2();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 


  case 5: 
   sprintf(txt,"[5]  Batt Type  ");
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetBatteryType();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 

case 6:
   sprintf(txt,"[6] Low Voltage   ");
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetLowBatteryVoltage();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 
case 7:
   sprintf(txt,"[7] High Voltage     ");
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetStartUpLoadsVoltage();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 
case 8:
   sprintf(txt,"[8] On Delay     ");
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
    Startup_Timers();
                //  unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 

  case 9 :
   sprintf(txt,"[9] OFF Delay    ");
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
    DelayOff();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 
case 10 :
  sprintf(txt,"[10] Timer Mode  ");
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetVoltageMode();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 

case 11 :
   sprintf(txt,"[11] UtilityMode  ");
   lcd.setCursor(0,0);
   lcd.print(txt);
   while (digitalRead(Set)==0)
   {
   SetUPSMode();
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 
case 12 :
{
  /*
   DateTime now = rtc.now();
   set_ds1307_hours=now.hour();
   set_ds1307_minutes=now.minute();
   set_ds1307_day=now.day();
   set_ds1307_month=now.month();
   set_ds1307_year=now.year();
   sprintf((char*)txt,"[12] H:%02d-M:%02d  ",set_ds1307_hours,set_ds1307_minutes);
   lcd.setCursor(0,0);
   lcd.print(txt);
   */
   lcd.setCursor(0,0);
   lcd.print("[12]    RTC     ");
   while (digitalRead(Set)==0)
   {
   SetDS1307_Time();    
                // unsigned long pressTime = millis();
                // while (digitalRead(Set) == 0) 
                // {
                //     if (millis() - pressTime > exitProgrampress)  // إذا كان الضغط أكثر من 2 ثانية
                //     {
                //         delay(200);  // Debouncing
                //         insideSetup=false;
                //         return;  // الخروج من SetUpProgram بالكامل
                //     }
                // }
   }
   break ; 
}
}   // end switch numbers 

//---------------------------------------------NAVIGATION----------------------------------------
while (digitalRead(Increment)==1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1)
{
  delay(200);
  programNumber++; 
}
if(digitalRead(Decrement)==1)
{
  delay(200);
  programNumber--;  
}
if (programNumber>12)  programNumber=1;
if (programNumber<1)   programNumber=12;
} // end while increment and decrement 
}  // end main while 
}  // end function 
//-----------------------------Setting Hour Timer 1-----------------------------
void SetTimerOn_1()
{
delay(500);
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
while (digitalRead(Set)==1 )
{
	 currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf(txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
     lcd.setCursor(9,0);
	 lcd.print("    ");
 	 }


//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
sprintf(txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_1++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_1--;
}
//-> perfect
if (minutes_lcd_1>59)    minutes_lcd_1=0;
if (minutes_lcd_1<0) minutes_lcd_1=0;
SecondsRealTimePv_ReConnect_T1=0;
Timer_isOn=0;
relayState_1=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
	 currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf(txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
   lcd.setCursor(4,0);
	 lcd.print("    ");
 	 }
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
   sprintf(txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
   lcd.setCursor(0,0);
   lcd.print(txt);
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_1++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_1--;
}

if  (hours_lcd_1>23) hours_lcd_1=0;
if  (hours_lcd_1<0) hours_lcd_1=0;
Timer_isOn=0; //
relayState_1=0;
SecondsRealTimePv_ReConnect_T1=0;
} // end while increment
} // end first while
lcd.clear();
EEPROM.write(0,hours_lcd_1); // save hours 1 timer tp eeprom
EEPROM.write(1,minutes_lcd_1); // save minutes 1 timer tp eeprom
delay(500);
}
//--------------------------------Set Timer 1 Off ------------------------------
void SetTimerOff_1()
{

delay(500);
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
while (digitalRead(Set)==1 )
{


  currentMillis_2=millis();
   if(currentMillis_2-previousMiliis_2 >=500)
	 {
   previousMiliis_2=currentMillis_2;
   sprintf(txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
     lcd.setCursor(9,0);
	 lcd.print("    ");
 	 }


//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
   sprintf(txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
   lcd.setCursor(0,0);
   lcd.print(txt);
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_2++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_2--;
}
//-> perfect
if (minutes_lcd_2>59)    minutes_lcd_2=0;
if (minutes_lcd_2<0) minutes_lcd_2=0;
SecondsRealTimePv_ReConnect_T1=0;
Timer_isOn=0;
relayState_1=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{

  
   currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf(txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
     lcd.setCursor(4,0);
	 lcd.print("    ");
 	 }


 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
  sprintf(txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
  lcd.setCursor(0,0);
  lcd.print(txt);
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_2++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_2--;
}

if  (hours_lcd_2>23) hours_lcd_2=0;
if  (hours_lcd_2<0) hours_lcd_2=0;
Timer_isOn=0; //
relayState_1=0;
SecondsRealTimePv_ReConnect_T1=0;
} // end while increment
} // end first while
lcd.clear();
EEPROM.write(2,hours_lcd_2); // save hours off  timer_1 to eeprom
EEPROM.write(3,minutes_lcd_2); // save minutes off timer_1 to eeprom
delay(500);
}
//---------------------------------Set Timer 2--------------------------------
void SetTimerOn_2()
{
delay(500);
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;

while (digitalRead(Set)==1 )
{
   currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf(txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
   lcd.setCursor(9,0);
	 lcd.print("    ");
 	 }


//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{

   sprintf(txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
   lcd.setCursor(0,0);
   lcd.print(txt);
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_timer2_start++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_timer2_start--;
}
//-> perfect
if (minutes_lcd_timer2_start>59)    minutes_lcd_timer2_start=0;
if (minutes_lcd_timer2_start<0) minutes_lcd_timer2_start=0;
Timer_2_isOn=0; //
relayState_2=0;
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
  currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf(txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
   lcd.setCursor(4,0);
	 lcd.print("    ");
 	 }



 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{

   sprintf(txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
   lcd.setCursor(0,0);
   lcd.print(txt);
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_timer2_start++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_timer2_start--;
}

if  (hours_lcd_timer2_start>23) hours_lcd_timer2_start=0;
if  (hours_lcd_timer2_start<0) hours_lcd_timer2_start=0;
Timer_2_isOn=0; //
relayState_2=0;
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment
} // end first while
lcd.clear();
EEPROM.write(4,hours_lcd_timer2_start); // save hours 1 timer tp eeprom
EEPROM.write(5,minutes_lcd_timer2_start); // save minutes 1 timer tp eeprom
delay(500);
}
//-----------------------------Set Timer 2 off-----------------------------
void SetTimerOff_2()
{
delay(500);
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;

while (digitalRead(Set)==1 )
{

  currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf(txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
   lcd.setCursor(9,0);
	 lcd.print("    ");
 	 }


//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
sprintf(txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
if (digitalRead(Increment)==1 )
{
delay(200);
minutes_lcd_timer2_stop++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
minutes_lcd_timer2_stop--;
}
//-> perfect
if (minutes_lcd_timer2_stop>59)    minutes_lcd_timer2_stop=0;
if (minutes_lcd_timer2_stop<0) minutes_lcd_timer2_stop=0;
Timer_2_isOn=0; //
relayState_2=0;
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment and decrement
} // end first while
//******************************************************************************
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{

  currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf(txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
   lcd.setCursor(9,0);
	 lcd.print("    ");
 	 }

 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
   sprintf(txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
   lcd.setCursor(0,0);
   lcd.print(txt);
if (digitalRead(Increment) == 1 )
{
delay(200);
hours_lcd_timer2_stop++;
}
if (digitalRead(Decrement) == 1 )
{
delay(200);
hours_lcd_timer2_stop--;
}

if  (hours_lcd_timer2_stop>23) hours_lcd_timer2_stop=0;
if  (hours_lcd_timer2_stop<0) hours_lcd_timer2_stop=0;
Timer_2_isOn=0; //
relayState_2=0;
SecondsRealTimePv_ReConnect_T2=0;
} // end while increment
} // end first while
lcd.clear();
EEPROM.write(6,hours_lcd_timer2_stop); // save hours off  timer_1 to eeprom
EEPROM.write(7,minutes_lcd_timer2_stop); // save minutes off timer_1 to eeprom
delay(500); 
}
//--------------------------Set Battery Voltage---------------------------------
  void SetLowBatteryVoltage()
  {

  delay(500);
  while (digitalRead(Set)==1 )
  {
    
  /*
  lcd.setCursor(0,0);
  lcd.print("[5] LV1");
  dtostrf(Mini_Battery_Voltage,4,1,txt);
  lcd.setCursor(8,0);
  lcd.print(txt);
  lcd.setCursor(12,0);
  if(batteryTypeLiPo4==0) lcd.print("V   "); 
  if(batteryTypeLiPo4==1) lcd.print("%   ");
  */
 lcd.setCursor(0,1);
lcd.print("LV1");
if(batteryTypeLiPo4==0)
{
dtostrf(Mini_Battery_Voltage,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
} 
 
if(batteryTypeLiPo4==1)
{ 
dtostrf(Mini_Battery_Voltage,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}
  //-> to make sure that the value will never be changed until the user press increment or decrement
  while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
  {
lcd.setCursor(0,1);
lcd.print("LV1");
if(batteryTypeLiPo4==0)
{
dtostrf(Mini_Battery_Voltage,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
} 
 
if(batteryTypeLiPo4==1)
{ 
dtostrf(Mini_Battery_Voltage,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}

  //********************************************* */
  if (batteryTypeLiPo4==0)
  {
  if (digitalRead(Increment)==1)
  {
  delay(100);
  Mini_Battery_Voltage+=0.1;
  }

  if (digitalRead(Decrement)==1 )
  {
  delay(100);
  Mini_Battery_Voltage-=0.1;
  }
  } // end if batteryType
  //********************************************* */
  if (batteryTypeLiPo4==1)
  {
  if (digitalRead(Increment)==1)
  {
  delay(100);
  Mini_Battery_Voltage+=1.0;
  }

  if (digitalRead(Decrement)==1 )
  {
  delay(100);
  Mini_Battery_Voltage-=1.0;
  }
  } 
  //*************************************************** */
  //-> perfect
  if (batteryTypeLiPo4==0)
  {
  if (Mini_Battery_Voltage>65) Mini_Battery_Voltage=0;
  if (Mini_Battery_Voltage<0) Mini_Battery_Voltage=0;
  }
  else 
  {
  if (Mini_Battery_Voltage>100)    Mini_Battery_Voltage=100;
  if (Mini_Battery_Voltage<0)      Mini_Battery_Voltage=0;    
  }
  } // end while increment and decrement
  }
  //- save to eeporm

  EEPROM.put(8, Mini_Battery_Voltage);
//-------------------------------------T2-----------------------------------------

delay(500);
while (digitalRead(Set)==1 )
{
lcd.setCursor(0,1);
lcd.print("LV2");
if(batteryTypeLiPo4==0)
{
dtostrf(Mini_Battery_Voltage_T2,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
} 
 
if(batteryTypeLiPo4==1)
{ 
dtostrf(Mini_Battery_Voltage_T2,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
lcd.setCursor(0,1);
lcd.print("LV2");
if(batteryTypeLiPo4==0)
{
dtostrf(Mini_Battery_Voltage_T2,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
} 
 
if(batteryTypeLiPo4==1)
{ 
dtostrf(Mini_Battery_Voltage_T2,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}
//*********************************************************** */
if(batteryTypeLiPo4==0){
if (digitalRead(Increment)==1 )
{
delay(100);
Mini_Battery_Voltage_T2+=0.1;
}
if (digitalRead(Decrement)==1  )
{
delay(100);
Mini_Battery_Voltage_T2-=0.1;
}
} // end if battery type
//****************************************************** */
if(batteryTypeLiPo4==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
Mini_Battery_Voltage_T2+=1.0;
}
if (digitalRead(Decrement)==1  )
{
delay(100);
Mini_Battery_Voltage_T2-=1.0;
}
} // end if battery type
//***************************************************** */
//-> perfect
if(batteryTypeLiPo4==0)
{
if (Mini_Battery_Voltage_T2>65)    Mini_Battery_Voltage_T2=0;
if (Mini_Battery_Voltage_T2<0) Mini_Battery_Voltage_T2=0;
} 
else 
{
 if (Mini_Battery_Voltage_T2>100)    Mini_Battery_Voltage_T2=100.0;
if (Mini_Battery_Voltage_T2<0) Mini_Battery_Voltage_T2=0;   
}
} // end while increment and decrement
}
//-> save to eepprom
lcd.clear();
EEPROM.put(12,Mini_Battery_Voltage_T2);
delay(500); 
} //- end set low voltage
//-----------------------------------Set Start Voltage----------------------------------------------
void SetStartUpLoadsVoltage()
{

delay(500);
while (digitalRead(Set)==1 )
{

  lcd.setCursor(0,1);
lcd.print("HV1 ");
if(batteryTypeLiPo4==0)
{
dtostrf(StartLoadsVoltage,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
}
if(batteryTypeLiPo4==1) 
{
dtostrf(StartLoadsVoltage,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}

//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
lcd.setCursor(0,1);
lcd.print("HV1 ");
if(batteryTypeLiPo4==0)
{
dtostrf(StartLoadsVoltage,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
}
if(batteryTypeLiPo4==1) 
{
dtostrf(StartLoadsVoltage,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}
//********************************************** */
if (batteryTypeLiPo4==0)
{
if (digitalRead(Increment)==1 )
{
delay(100);
StartLoadsVoltage+=0.1;
}
if (digitalRead(Decrement)==1)
{
delay(100);
StartLoadsVoltage-=0.1;
}
}
//************************************************ */
if (batteryTypeLiPo4==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
StartLoadsVoltage+=1.0;
}
if (digitalRead(Decrement)==1)
{
delay(100);
StartLoadsVoltage-=1.0;
}
}
//********************************************* */

//-> perfect
if(batteryTypeLiPo4==0)
{
if (StartLoadsVoltage>65)    StartLoadsVoltage=0;
if (StartLoadsVoltage<0)     StartLoadsVoltage=0;
}
else 
{
if (StartLoadsVoltage>100)    StartLoadsVoltage=0;
if (StartLoadsVoltage<0)     StartLoadsVoltage=0;
}
} // end while increment and decrement
//--------------------------------------------------
} 

//- save to eeporm
EEPROM.put(16,StartLoadsVoltage);
//-------------------------------------T2-----------------------------------------
delay(500);
while (digitalRead(Set)==1 )
{
    /*
lcd.setCursor(0,0);
lcd.print("[6] HV2 ");
dtostrf(StartLoadsVoltage_T2,4,1,txt);
lcd.setCursor(8,0);
lcd.print(txt);
lcd.setCursor(12,0);
if(batteryTypeLiPo4==0) lcd.print("V   "); 
if(batteryTypeLiPo4==1) lcd.print("%   ");
*/
lcd.setCursor(0,1);
lcd.print("HV2 ");
if(batteryTypeLiPo4==0)
{
dtostrf(StartLoadsVoltage_T2,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
}
if(batteryTypeLiPo4==1) 
{
dtostrf(StartLoadsVoltage_T2,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
lcd.setCursor(0,1);
lcd.print("HV2 ");
if(batteryTypeLiPo4==0)
{
dtostrf(StartLoadsVoltage_T2,4,1,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("V   "); 
}
if(batteryTypeLiPo4==1) 
{
dtostrf(StartLoadsVoltage_T2,4,0,txt);
lcd.setCursor(8,1);
lcd.print(txt);
lcd.setCursor(12,1);
lcd.print("%   ");
}
//***************************************** */
if(batteryTypeLiPo4==0)
{
if (digitalRead(Increment)==1 )
{
delay(100);
StartLoadsVoltage_T2+=0.1;
}
if (digitalRead(Decrement)==1)
{
delay(100);
StartLoadsVoltage_T2-=0.1;
}
}
//************************************************/
if(batteryTypeLiPo4==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
StartLoadsVoltage_T2+=1.0;
}
if (digitalRead(Decrement)==1)
{
delay(100);
StartLoadsVoltage_T2-=1.0;
}
}
//***********************************************/

//-> perfect
if (batteryTypeLiPo4==0)
{
if (StartLoadsVoltage_T2>65)    StartLoadsVoltage_T2=0;
if (StartLoadsVoltage_T2<0)     StartLoadsVoltage_T2=0;
}
else 
{
 if (StartLoadsVoltage_T2>100)    StartLoadsVoltage_T2=100;
if (StartLoadsVoltage_T2<0)     StartLoadsVoltage_T2=0; 
}

} // end while increment and decrement
}
//-> save to eepprom
lcd.clear();
EEPROM.put(20,StartLoadsVoltage_T2);
delay(500); 
} // end startupvoltage
//------------------------------------ Startup Timer 1 -----------------------------------------
void Startup_Timers()
{
delay(500);
while (digitalRead(Set)==1 )
{
sprintf((char*)txt,"[8] T1 ON %02d S  ",startupTIme_1);
lcd.setCursor(0,0);
lcd.print(txt);
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
sprintf((char*)txt,"[8] T1 ON %02d S  ",startupTIme_1);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Increment)==1 )
{
delay(50);
startupTIme_1++;
}
if (digitalRead(Decrement)==1)
{
delay(50);
startupTIme_1--;
}
//-> perfect
if (startupTIme_1>900)    startupTIme_1=900;
if (startupTIme_1<0) startupTIme_1=0;
} // end while increment and decrement
} // end first while
EEPROM.put(24,startupTIme_1);
//******************************************************************************
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
sprintf((char*)txt,"[8]T2 ON %02d S  ",startupTIme_2);
lcd.setCursor(0,0);
lcd.print(txt);
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
sprintf((char*)txt,"[8]T2 ON %02d S  ",startupTIme_2);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Increment) == 1 )
{
delay(50);
startupTIme_2++;
}
if (digitalRead(Decrement) == 1 )
{
delay(50);
startupTIme_2--;
}

if  (startupTIme_2>900) startupTIme_2=900;
if  (startupTIme_2<0) startupTIme_2=0;
} // end while increment
} // end first while
lcd.clear();
EEPROM.put(26,startupTIme_2);
delay(500); 
} // end startup timer

//------------------------------------DELAY OFF------------------------------------------------
void DelayOff()
{
delay(500);
while (digitalRead(Set)==1 )
{
sprintf(txt,"[9]T1 OFF %02d S  ",offDelay_1);
lcd.setCursor(0,0);
lcd.print(txt);
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
sprintf(txt,"[9]T1 OFF %02d S  ",offDelay_1);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Increment)==1 )
{
delay(50);
offDelay_1++;
}
if (digitalRead(Decrement)==1)
{
delay(50);
offDelay_1--;
}
//-> perfect
if (offDelay_1>240)    offDelay_1=240;
if (offDelay_1<1)     offDelay_1=1;
} // end while increment and decrement
} // end first while
EEPROM.put(35,offDelay_1);
//******************************************************************************
delay(500);     //read time for state
while (digitalRead(Set)==1 )
{
sprintf(txt,"[9]T2 OFF %02d S  ",offDelay_2);
lcd.setCursor(0,0);
lcd.print(txt);
 //-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement) == 1 )
{
sprintf(txt,"[9]T2 OFF %02d S  ",offDelay_2);
lcd.setCursor(0,0);
lcd.print(txt);
if (digitalRead(Increment) == 1 )
{
delay(50);
offDelay_2++;
}
if (digitalRead(Decrement) == 1 )
{
delay(50);
offDelay_2--;
}

if  (offDelay_2>240) offDelay_2=240;
if  (offDelay_2<1) offDelay_2=1;
} // end while increment
} // end first while
lcd.clear();
EEPROM.put(37,offDelay_2);
delay(500); 
}
//------------------------------------Set Time--------------------------------------------------


void SetDS1307_Time()
{
DateTime now = rtc.now();
delay(500);
set_ds1307_hours=now.hour();
set_ds1307_minutes=now.minute();
//set_ds1307_day=now.day();
//set_ds1307_month=now.month();
//set_ds1307_year=now.year();
set_ds1307_day=1;
set_ds1307_month=1;
set_ds1307_year=24;
currentMillis_1=0,previousMiliis_1=0;
currentMillis_2=0;previousMiliis_2=0;
while (digitalRead(Set)==1 )
{
/*
   currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf((char*)txt,"[12] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
   lcd.setCursor(5,0);
	 lcd.print("    ");
 	 }
*/
   sprintf((char*)txt,"[12] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
   lcd.setCursor(0,0);
   lcd.print(txt);
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
  sprintf((char*)txt,"[12] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
  lcd.setCursor(0,0);
  lcd.print(txt);
if (digitalRead(Increment)==1 )
{
delay(200);
set_ds1307_hours++;
}
if (digitalRead(Decrement)==1)
{
delay(200);
set_ds1307_hours--;
}
//-> perfect
if (set_ds1307_hours>23)    set_ds1307_hours=0;
if (set_ds1307_hours<0) set_ds1307_hours=0;
} // end while increment and decrement
} // end first while
//-----------------------------------------Set Minutes------------------------------------------

currentMillis_1=0,previousMiliis_1=0;
currentMillis_2=0;previousMiliis_2=0;
delay(500); 
while (digitalRead(Set)==1 )
{
  /*
   currentMillis_2=millis();
	 if(currentMillis_2-previousMiliis_2 >=500)
	 {
	 previousMiliis_2=currentMillis_2;
   sprintf((char*)txt,"[12] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
   lcd.setCursor(0,0);
   lcd.print(txt);
	 }

	 currentMillis_1=millis();
	 if(currentMillis_1-previousMiliis_1 >=1000)
	 {
	 previousMiliis_1=currentMillis_1;
   lcd.setCursor(10,0);
	 lcd.print("    ");
 	 }

*/
   sprintf((char*)txt,"[12] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
   lcd.setCursor(0,0);
   lcd.print(txt);
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{

  sprintf((char*)txt,"[12] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
  lcd.setCursor(0,0);
  lcd.print(txt);
if (digitalRead(Increment)==1 )
{
delay(100);
set_ds1307_minutes++;
}
if (digitalRead(Decrement)==1)
{
delay(100);
set_ds1307_minutes--;
}
//-> perfect
if (set_ds1307_minutes>59)    set_ds1307_minutes=0;
if (set_ds1307_minutes<0)     set_ds1307_minutes=0;
} // end while increment and decrement
} // end first while
rtc.adjust(DateTime(set_ds1307_year,set_ds1307_month,set_ds1307_day,set_ds1307_hours, set_ds1307_minutes, 0));
lcd.clear();
delay(500);
} // end this function
//------------------------------------------SET DATE--------------------------------------------
/*
while (digitalRead(Set)==1 )
{
sprintf(txt,"[12] %02d/%02d/%04d",set_ds1307_day,set_ds1307_month,set_ds1307_year);
lcd.setCursor(0,0);
lcd.print(txt);
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
set_ds1307_day++;
}
if (digitalRead(Decrement)==1)
{
delay(100);
set_ds1307_day--;
}
//-> perfect
if (set_ds1307_day>31)    set_ds1307_day=0;
if (set_ds1307_day<0)     set_ds1307_day=0;
} // end while increment and decrement
} // end first while
//----------------------------------------Set Month---------------------------------
delay(500);
while (digitalRead(Set)==1 )
{
sprintf(txt,"[12] %02d/%02d/%04d",set_ds1307_day,set_ds1307_month,set_ds1307_year);
lcd.setCursor(0,0);
lcd.print(txt);
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
set_ds1307_month++;
}
if (digitalRead(Decrement)==1)
{
delay(100);
set_ds1307_month--;
}
//-> perfect
if (set_ds1307_month>12)    set_ds1307_month=0;
if (set_ds1307_month<0)     set_ds1307_month=0;
} // end while increment and decrement

} // end first while
//------------------------------------Set year--------------------------------------------
delay(500);
while (digitalRead(Set)==1 )
{
sprintf(txt,"[12] %02d/%02d/%04d",set_ds1307_day,set_ds1307_month,set_ds1307_year);
lcd.setCursor(0,0);
lcd.print(txt);
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
if (digitalRead(Increment)==1 )
{
delay(100);
set_ds1307_year++;
}
if (digitalRead(Decrement)==1)
{
delay(100);
set_ds1307_year--;
}
//-> perfect
if (set_ds1307_year>2030)    set_ds1307_year=0;
if (set_ds1307_year<0)     set_ds1307_year=0;
} // end while increment and decrement
} // end first while
rtc.adjust(DateTime(set_ds1307_year,set_ds1307_month,set_ds1307_day,set_ds1307_hours, set_ds1307_minutes, 0));
lcd.clear();
delay(500);
} // end setDS1307
*/
//----------------------------------SET VOLTAGE MODE-------------------------------------------
void SetVoltageMode()
{
delay(500);
while (digitalRead(Set)==1)
{
sprintf(txt,"[10] Timer Mode   ");
lcd.setCursor(0,0);
lcd.print(txt); 

if(RunOnBatteryVoltageMode==0)
{
lcd.setCursor(7,1);
lcd.print("ON ");
}
if(RunOnBatteryVoltageMode==1)
{
lcd.setCursor(7,1);
lcd.print("OFF");
}

while (digitalRead(Increment)==1  || digitalRead(Decrement)==1)
{
if(digitalRead(Increment)==1)
{
  delay(100);
  RunOnBatteryVoltageMode=1;
}
if(digitalRead(Decrement)==1)
{
  delay(100);
  RunOnBatteryVoltageMode=0;
}
} // end while increment and decrement 
} // end while program
lcd.clear();
EEPROM.write(28,RunOnBatteryVoltageMode);    // run on battery voltage mode
delay(500);
} // END FUNCTION 

//-------------------------------------------------------------
void SetUPSMode()
{
delay(500);
while (digitalRead(Set)==1)
{

if(UPSMode==0)
{
lcd.setCursor(7,1);
lcd.print("OFF");
}
if(UPSMode==1)
{
lcd.setCursor(7,1);
lcd.print("ON ");
}

while (digitalRead(Increment)==1  || digitalRead(Decrement)==1)
{
if(digitalRead(Increment)==1)
{
  delay(100);
  UPSMode=1;
}
if(digitalRead(Decrement)==1)
{
  delay(100);
  UPSMode=0;
}
} // end while increment and decrement 
} // end while program
lcd.clear();
EEPROM.write(29,UPSMode); // ups mode
delay(500);
}

//--------------------------------SET BATTERY VOLTAGE CALLIBRATION-------------------------------
/*
void SetBatteryVoltageError()
{
delay(500);
while (digitalRead(Set)==1 )
{
VinBatteryError=Vin_Battery;
sprintf(txt,"[%1d]  Batt Call  ",programNumber);
lcd.setCursor(0,0);
lcd.print(txt); 
lcd.setCursor(0,1);
dtostrf(VinBatteryError,4,1,txt);
lcd.setCursor(6,1);
lcd.print(txt);
lcd.setCursor(10,1);
lcd.print("V   ");
if (digitalRead(Exit)==1 )
{
break;     //break out of the while loop
}
//-> to make sure that the value will never be changed until the user press increment or decrement
while (digitalRead(Increment) == 1 || digitalRead(Decrement)==1)
{
sprintf(txt,"[%1d]  Batt Call  ",programNumber);
lcd.setCursor(0,0);
lcd.print(txt); 
lcd.setCursor(0,1);
dtostrf(VinBatteryError,4,1,txt);
lcd.setCursor(6,1);
lcd.print(txt);
lcd.setCursor(10,1);
lcd.print("V   ");
if (digitalRead(Increment)==1 )
{
delay(50);
VinBatteryError+=0.1;
}
if (digitalRead(Decrement)==1)
{
delay(50);
VinBatteryError-=0.1;
}
if(VinBatteryError > 70.0  ) VinBatteryError=0;
if (VinBatteryError<0) VinBatteryError=0;
if (VinBatteryError>=Vin_Battery_) addError=1;    // add
if (VinBatteryError<Vin_Battery_) addError=0;    // minus
} // end while increment and decrement
Vin_Battery=VinBatteryError;
}
lcd.clear();
//-> i moved the operation to here because of errors
VinBatteryDifference=fabs(VinBatteryError-Vin_Battery_);
EEPROM.write(30,addError);
EEPROM.put(31,VinBatteryDifference); 
delay(500);
}
*/
//----------------------------------SET BATTERY TYPE----------------------------------------
void SetBatteryType()
{

delay(500);
while (digitalRead(Set)==1)
{

if(batteryTypeLiPo4==0)
{
lcd.setCursor(7,1);
lcd.print("USER ");
}
if(batteryTypeLiPo4==1)
{
lcd.setCursor(7,1);
lcd.print("LiPo4");
}

while (digitalRead(Increment)==1  || digitalRead(Decrement)==1)
{
if(digitalRead(Increment)==1)
{
  delay(100);
  batteryTypeLiPo4=1;
}
if(digitalRead(Decrement)==1)
{
  delay(100);
  batteryTypeLiPo4=0;
}
} // end while increment and decrement 
} // end while program
lcd.clear();
EEPROM.write(50,batteryTypeLiPo4); // ups mode
delay(500);

}
 //-------------------------------CHECK TIMER IN RANGE--------------------------------------------
 //-------------------Check for timer activation inside range--------------------
 void CheckForTimerActivationInRange()
 {
 if (RunOnBatteryVoltageMode==0)
 {
 //-> first compare is hours
 if(ReadHours() > hours_lcd_1 && ReadHours()< hours_lcd_2)
 {
 Timer_isOn=1;

 }
 //-> seconds compare hours if equal now then compare minutes
 if(ReadHours()== hours_lcd_1 || ReadHours()== hours_lcd_2)
 {
 if(ReadHours()==hours_lcd_1)
 {
 //-> minutes must be bigger
 if(ReadMinutes()>=minutes_lcd_1) 
 {

  Timer_isOn=1;
 }
 }
 if(ReadHours()==hours_lcd_2)
 {
 //-> minutes must be less
 if(ReadMinutes()< minutes_lcd_2) 
 {
  
  Timer_isOn=1;
 }
 }
 }
 //------------------------------Timer 2-----------------------------------------
 if(ReadHours() > hours_lcd_timer2_start && ReadHours()< hours_lcd_timer2_stop)
 {
 Timer_2_isOn=1;

 }
 //-> seconds compare hours if equal now then compare minutes
 if(ReadHours()== hours_lcd_timer2_start || ReadHours()== hours_lcd_timer2_stop )
 {
 if(ReadHours()==hours_lcd_timer2_start)
 {
 //-> minutes must be bigger
 if(ReadMinutes()>=minutes_lcd_timer2_start) 
 {
 
  Timer_2_isOn=1;
 }
 }
 if(ReadHours()==hours_lcd_timer2_stop)
 {
 //-> minutes must be less
 if(ReadMinutes()<minutes_lcd_timer2_stop) 
 {
  
  Timer_2_isOn=1;
 }
 }
 }
 } // run on battery voltage mode
 }  // end function
 //**************************************1****************************************
 void CheckForTimerActivationOutRange()
 {
 if (RunOnBatteryVoltageMode==0)
 {
 //------------------------------First Timer-------------------------------------
 if (ReadHours() < hours_lcd_1  && ReadHours() < hours_lcd_2 )
 {
 Timer_isOn=0;
 
 }

 if (ReadHours() > hours_lcd_1  && ReadHours() > hours_lcd_2 )
 {
 Timer_isOn=0;
 
 }


 if (ReadHours()==hours_lcd_1)
 {
 if(ReadMinutes() < minutes_lcd_1)
 {
 Timer_isOn=0;
 
 }
 }
 //-> check for hours
 if (ReadHours()==hours_lcd_2)
 {
 if(ReadMinutes() > minutes_lcd_2)
 {
 Timer_isOn=0;
 
 }
 }
 //----------------------------Second Timer--------------------------------------
 if (ReadHours() < hours_lcd_timer2_start  && ReadHours() < hours_lcd_timer2_stop )
 {
 Timer_2_isOn=0;
 
 }

 if (ReadHours() > hours_lcd_timer2_start  && ReadHours() > hours_lcd_timer2_stop )
 {
 Timer_2_isOn=0;
 
 }


 if (ReadHours()==hours_lcd_timer2_start)
 {
 if(ReadMinutes() < minutes_lcd_timer2_start)
 {
 Timer_2_isOn=0;
 
 }
 }
 //-> check for hours
 if (ReadHours()==hours_lcd_timer2_stop)
 {
 if(ReadMinutes() > minutes_lcd_timer2_stop)
 {
 Timer_2_isOn=0;
 
 }
 }
 //--------------------------------End of Second Timer
 } // end of run on battery voltage
 }
 //-----------------------------------Read Time-------------------------------------------------
 //-------------------------------Read Seconds-----------------------------------
 unsigned short ReadSeconds()
 {
  DateTime now = rtc.now();
 Full_Seconds=now.second();
 return Full_Seconds;
 }

 //-------------------------------Read Minutes-----------------------------------
 unsigned short ReadMinutes()
 {
  DateTime now = rtc.now();
 	Full_Seconds=now.minute();
 	return Full_Seconds;
 }
 //----------------------------Read hours----------------------------------------
 unsigned short ReadHours()
 {
  DateTime now = rtc.now();

 	Full_Seconds=now.hour();
 	return Full_Seconds;
 }

 //----------------------------EEPROM Load------------------------------------------------
 void EEPROM_Load()
 {
//*****************timer 1****************
hours_lcd_1=EEPROM.read(0);
minutes_lcd_1=EEPROM.read(1);
hours_lcd_2=EEPROM.read(2);
minutes_lcd_2=EEPROM.read(3);
//*****************timer 2*****************
hours_lcd_timer2_start=EEPROM.read(4);
minutes_lcd_timer2_start=EEPROM.read(5);
hours_lcd_timer2_stop=EEPROM.read(6);
minutes_lcd_timer2_stop=EEPROM.read(7);
//**********************************************
ByPassState=0;   // enable is zero  // delete function to be programmed for rom spac
Timer_Enable=1;      // delete function to be programmed for rom space
VinBatteryDifference=0;
RunOnBatteryVoltageMode=EEPROM.read(28);
UPSMode=EEPROM.read(29) ;   // ups mode
addError=EEPROM.read(30);
batteryTypeLiPo4=EEPROM.read(50);

EEPROM.get(8,Mini_Battery_Voltage);
EEPROM.get(16,StartLoadsVoltage);
EEPROM.get(24,startupTIme_1);
EEPROM.get(26,startupTIme_2);
EEPROM.get(12,Mini_Battery_Voltage_T2);
EEPROM.get(20,StartLoadsVoltage_T2);
//EEPROM.get(31,VinBatteryDifference);
EEPROM.get(35,offDelay_1);
EEPROM.get(37,offDelay_2);
}
//------------------------------------------RunTimersCheckNow---------------------------------------
void RunTimersNowCheck()
{
  
//---------------------------------Factory Settings-------------------------
if (digitalRead(Increment)==1 && digitalRead(Decrement)==1)      // first
{
LCD_ReConfig();
delay(1000);
if ( digitalRead(Increment)==1  && digitalRead(Decrement)==1)
{
delay(1000);
EEPROM_FactorySettings(1);        // summer time
//delay(100);
EEPROM_Load();    // read the new values from epprom
lcd.setCursor(0,1);
lcd.print("Factory Settings");
delay(1000);
lcd.clear();
}
}
//-----------------------------Bypass Mode -------------------------------------
if(digitalRead(Increment)==1 && digitalRead(Decrement)==0)
{
delay(1000);
if (digitalRead(Increment)==1 && digitalRead(Decrement)==0)
{
delay(1000);
if (digitalRead(Increment)==1 && digitalRead(Exit)==0)
{
RunLoadsByBass++;
if (  RunLoadsByBass==1 ) digitalWrite(Relay_L_Solar,1);
if (RunLoadsByBass>=2 )
{
digitalWrite(Relay_L_Solar_2,1);
}
}
}
}
//-----------------------------END BYPASS MODE--------------------------------
}  // end function
//-----------------------------------------EEPROM Factory Settings----------------------------
void EEPROM_FactorySettings(char period)
{
if(period==1) // summer  timer
{
if(SystemBatteryMode==12 )
{
Mini_Battery_Voltage=12.0;
StartLoadsVoltage=13.0;
Mini_Battery_Voltage_T2=12.3,
StartLoadsVoltage_T2=13.2;
}
if(SystemBatteryMode==24)
{
Mini_Battery_Voltage=30.0;
StartLoadsVoltage=60.0;
Mini_Battery_Voltage_T2=35.0,
StartLoadsVoltage_T2=70.0;
}
if(SystemBatteryMode==48)
{
Mini_Battery_Voltage=49.0;
StartLoadsVoltage=51.0;
Mini_Battery_Voltage_T2=50.0,
StartLoadsVoltage_T2=52.0;
}

// if(batteryTypeLiPo4==1) 
// {
// Mini_Battery_Voltage=60.0;
// StartLoadsVoltage=80.0;
// Mini_Battery_Voltage_T2=70.0,
// StartLoadsVoltage_T2=90.0; 
// }
startupTIme_1 =15;
startupTIme_2=20;
offDelay_1=15;
offDelay_2=25;
addError=1;
VinBatteryDifference=0.0;
batteryTypeLiPo4=1; // default is user battery 

//*****************timer 1****************
EEPROM.write(0,8);  // writing start hours
EEPROM.write(1,0);    // writing  start minutes
EEPROM.write(2,17);    // writing off hours
EEPROM.write(3,0);    // writing off minutes
//****************timer 2********************
EEPROM.write(4,9);  // writing start hours
EEPROM.write(5,0);    // writing  start minutes
EEPROM.write(6,17);    // writing off hours
EEPROM.write(7,0);    // writing off minutes
EEPROM.write(28,1);    // run on battery voltage mode
EEPROM.write(29,1);   // ups mode default is on 
//EEPROM.write(30,addError);
EEPROM.write(50,batteryTypeLiPo4);  // default is user
//**********************************************
EEPROM.put(8,Mini_Battery_Voltage);
EEPROM.put(12,Mini_Battery_Voltage_T2);
EEPROM.put(16,StartLoadsVoltage);
EEPROM.put(20,StartLoadsVoltage_T2);
EEPROM.put(24,startupTIme_1);
EEPROM.put(26,startupTIme_2);
//EEPROM.put(31,VinBatteryDifference);
EEPROM.put(35,offDelay_1);
EEPROM.put(37,offDelay_2);
} // end if period
}
//----------------------------------------Check Battery System Mode------------------------------
void CheckSystemBatteryMode()
{
if (Vin_Battery>= 35 && Vin_Battery <= 70) SystemBatteryMode=48;
else if (Vin_Battery>=18 && Vin_Battery <=32) SystemBatteryMode=24;
else if (Vin_Battery >=1 && Vin_Battery<= 16 ) SystemBatteryMode=12;
else if(Vin_Battery==0) SystemBatteryMode=24;
else SystemBatteryMode=24; // take it as default

}
//----------------------------------LCD Reconfig()---------------------------------
void LCD_ReConfig()
{
digitalWrite(Backlight,1);
UpdateScreenTime=0;
}
//---------------------------------Check Time Occurred ON------------------------------------------
 char CheckTimeOccuredOn(char seconds_required, char minutes_required,char hours_required)
 {
	DateTime now = rtc.now();

	if (now.hour()==hours_required && now.minute()==minutes_required)
	{
	return 1;
	}
	else {
		return 0;
	}
 }
 //---------------------------------Check Time Occured OFF----------------------------------------
 char CheckTimeOccuredOff(char seconds_required, char minutes_required,char hours_required)
 {
	DateTime now = rtc.now();

	if (now.hour()==hours_required && now.minute()==minutes_required)
	{
	return 1;
	}
	else {
		return 0;
	}
 }
//---------------------------------Check Timers-------------------------------------------------
void Check_Timers()
{
if(RunOnBatteryVoltageMode==0)
{
//-> timer start
matched_timer_1_start=CheckTimeOccuredOn(seconds_lcd_1,minutes_lcd_1,hours_lcd_1);
matched_timer_1_stop=CheckTimeOccuredOff(seconds_lcd_2,minutes_lcd_2,hours_lcd_2);
matched_timer_2_start=CheckTimeOccuredOn(seconds_lcd_timer2_start,minutes_lcd_timer2_start,hours_lcd_timer2_start);
matched_timer_2_stop=CheckTimeOccuredOff(seconds_lcd_timer2_stop,minutes_lcd_timer2_stop,hours_lcd_timer2_stop);
//---------------------------- Timer 1 -----------------------------------------
//-> turn Load On
if (matched_timer_1_start==1)
{
Timer_isOn=1;
relayState_1=1;
TurnOffLoadsByPass=0;

//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false )
{
digitalWrite(Relay_L_Solar,1);
relayState_1=1;
}
//-> if run with out battery is selected
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true )
{
digitalWrite(Relay_L_Solar,1);
relayState_1=1;
}
} // end if ac_available
//-> Turn Load off
//******************************************************************************
if (matched_timer_1_stop==1)
{
relayState_1=0;
Timer_isOn=0;        // to continue the timer after breakout the timer when grid is available
///EEPROM_write(0x49,0);        //- save it to eeprom if power is cut
//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1 && Timer_Enable==1  &&  RunWithOutBattery==false  )
{
//for the turn off there is no need for delay
SecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T1=0;
digitalWrite(Relay_L_Solar,0); // relay off
relayState_1=0;

}
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true  )
{
//for the turn off there is no need for delay
SecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T1=0;
digitalWrite(Relay_L_Solar,0); // relay off
relayState_1=0;
}
}
//}// end if of ac_available
//-------------------------- Timer 1 End----------------------------------------
//------------------------- Timer 2 Start---------------------------------------
if (matched_timer_2_start==1)
{
Timer_2_isOn=1;
relayState_2=1;
TurnOffLoadsByPass=0;     // this variable just for if user shutdown loads and don't want to reactivated so it will be zeroed until next timer
///EEPROM_write(0x50,1);        //- save it to eeprom if power is cut
//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1 && Timer_Enable==1  && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false)
{
digitalWrite(Relay_L_Solar_2,1);
relayState_2=1;

}

if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true)
{
digitalWrite(Relay_L_Solar_2,1);
relayState_2=1;
}

} // end if ac_available


if (matched_timer_2_stop==1)
{
Timer_2_isOn=0;        // to continue the timer after breakout the timer when grid is available
relayState_2=0;
///EEPROM_write(0x50,0);        //- save it to eeprom if power is cut
//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
if (digitalRead(AC_Available)==1  && Timer_Enable==1 && RunWithOutBattery==false )
{
///SolarOnGridOff_2=0; // to enter once again in the interrupt
//for the turn off there is no need for delay
digitalWrite(Relay_L_Solar_2,0);
relayState_2=0;
SecondsRealTimePv_ReConnect_T2=0;      // MUST Be zero 
CountSecondsRealTimePv_ReConnect_T2=0; // MUST Be zero 

}

if (digitalRead(AC_Available)==1 && Timer_Enable==1  && RunWithOutBattery==true )
{
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar_2,0); // relay off 
relayState_2=0;
}

} // end match timer stop
} //end batteryvoltagemode if

//*******************************************************************************
//-------------------------Bypass System----------------------------------------
if(digitalRead(AC_Available)==0 &&  UPSMode==0 )   // voltage protector is not enabled
{
//delay(250);     // for error to get one seconds approxmiallty
//SecondsRealTime++;
RunLoadsByBass=0; // to make load turn off if user manually turned on loads 
CountSecondsRealTime=1;
relayState_1=1;
relayState_2=1;
if(SecondsRealTime >= startupTIme_1 && digitalRead(AC_Available)==0)
{

digitalWrite(Relay_L_Solar,1);

}
if(SecondsRealTime >= startupTIme_2 && digitalRead(AC_Available)==0)
{

digitalWrite(Relay_L_Solar_2,1);

}

} // end function of voltage protector

//------------------------------Bypass Mode Upo Mode-------------------------------------
 if(digitalRead(AC_Available)==0 && UPSMode==1 )   // voltage protector is not enabled
{
//delay(250);       // for error to get one seconds approxmiallty
//SecondsRealTime++;
RunLoadsByBass=0; // to make load turn off if user manually turned on loads 
CountSecondsRealTime=1;
relayState_1=1;
relayState_2=1;
if( digitalRead(AC_Available)==0 && LoadsAlreadySwitchedOFF==0)
{

LoadsAlreadySwitchedOFF=1;
digitalWrite(Relay_L_Solar,0);
digitalWrite(Relay_L_Solar_2,0);
relayState_1=0;
relayState_2=0;
}
if(SecondsRealTime >= startupTIme_1 && digitalRead(AC_Available)==0 && LoadsAlreadySwitchedOFF==1 )
{
digitalWrite(Relay_L_Solar,1);

}
if(SecondsRealTime >= startupTIme_2 && digitalRead(AC_Available)==0 && LoadsAlreadySwitchedOFF==1 )
{
digitalWrite(Relay_L_Solar_2,1);

}
} // end function of voltage protector
//------------------------Functions for reactiving timers------------------------
/*
 these function is used for reactiving timers when grid available in the same timer is on or off
*/
//-> if the  ac is shutdown and timer is steel in the range of being on  so reactive timer 1
if (digitalRead(AC_Available)==1 && Timer_isOn==1 && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0 )
{

//SecondsRealTimePv_ReConnect_T1++;
CountSecondsRealTimePv_ReConnect_T1=1;
relayState_1=1;
CountCutSecondsRealTime_T1=0;
CutSecondsRealTime_T1=0;
//delay(200);
if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1) 
{   
  digitalWrite(Relay_L_Solar,1);
 
}

}
if (digitalRead(AC_Available)==1 && Timer_isOn==1  && RunWithOutBattery==true && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0 )
{
//econdsRealTimePv_ReConnect_T1++;
CountSecondsRealTimePv_ReConnect_T1=1;
relayState_1=1;
CountCutSecondsRealTime_T1=0;
CutSecondsRealTime_T1=0;
//delay(200);

if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1)  
{
  digitalWrite(Relay_L_Solar,1);
  
}
}
//-> if the  ac is shutdown and timer is steel in the range of being on  so reactive timer 2
if (digitalRead(AC_Available)==1 && Timer_2_isOn==1 && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)     //run with battery
{
//SecondsRealTimePv_ReConnect_T2++;
CountSecondsRealTimePv_ReConnect_T2=1;
 relayState_2=1; 
CountCutSecondsRealTime_T2=0;
CutSecondsRealTime_T2=0;
//delay(50);
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
{
 digitalWrite(Relay_L_Solar_2,1);

}
}

if ( digitalRead(AC_Available)==1 && Timer_2_isOn==1 &&  RunWithOutBattery==true && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)            //run without battery
{
//SecondsRealTimePv_ReConnect_T2++;
CountSecondsRealTimePv_ReConnect_T2=1;
 relayState_2=1; 
CountCutSecondsRealTime_T2=0;
CutSecondsRealTime_T2=0;
//delay(50);
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
{
 digitalWrite(Relay_L_Solar_2,1);
 
}
}
//-------------------------------RunOnBatteryMode-------------------------------
 if ( digitalRead(AC_Available)==1 && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==1 )
{

//SecondsRealTimePv_ReConnect_T1++;
CountSecondsRealTimePv_ReConnect_T1=1;
relayState_1=1; 
CountCutSecondsRealTime_T1=0;
CutSecondsRealTime_T1=0;
//delay(200);
if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1) 
{
      digitalWrite(Relay_L_Solar,1);
       
}
}

if ( digitalRead(AC_Available)==1 && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==1)     //run with battery
{
//SecondsRealTimePv_ReConnect_T2++;
CountSecondsRealTimePv_ReConnect_T2=1;
 relayState_2=1; 
CountCutSecondsRealTime_T2=0;
CutSecondsRealTime_T2=0;
//delay(50);
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
{
   digitalWrite(Relay_L_Solar_2,1);
   
}
}
//------------------------------Turn Off Loads----------------------------------
//--Turn Load off when battery Voltage  is Low and AC Not available and Bypass is enabled
if (Vin_Battery<Mini_Battery_Voltage &&  digitalRead(AC_Available)==1  && RunWithOutBattery==false )
{
CountCutSecondsRealTime_T1=1;
relayState_1=0;
Start_Timer_0_A();         // give some time for battery voltage
}

//--Turn Load off when battery Voltage  is Low and AC Not available and Bypass is enabled
if (Vin_Battery<Mini_Battery_Voltage_T2 &&  digitalRead(AC_Available)==1  &&  RunWithOutBattery==false )
{
CountCutSecondsRealTime_T2=1;
relayState_2=0;
Start_Timer_0_A();         // give some time for battery voltage
}
}// end of check timers
//----------------------------------------Start Timer-+-----------------------------------------
void Start_Timer_0_A()
{
//Read_Battery();
 //********************************Turn Off loads*******************************
if( CutSecondsRealTime_T1>= offDelay_1 &&  Vin_Battery<Mini_Battery_Voltage && digitalRead(AC_Available)==1 && RunLoadsByBass==0 )
{
CutSecondsRealTime_T1=0;
CountCutSecondsRealTime_T1=0;
CountSecondsRealTimePv_ReConnect_T1=0;
SecondsRealTimePv_ReConnect_T1=0;
digitalWrite(Relay_L_Solar,0);
relayState_1=0; 
}

if( CutSecondsRealTime_T2>= offDelay_2 && Vin_Battery<Mini_Battery_Voltage_T2 && digitalRead(AC_Available)==1  && RunLoadsByBass==0)
{
CutSecondsRealTime_T2=0;
CountCutSecondsRealTime_T2=0;
CountSecondsRealTimePv_ReConnect_T2=0;
SecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar_2,0);
relayState_2=0; 
}

} //end start timer
//-----------------------------------------Turn Off Loads Grid----------------------------------
void TurnLoadsOffWhenGridOff()
{
//delay(500);
if( digitalRead(AC_Available)==1 && Timer_isOn==0 && RunLoadsByBass==0  && RunOnBatteryVoltageMode==0)
{
SecondsRealTime=0;
CountSecondsRealTime=0;
SecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T1=0;
digitalWrite(Relay_L_Solar,0);
relayState_1=0; 

}

if (digitalRead(AC_Available)==1 && Timer_2_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
{
SecondsRealTime=0;
CountSecondsRealTime=0;
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar_2,0);
relayState_2=0; 
}

//-> upo mode
if (digitalRead(AC_Available)==1 &&  RunLoadsByBass==0 && UPSMode==1 && LoadsAlreadySwitchedOFF==1)
{
LoadsAlreadySwitchedOFF=0;
SecondsRealTime=0;
SecondsRealTimePv_ReConnect_T1=0;
SecondsRealTimePv_ReConnect_T2=0;
CountSecondsRealTime=0;
CountSecondsRealTimePv_ReConnect_T1=0;
CountSecondsRealTimePv_ReConnect_T2=0;
digitalWrite(Relay_L_Solar_2,0);
digitalWrite(Relay_L_Solar,0);
relayState_1=0; 
relayState_2=0; 
}
}

//----------------------------------------Timer for counting seconds-------------------------------
void Timer_Seconds()
{
noInterrupts();
TCCR1A = 0; // very important 
TCCR1B = 0; // very important 
OCR1A=15600;  // 1 second 
TCCR1B |= (1<< CS10) | (1<<CS12) | (1<WGM12); // 1024 prescalar 
TIMSK1 |= (1 << OCIE1A) ;  // enabling interrupts 
interrupts();
}
//--------------------------------------Timer Interrupt----------------------------------------
ISR(TIMER1_COMPA_vect) 
{
   TCNT1=0;   // very important 

  UpdateScreenTime++;
  
  if (CountSecondsRealTime==1) SecondsRealTime++;                                     // for counting real time for  grid count
  if (CountSecondsRealTimePv_ReConnect_T1==1) SecondsRealTimePv_ReConnect_T1++; // for counting real time for pv connect
  if(CountSecondsRealTimePv_ReConnect_T2==1) SecondsRealTimePv_ReConnect_T2++; // for counting real timer 
  if(CountCutSecondsRealTime_T1==1) CutSecondsRealTime_T1++; 
  if(CountCutSecondsRealTime_T2==1) CutSecondsRealTime_T2++; 
if (UpdateScreenTime==60 && programNumber==0 )  // 1800 is 60 seconds to update
{
  UpdateScreenTime=0;
  digitalWrite(Backlight,0);
  lcd.begin(16,2);
  lcd.clear();
  lcd.noCursor();
  lcd.setCursor(0,0); 
 // lcd.noDisplay();
}
//------------------------------------------------------------------------------------------
TurnLoadsOffWhenGridOff(); // just to check that what matter happens the loads will switch off even if mcu got stuck 
delay(100);  


 }

 //-------------------------------------Wire Timeout----------------------------------------------
 /*
 ref: 
 - https://myhomethings.eu/en/arduino-and-the-i2c-twi-protocol/
 - https://www.fpaynter.com/2020/07/i2c-hangup-bug-cured-miracle-of-miracles-film-at-11/
 - https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/?_gl=1*115qfm6*_ga*MTAzNjA3ODQuMTY4Nzg0NzA2Mg..*_ga_NEXN8H46L5*MTY5NDMyMzk0OC42OC4xLjE2OTQzMjY3NDEuMC4wLjA.
 */
void CheckWireTimeout()
{
 if (Wire.getWireTimeoutFlag())
	{
		Wire.clearWireTimeoutFlag();   // this flag is cleared manually or cleared when  setWireTimeout() is called 
	}
}



//---------------------------------------Check For Params-----------------------------------------
void CheckForParams()
{
//----------------Timer 1 ----------------
if (hours_lcd_1< 0  || hours_lcd_1 > 23)
{
hours_lcd_1=8; 
EEPROM.write(0,hours_lcd_1);  
EEPROM_Load();
}  
if (minutes_lcd_1< 0  || minutes_lcd_1 > 59)
{
minutes_lcd_1=0; 
EEPROM.write(1,minutes_lcd_1);  
EEPROM_Load();
} 
if (hours_lcd_2< 0  || hours_lcd_2 > 23)
{
hours_lcd_2=17; 
EEPROM.write(2,hours_lcd_2);  
EEPROM_Load();
}  
if (minutes_lcd_2< 0  || minutes_lcd_2 > 59)
{
minutes_lcd_2=0; 
EEPROM.write(3,minutes_lcd_2);  
EEPROM_Load();
} 
//----------------Timer 2 ------------------------
if (hours_lcd_timer2_start< 0  || hours_lcd_timer2_start > 23)
{
hours_lcd_timer2_start=9; 
EEPROM.write(4,hours_lcd_timer2_start);  
EEPROM_Load();
}  
if (minutes_lcd_timer2_start< 0  || minutes_lcd_timer2_start > 59)
{
minutes_lcd_timer2_start=0; 
EEPROM.write(5,minutes_lcd_timer2_start);  
EEPROM_Load();
} 
if (hours_lcd_timer2_stop< 0  || hours_lcd_timer2_stop > 23)
{
hours_lcd_timer2_stop=17; 
EEPROM.write(6,hours_lcd_timer2_stop);  
EEPROM_Load();
}  
if (minutes_lcd_timer2_stop< 0  || minutes_lcd_timer2_stop > 59)
{
minutes_lcd_timer2_stop=0; 
EEPROM.write(7,minutes_lcd_timer2_stop);  
EEPROM_Load();
}
//---------------------------LOW Voltage------------------------------
if (Mini_Battery_Voltage< 0  || Mini_Battery_Voltage > 65.0 || isnan(Mini_Battery_Voltage))
{
if (SystemBatteryMode==12) Mini_Battery_Voltage=12.0; 
if (SystemBatteryMode==24) Mini_Battery_Voltage=24.5; 
if (SystemBatteryMode==48) Mini_Battery_Voltage=49.0; 
EEPROM.put(8,Mini_Battery_Voltage);
EEPROM_Load();
}
if (Mini_Battery_Voltage_T2< 0  || Mini_Battery_Voltage_T2 > 65.0 || isnan(Mini_Battery_Voltage_T2))
{
if (SystemBatteryMode==12) Mini_Battery_Voltage_T2=12.3; 
if (SystemBatteryMode==24) Mini_Battery_Voltage_T2=25.0; 
if (SystemBatteryMode==48) Mini_Battery_Voltage_T2=50.0; 
EEPROM.put(12,Mini_Battery_Voltage_T2);
EEPROM_Load();
}
//--------------------------Start Loads Voltage------------------------
if (StartLoadsVoltage< 0  || StartLoadsVoltage > 65.0 || isnan(StartLoadsVoltage) )
{
if (SystemBatteryMode==12) StartLoadsVoltage=13.0; 
if (SystemBatteryMode==24) StartLoadsVoltage=25.5; 
if (SystemBatteryMode==48) StartLoadsVoltage=51.0; 
EEPROM.put(16,StartLoadsVoltage);
EEPROM_Load();
}
if (StartLoadsVoltage_T2< 0  || StartLoadsVoltage_T2 > 65.0 || isnan(StartLoadsVoltage_T2))
{
if (SystemBatteryMode==12) StartLoadsVoltage_T2=13.2; 
if (SystemBatteryMode==24) StartLoadsVoltage_T2=26.0; 
if (SystemBatteryMode==48) StartLoadsVoltage_T2=52.0; 
EEPROM.put(20,StartLoadsVoltage_T2);
EEPROM_Load();
}
//-------------------------Startup Timers--------------------------------
if (startupTIme_1< 0  || startupTIme_1 > 900)
{
startupTIme_1=90; 
EEPROM.put(24,startupTIme_1);
EEPROM_Load();
}
if (startupTIme_2< 0  || startupTIme_2 > 900)
{
startupTIme_2=120; 
EEPROM.put(26,startupTIme_2);
EEPROM_Load();
}

//-------------------------Startup Timers--------------------------------
if (offDelay_1< 0  || offDelay_1 > 240)
{
offDelay_1=15; 
EEPROM.put(35,offDelay_1);
EEPROM_Load();
}
if (offDelay_2< 0  || offDelay_2 > 240)
{
offDelay_2=25; 
EEPROM.put(37,offDelay_2);
EEPROM_Load();
}
//----------------------Run On Battery Voltage Mode-------------------------
if (RunOnBatteryVoltageMode < 0 || RunOnBatteryVoltageMode > 1 )
{
  RunOnBatteryVoltageMode=1;
  EEPROM.write(28,RunOnBatteryVoltageMode);
  EEPROM_Load();
}
//----------------------------UPS Mode------------------------------------
if (UPSMode < 0 || UPSMode > 1 )
{
  UPSMode=1;
  EEPROM.write(29,UPSMode);
  EEPROM_Load();
}



if (VinBatteryDifference<0 || VinBatteryDifference>=70 || isnan(VinBatteryDifference) )
{
  VinBatteryDifference=0;
  EEPROM.put(31,VinBatteryDifference);
  EEPROM_Load();
}

 if (addError<0 || addError>1 )
{
  addError=1;
  EEPROM.write(30,addError);     //add error is on so add error to vin battery
  EEPROM_Load();
}

}


//-----------------------------------------------------------

void TimerBattery()
{
noInterrupts();
TCCR2A=0; // very imprtant 
TCCR2B = 0; // very important 
OCR2A=100; // 10 ms 
TCCR2B |=  (1<< CS20) | (1 << CS21) | (1<<CS22) ;  // prescalar 1024
TIMSK2 |= (1 << OCIE2A) ;  // enabling interrupts overflow 
interrupts(); 
}

ISR(TIMER2_COMPA_vect)
{
TCNT2=0; // very important 
if(batteryTypeLiPo4==0)
{
samplesReading++;
ADC_Value=analogRead(A1);
Battery_Voltage=(ADC_Value *5.0)/1024.0;
Battery[samplesReading]=((10.5/0.5)*Battery_Voltage);
sum+=Battery[samplesReading];
if (samplesReading==100)
{
// Vin_Battery_=sum / 50.0;   // i can make it 20 samples but because the smd coil is making problem
// if (addError==1) Vin_Battery=Vin_Battery_+VinBatteryDifference;
// else if(addError==0)  Vin_Battery=Vin_Battery_-VinBatteryDifference;
// sum=0;
// samplesReading=0;
Vin_Battery=sum / 100.0;   // i can make it 20 samples but because the smd coil is making problem
sum=0;
samplesReading=0;
}
} // end if batterytype
if (batteryTypeLiPo4==1) Vin_Battery=formattedBatteryCapacity.toDouble(); 

}

//---------------------------------------WORKING MODE----------------------------------------------
void WorkingMode()
{
	if (digitalRead(AC_Available)==0)
	{
	digitalWrite(Flash,1);
	}
	else if(batteryTypeLiPo4 ==0 )
	{
	digitalWrite(Flash,1);
  delay(500);
  digitalWrite(Flash,0);
  delay(500);
  }
  
  else if(batteryTypeLiPo4==1 && bmsErrorFlag==0) // connection is success
  {
  digitalWrite(Flash,1);
  delay(100);
  digitalWrite(Flash,0);
  delay(100);
  }
  else {
    digitalWrite(Flash,0);
  }
  

}

void clearSerialBuffer() {
    bool dataLeft = true;
    int retries = 0;

    while (dataLeft && retries < 10) {  // Retry a limited number of times
        dataLeft = false;  // Assume no data
        while (Serial.available() > 0) {
            Serial.read();  // Read any available data
            dataLeft = true; // Data was actually present
        }
        retries++;
        delay(10);  // Short delay to allow more bytes to come in if there are any left
    }
}

void Read_LiPo4()
{

  
     
     pipSend(pipCommands.qpigs, sizeof(pipCommands.qpigs));
    // Read and print the incoming serial data from the inverter until <CR>
     receivedData = "";
     endOfResponse = false;  // Flag to indicate when end of response is found
     startTime_inverter = millis();  // Record the start time

    while (!endOfResponse) {
        if (Serial.available()>0) { // very important 
            char incomingByte = Serial.read();
            receivedData += incomingByte;

            // Check for carriage return '\r'
            if (incomingByte == '\r') {
                endOfResponse = true;  // Stop reading when <CR> is found
                bmsErrorFlag=0; // connection is success
                lcd.setCursor(7,0);
                lcd.print("   ");
                delay(200);
                
            }


        }

            if (millis() - startTime_inverter > timeout_inverter) {
            
            // lcd.setCursor(0,0);
            // lcd.print("timeout");
            bmsErrorFlag=1;
            break;  // Exit the loop if timeout is reached
            }
    }  // end while 

    // Check if the response matches the expected format
    if (isValidResponse(receivedData) ) 
    {
       

        // Extract and print battery voltage
        batteryVoltage = getValue(receivedData, ' ', 8);  // 9th item (index 8)
        batteryCapacity = getValue(receivedData, ' ', 10); // 11th item (index 10)
       // loadKW= getValue(receivedData, ' ', 6).toFloat();  // new way
        loadKW= getValue(receivedData, ' ', 6);  
        batteryDischargeCurrent = getValue(receivedData, ' ', 15); // 16th item (index 16

        


    } 
    else 
    {    
            // getting data but reponse is not acceptable 

            bmsErrorFlag=1;
           // Serial.println("Response is NOT accepted.");
            batteryCapacity="0";
            batteryVoltage="0";
            batteryDischargeCurrent="0";
            // lcd.setCursor(0,0);
            // lcd.print("Error ");  
            // delay(500);
            Serial.end(); 
            delay(500);
            Serial.begin(2400);
    }  // end else 

    //-> display lcd 
     if (batteryCapacity == "100") 
       {
      formattedBatteryCapacity = "100"; // Keep "100" as is for 100%
       }
      else if(batteryCapacity=="0")
      {
        formattedBatteryCapacity="0";
      
      }
 
       else 
       {
        formattedBatteryCapacity = batteryCapacity.substring(1); // Remove leading zero for values < 100%
       }

        formattedBatteryVoltage = batteryVoltage.substring(0, 4); // Keep only "24.3
        
       // sprintf(txt,"%sV      %s%%      ",formattedBatteryVoltage.c_str(),formattedBatteryCapacity.c_str());
       // sprintf(txt,"%sV  %s%%  %dA      ",formattedBatteryVoltage.c_str(),formattedBatteryCapacity.c_str(),batteryDischargeCurrent.toInt());
        sprintf(txt,"%sV  %s%%  %dA      ",formattedBatteryVoltage.c_str(),formattedBatteryCapacity.c_str(),batteryDischargeCurrent.toInt());
        lcd.setCursor(0,1);
        lcd.print(txt);

}
//-------------------------------------------------------------------------------------------
// Function to validate the received response format
bool isValidResponse(const String& data) {
    // Ensure the data is long enough to have all expected parts
    if (data.length() < 80) {  // 40 default worked value 
        return false;
    }
  
  // response :                          (BBB.B CC.C DDD.D EE.E FFFF GGGG HHH III JJ.JJ KKK OOO TTTT EE.E UUU.U WW.WW PPPPP b7b6b5b4b3b2b1b0 QQ VV MMMMM b10b9b8 Y ZZ AAAA BB.B<CRC><cr> 
  // resonse from AXPERT MAX E TWIN 11K :(000.0 00.0 229.9 50.0 01542 01229 014 457 55.70 000 100 0041 03.3 378.6 00.00 00000 00010000 00 00 01273 010 0 00 0000u 
    // Remove the starting '(' and ending character for processing
    String content = data.substring(1, data.length() - 1);
    int startIndex = 0;
    int spaceIndex;

  //  // Validate each expected part of the response
  //   String expectedFormats[] = {
  //       "000.0", // BBB.B  
  //       "00.0",  // CC.C
  //       "000.0", // DDD.D
  //       "00.0",  // EE.E
  //       "0000",  // FFFF
  //       "0000",  // GGGG
  //       "000",   // HHH   load in kw
  //       "339",   // III
  //       "25.60", // JJ.JJ battery voltage
  //       "000",   // KKK
  //       "100",   // OOO   battery capacity 
  //       "0030"   // TTTT
  //   };
  //-> this expected string with battery discharge current 
    // String expectedFormats[] = {
    // "000.0",  // BBB.B  
    // "00.0",   // CC.C
    // "000.0",  // DDD.D
    // "00.0",   // EE.E
    // "0000",   // FFFF
    // "0000",   // GGGG
    // "000",    // HHH   load in kw
    // "339",    // III
    // "25.60",  // JJ.JJ battery voltage
    // "000",    // KKK
    // "100",    // OOO   battery capacity 
    // "0030",   // TTTT
    // "00.0",   // EE.E   input pv1 
    // "000.0",  // UUU.U  input pv2 
    // "00.00",  // WW.WW  battery voltage from scc
    // "00000"   // PPPPP   battery discharge current 
    // };

    
    // String expectedFormats[] = {
    // "000.0",  // BBB.B  
    // "00.0",   // CC.C
    // "000.0",  // DDD.D
    // "00.0",   // EE.E
    // "00000",   // FFFFF
    // "00000",   // GGGGG
    // "000",    // HHH   load in kw
    // "339",    // III
    // "25.60",  // JJ.JJ battery voltage
    // "000",    // KKK
    // "100",    // OOO   battery capacity 
    // "0030",   // TTTT
    // "00.0",   // EE.E   input pv1 
    // "000.0",  // UUU.U  input pv2 
    // "00.00",  // WW.WW  battery voltage from scc
    // "00000"   // PPPPP   battery discharge current 
    // };
//->
//     String expectedFormatsMaxETwin[] = {
//     "000.0",
//      "00.0", 
//      "000.0",
//      "00.0", 
//      "00000",
//      "00000",
//      "000",
//      "000",
//      "00.00",
//      "000",
//      "000",
//      "0000",
//      "00.0",
//      "000.0",
//      "00.00",
//      "00000"
// };
  // Define multiple expected formats
    String expectedFormats[][16] = {


        { "000.0", "00.0", "000.0", "00.0", "00000", "00000", "000", "000", "00.00", "000", "000", "0000", "00.0", "000.0", "00.00", "00000" },  // VMII
       // { "BBB.B", "CC.C", "DDD.D", "EE.E", "FFFF", "GGGG", "HHH", "III", "JJ.JJ", "KKK", "OOO", "TTTT", "EE.E", "UUU.U", "WW.WW", "PPPPP" }     //MAX E TWIN 
    };


    // Loop through each format array and test all expected formats then test it and return only true if mataches any format 
    for (int f = 0; f < sizeof(expectedFormats) / sizeof(expectedFormats[0]); f++) {
        startIndex = 0;
        bool isFormatValid = true;

        for (int i = 0; i < 16; i++) {
            spaceIndex = content.indexOf(' ', startIndex);
            String segment = (spaceIndex == -1) ? content.substring(startIndex) : content.substring(startIndex, spaceIndex);

            if (!matchesFormat(segment, expectedFormats[f][i])) {
                isFormatValid = false;
                break;
            }
            startIndex = spaceIndex + 1;
        }

        // If one format matches, return true
        if (isFormatValid) {
            return true;
        }
    }

    // No formats matched
    return false;
}

// Function to check if a segment matches the expected format
bool matchesFormat(const String& value, const String& expected) {
    if (expected.length() != value.length()) return false;

    for (int i = 0; i < expected.length(); i++) {
        char expChar = expected.charAt(i);
        char valChar = value.charAt(i);

        if (expChar == '0') {
            if (!isDigit(valChar)) return false; // Must be a digit
        } else if (expChar == '.') {
            if (valChar != '.') return false; // Must be a dot
        }
    }
  
    return true; // Matches the expected format
}

// Helper function to split and get the Nth value from the response
String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex = 0; // Current index in the string
    int startIndex = 0; // Starting index for the current value

    // Loop through the data to find the index
    while (strIndex < data.length()) {
        // If we find the separator or reach the end of the string
        if (data.charAt(strIndex) == separator || strIndex == data.length() - 1) {
            found++;
            // If it's the last character, adjust the end index
            int endIndex = (strIndex == data.length() - 1) ? strIndex + 1 : strIndex;
            // Check if this is the index we want
            if (found - 1 == index) { // -1 because we're counting from 0
                // Create a substring and trim it
                String value = data.substring(startIndex, endIndex);
                value.trim(); // Trim to remove extra spaces
                return value; // Return the trimmed value
            }
            // Update the startIndex for the next value
            startIndex = strIndex + 1;
        }
        strIndex++;
    }

    return ""; // Return empty string if not found
}

// CRC-16/IBM Calculation
uint16_t crc16(const uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

void pipSend(unsigned char *cmd, int len) {
    uint16_t crc = crc16(cmd, len);  // Compute CRC

    // Send command to inverter
    for (int i = 0; i < len; i++) {
        Serial.write(cmd[i]);
      }

    // Send CRC and carriage return
    Serial.write(crc & 0xFF);  // CRC low byte
    Serial.write(crc >> 8);    // CRC high byte
    Serial.write(0x0d);        // Carriage return
}

//-------------------------------------------------------------------------------------------------


//---------------------------------------MAIN LOOP-------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Config();
  Config_Interrupts();
  EEPROM_Load();
  Timer_Seconds();
  }

void loop() {
  // put your main code here, to run repeatedly:
  //CheckForParams();
  CheckForSet(); // done 
  RunTimersNowCheck(); // done 
  WorkingMode();   // done 
  CheckForTimerActivationInRange();  // done
  CheckForTimerActivationOutRange();  // done
  Screen_1();  // done 
  Check_Timers();  // done
  TurnLoadsOffWhenGridOff();  // done
  CheckWireTimeout();        // done 
  CheckSystemBatteryMode();  // done
  delay(50);                 // done 
 }