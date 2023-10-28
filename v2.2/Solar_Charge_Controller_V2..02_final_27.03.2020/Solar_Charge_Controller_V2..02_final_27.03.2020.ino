

////////////////////////////////ARDUINO PWM SOLAR CHARGE CONTROLLER V-2.02///////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------

  
  //  This code is for an arduino based Solar PWM charge controller ( V-2.0)
  //  Credit :logic of 3 stage charging taken 
  //  Last updated on 24/03/2020
  //----------------------------------------------------------------------------------------------------
  // Arduino pins Connections----------------------------------------------------------------------------
  
    // A0 - Voltage divider to measure solar panel voltage
    // A1 - Voltage divider to measure battery voltage
    // A2 - ACS712 to monitor load current
    // A3 - ACS712 to monitor solar current
    // A4 - LCD SDA 
    // A5 - LCD SCL
  
    // D0- Not used
    // D1- Not used
    // D2 - Control Load MOSFET Q2 
    // D3 - PWM Output to control MOSFET Q1 
    // D4 - Not Used
    // D5 - Battery Red LED       
    // D6 - Battery Green LED 
    // D7 - Battery Blue LED 
    // D8 - Load Red Led 
    // D9 - Load Green Led  
    // D10- Solar Red LED
    // D11- Solar Green LED  
    // D12- DS18B20 Temp. Sensor
    // D13-not used  

#include <Wire.h>
#include <OneWire.h>  
#include <DallasTemperature.h> 
#include <LiquidCrystal_I2C.h>

#define SOL_ADC A0     // Solar panel side voltage divider is connected to pin A0 
#define BAT_ADC A1     // Battery side voltage divider is connected to pin A1
#define LOAD_CURRENT_ADC A2  // ACS 712 current sensor is connected to pin A2 for load curremt
#define SOL_CURRENT_ADC A3  // ACS 712 current sensor is connected to pin A3 for solar current
#define AVG_NUM 10    // number of iterations of the adc routine to average the adc readings
#define BAT_MIN 10.5  // minimum battery voltage for 12V system
#define BAT_MAX 15.0  // maximum battery voltage for 12V system
#define BULK_CH_SP 14.4 // bulk charge set point for sealed lead acid battery // flooded type set it to 14.6V
#define FLOAT_CH_SP 13.6  //float charge set point for lead acid battery
#define LVD 11.5          //Low voltage disconnect setting for a 12V system
#define PWM_PIN 3         // pin-3 is used to control the charging MOSFET //the default frequency is 490.20Hz
#define LOAD_PIN 2       // pin-2 is used to control the load
#define BAT_RED_LED 5
#define BAT_GREEN_LED 6
#define BAT_BLUE_LED 7
#define LOAD_RED_LED 8
#define LOAD_GREEN_LED 9
#define SOL_RED_LED 10
#define SOL_GREEN_LED 11
#define ONE_WIRE_BUS 12 // Data wire of DS18B20 temp. sensor is connected to pin 12  

//--------------------------------------------------------------------------------------------------------------------------
///////////////////////DECLARATION OF ALL BIT MAP ARRAY FOR FONTS////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------

byte solar[8] = //icon for solar panel
{
  0b11111,0b10101,0b11111,0b10101,0b11111,0b10101,0b11111,0b00000
};
byte battery[8] =  //icon for battery
{
  0b01110,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b11111
};

byte energy[8] =  // icon for power
{
  0b00010,0b00100,0b01000,0b11111,0b00010,0b00100,0b01000,0b00000
};
/*byte alarm[8] =  // icon for alarm
{
 0b00000,0b00100,0b01110,0b01110,0b01110,0b11111,0b00000,0b00100
};*/
byte temp[8] = //icon for termometer
{
 0b00100,0b01010,0b01010,0b01110,0b01110,0b11111,0b11111,0b01110
};

byte charge[8] = // icon for battery charge
{
  0b01010,0b11111,0b10001,0b10001,0b10001,0b01110,0b00100,0b00100,
};
byte not_charge[8]=
{
  0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b00000,
};


//--------------------------------------------------------------------------------------------------------------------------
///////////////////////DECLARATION OF ALL GLOBAL VARIABLES//////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------
float solar_volt=0;
float bat_volt=0;
float load_current=0;
float solar_current=0;
float offsetVoltage = 2.5; // for ACS712 sensor
float Sensitivity =0.185; // 185mV/A for ACS712-5A variant
int temperature=0;
int temp_change=0;
float system_volt=0;
float bulk_charge_sp=0;
float float_charge_sp=0;
int charge_stage =0;
float load_status=0;
double PWM_Duty =0; // PWM_Duty varies from 0 to 255
float lvd;

// Variables for power and energy measurements

long unsigned time =0; 
long unsigned msec=0;
long unsigned last_msec=0;
long unsigned elasped_msec=0;
long unsigned elasped_time=0;
float load_ampSecs = 0;
float load_ampHours=0;
float load_watts=0;
float load_wattSecs = 0;
float load_wattHours=0;
float solar_ampSecs = 0;
float solar_ampHours=0;
float solar_watts=0;
float solar_wattSecs = 0;
float solar_wattHours=0;

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 


// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27,20,4);  // Set the LCD I2C address // In my case 0x27
//******************************************************* MAIN PROGRAM START ************************************************
void setup()
{

Serial.begin(9600);
sensors.begin(); // // Start up the library  
TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 PWM frequency of 980.39 Hz

 
pinMode(SOL_RED_LED ,OUTPUT);
pinMode(SOL_GREEN_LED,OUTPUT);
pinMode(BAT_RED_LED,OUTPUT);
pinMode(BAT_GREEN_LED,OUTPUT);
pinMode(BAT_BLUE_LED,OUTPUT);
pinMode(LOAD_RED_LED ,OUTPUT);
pinMode(LOAD_GREEN_LED,OUTPUT);
pinMode(PWM_PIN,OUTPUT);
pinMode(LOAD_PIN,OUTPUT);
digitalWrite(PWM_PIN,LOW);  // default value of pwm duty cycle
digitalWrite(LOAD_PIN,LOW);  // default load state is OFF
lcd.init();   // initialize the lcd for 16 chars 2 lines, turn on backlight
lcd.backlight(); // finish with backlight on  
lcd.createChar(1,solar);
lcd.createChar(2, battery);
lcd.createChar(3, energy);
//lcd.createChar(4,alarm);
lcd.createChar(5,temp);
lcd.createChar(6,charge);
lcd.createChar(7,not_charge);
lcd.clear();
}

void loop()
{
 read_data();             // read different sensors data from analog pin of arduino
 system_voltage();        // detect the system voltage according to battery voltage
 setpoint();      // decide the charge set point according to system voltage
 charge_cycle();         // pwm charging of battery
 power();                // calculate the load power and energy
 load_control();         //control the load
 led_indication();       // led indica
 print_data();            // print in serial monitor
 lcd_display();           // lcd display
 }
//************************************************************ PROGRAM END *************************************************


//------------------------------------------------------------------------------------------------------
////////////////// READS AND AVERAGES THE ANALOG INPUTS (SOLRAR VOLTAGE,BATTERY VOLTAGE)////////////////
//------------------------------------------------------------------------------------------------------
int read_adc(int adc_parameter)
{
  
  int sum = 0;
  int sample ;
  for (int i=0; i<AVG_NUM; i++) 
  {                                        // loop through reading raw adc values AVG_NUM number of times  
    sample = analogRead(adc_parameter);    // read the input pin  
    sum += sample;                        // store sum for averaging
    delayMicroseconds(50);              // pauses for 50 microseconds  
  }
  return(sum / AVG_NUM);                // divide sum by AVG_NUM to get average and return it
}
//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////READ THE DATA//////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
 void read_data(void) 
 {
    //5V = ADC value 1024 => 1 ADC value = (5/1024)Volt= 0.0048828Volt
    // Vout=Vin*R2/(R1+R2) => Vin = Vout*(R1+R2)/R2   R1=100 and R2=20
    
     solar_volt = read_adc(SOL_ADC)*0.00488*(120/20);
     bat_volt   = read_adc(BAT_ADC)*0.00488*(120/20);
     load_current = ((read_adc(LOAD_CURRENT_ADC)*0.00488 - offsetVoltage-2.5)/Sensitivity ); // 2.4V offset when no load is connected
     solar_current = ((read_adc(SOL_CURRENT_ADC)*0.00488 - offsetVoltage)/ Sensitivity ); 
      if (load_current <0)
     {
      load_current = 0;
     }
     if (solar_current <0)
     {
      solar_current = 0;
     }
    
    sensors.requestTemperatures();  // get temperature readings 
    temperature = sensors.getTempCByIndex(0) ;    // 0 refers to the first IC on the wire 
     
  }
  //------------------------------------------------------------------------------------------------------------
/////////////////////////////////POWER AND ENERGY CALCULATION //////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void power(void){
  
msec = millis();
elasped_msec = msec - last_msec; //Calculate how long has past since last call of this function
elasped_time = elasped_msec / 1000.0; // 1sec=1000 msec
load_watts = load_current * bat_volt; //Watts now
solar_watts = solar_current * solar_volt; //Watts now

load_ampSecs = (load_current*elasped_time); //AmpSecs since last measurement
solar_ampSecs = (solar_current*elasped_time); //AmpSecs since last measurement

load_wattSecs = load_ampSecs * bat_volt; //WattSecs since last measurement
solar_wattSecs = solar_ampSecs * solar_volt; //WattSecs since last measurement

load_ampHours = load_ampHours + load_ampSecs/3600; // 1 hour=3600sec //Total ampHours since program started
solar_ampHours = solar_ampHours + solar_ampSecs/3600; // 1 hour=3600sec //Total ampHours since program started

load_wattHours = load_wattHours + load_wattSecs/3600; // 1 hour=3600sec //Total wattHours since program started
solar_wattHours = solar_wattHours + solar_wattSecs/3600; // 1 hour=3600sec //Total wattHours since program started

last_msec = msec; //Store 'now' for next time


/*
  
last_time = current_time;
current_time = millis();
load_watts = load_current * bat_volt; //load Watts now
solar_watts = solar_current * solar_volt; //solar Watts now
load_wattHours = load_wattHours +  load_watts*(( current_time -last_time) /3600000.0) ; // calculating energy in Watt-Hour
solar_wattHours = solar_wattHours+  solar_watts*(( current_time -last_time) /3600000.0) ; // calculating energy in Watt-Hour
*/
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////PRINT DATA IN SERIAL MONITOR/////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
  void print_data(void) 
  {
    //delay(100);
    Serial.print("        Solar Voltage: ");  
    Serial.print(solar_volt);
    Serial.print("        Battery Voltage: "); 
    Serial.print(bat_volt);
    Serial.print("Temp: " );
    Serial.print (temperature);
    Serial.print("        Bulk Voltage Setpoint: "); 
    Serial.print(bulk_charge_sp);
    Serial.print("        Float Voltage Setpoint: ");
    Serial.print(float_charge_sp);  
    Serial.print("        PWM Duty: ");
    Serial.print((PWM_Duty /255 )*100);  // convert a number between 0 -1024  to  0-1024 then convert float to intiger 
    Serial.print("%");
    Serial.print("        Charge Stage: ");
    Serial.println(charge_stage);
    Serial.print("Solar Current: ");
    Serial.print(solar_current);
    Serial.println("A");
  }
//----------------------------------------------------------------------------------------------------------------------
//////////////////////////////////SYSTEM VOLTAGE AUTO DETECT ///////////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------------
void system_voltage(void)
{
  if ((bat_volt >BAT_MIN) && (bat_volt < BAT_MAX))
  {
     system_volt = 12;
  }
  /*
  else if  ((bat_volt > BAT_MIN*2 ) && (bat_volt < BAT_MAX*2))
  {
    system_volt=24;
  }*/
  else if ((bat_volt > BAT_MIN/2 ) && (bat_volt < BAT_MAX/2))
  {
    system_volt=6;
  }
  
}
//---------------------------------------------------------------------------------------------------------------------------
 ////////////////////////////////////CHARGE SET POINT ///////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------------------------------------------------------------
 
void setpoint(void)
{
  temp_change =temperature-25.0; // 25deg cel is taken as standard room temperature 
 // temperature compensation = -5mv/degC/Cell 
  // If temperature is above the room temp ;Charge set point should reduced
  // If temperature is bellow the room temp ;Charge set point should increased
  if(system_volt ==12)
  {
     bulk_charge_sp = BULK_CH_SP-(0.050*temp_change) ;
     float_charge_sp=FLOAT_CH_SP-(0.050*temp_change) ;
     lvd =LVD;
  }
 
  else if(system_volt ==6)
  {
     bulk_charge_sp = (BULK_CH_SP/2)-(0.015*temp_change) ;
     float_charge_sp= (FLOAT_CH_SP/2)-(0.015*temp_change) ;
     lvd=LVD/2;
  }
  /*
  else if (system_volt == 24)
  {
   bulk_charge_sp = (BULK_CH_SP*2)-(0.060*temp_change) ;
   float_charge_sp= (FLOAT_CH_SP*2)-(0.060*temp_change) ;
   lvd=LVD*2;
  }
  */
  
}
//--------------------------------------------------------------------------------------------------------------------------------
 ///////////////////////////////////////////////////PWM CHARGE CYCLE @500 HZ //////////////////////////////////////////////////
 //-------------------------------------------------------------------------------------------------------------------------------
void charge_cycle(void)
{
  if(solar_volt <= 10){ 
    read_data();         // Reading voltage temp from the sensors
    setpoint();              // Reading temp compensated charging set point
    PWM_Duty = 0;
    analogWrite(PWM_PIN,PWM_Duty);  //generate PWM from D9 @ 0% duty // Shut down the charger
    
    }
 

//--------------------------------------------------------------------------------------------------------------------------------
 ///////////////////////////////////////////////////PWM CHARGE CYCLE @500 HZ //////////////////////////////////////////////////
 //-------------------------------------------------------------------------------------------------------------------------------

///////////////////////////////////  STAGE-1 (BULK CHARGING)//////////////////////////////////////////////////////
    // During this stage the MOSFET is fully on by setting the duty cycle to 100%
    // Constant Current Charging
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    
     if((bat_volt < bulk_charge_sp) && (solar_volt> (bat_volt + .5)) && charge_stage == 1){
    read_data();        // Reading voltage and temp from the sensors
    setpoint();               // Reading temp compensated charging set point
    PWM_Duty = 252.45;
    analogWrite(PWM_PIN,PWM_Duty); // 99 % duty cycle // rapid charging   
   }
    charge_stage = 2;
    

  ///////////////////////////////////  STAGE-2 (TOPPING CHARGE)//////////////////////////////////////////////////////
  // During this stage the MOSFET is fully on by setting the duty cycle to 100%
  // Constant voltage
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
   // 25.5 is 10 % duty cycle
    if((PWM_Duty> 25.5 ) && (solar_volt > (bulk_charge_sp + .5)) && charge_stage == 2){
    time = millis();  
     read_data();         // Reading voltage and temp from the sensors
     setpoint();              // Reading temp compensated charging set point
    
    if(bat_volt >= bulk_charge_sp){ // if the battery voltage rise above the bulk charge setpoint 
    PWM_Duty--;                     // Slow down the charging rate by reducing the pwm duty cycle
    if(PWM_Duty < 0){               // if during slow down process duty cycle falls below 0% then set to 0%
    PWM_Duty=0;
    }
     analogWrite(PWM_PIN,PWM_Duty);    //generate PWM from D3 @ 0% duty // MOSFET Q1 is OFF
    }
    if(bat_volt < bulk_charge_sp){ // if the battery voltage falls below the bulk charge setpoint, then go to stage-1
    charge_stage = 1;
    }  
    else{
      analogWrite(PWM_PIN,PWM_Duty);  
    }
 
  /* while( millis()-time > 3600000){ // Ensure the charger stay 1hour in bulk charging 
    charge_stage = 3;
   }
   */
    
    }
   charge_stage = 3; 
   

 ///////////////////////////////////  STAGE-3 (FLOAT CHARGING)//////////////////////////////////////////////////////
 // During this stage the MOSFET is fully on by setting the duty cycle to 100%
 // Constant Current Charging
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    if((solar_volt > ( float_charge_sp + .5)) && charge_stage == 3){
    read_data();         // Reading voltage and temp from the sensors
    setpoint();      // Reading temp compensated charging set point
    if(bat_volt >= float_charge_sp){ // if the battery voltage is more than float charge setpoint then shut down the charger
    PWM_Duty = 0;                    // setting duty cycle =0 to turn off the MOSFET Q1
    analogWrite(PWM_PIN,PWM_Duty);  //generate PWM from D9 @ 0% duty // MOSFET Q1 is OFF
    }
    else if(bat_volt <  float_charge_sp - 0.5){ // if the battery voltage falls below the float charge setpoint then go to stage -1
    charge_stage = 1;
    }
    else{
    PWM_Duty = 252.25; // setting duty cycle = 5% for trickle charge
    
    analogWrite(PWM_PIN,PWM_Duty);  //generate PWM from D9 @ 5% duty // Q1 is driving @ 5% duty cycle
   
    } 
    
   }
  }
//----------------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////LOAD CONTROL/////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------------  
  
void load_control()
{  
 if (solar_volt < 5  ) // load will on when night
{ 
  if(bat_volt >lvd)   // check if battery is healthy 
  {
  load_status=1;
  digitalWrite(LOAD_PIN, HIGH); // load is ON
  }
  else if(bat_volt < lvd)
  {
    load_status=0;
   digitalWrite(LOAD_PIN, LOW); //load is OFF
  }
 }
 else // load will off during day
 {
   load_status=0;
   digitalWrite(LOAD_PIN, LOW);
 }
}


//-------------------------------------------------------------------------------------------------
//////////////////////////LED INDICATION////////////////////////////////////
//-------------------------------------------------------------------------------------------------
void led_indication(void)
{
  solar_led();              //Solar led indication
  battery_led();           //Battery status led indication
  load_led();              //Load led indication
}
//----------------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////SOLAR LED INDICATION/////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------------  
  
  void solar_led()
  
  {
    if (solar_volt > float_charge_sp)
  {  
      digitalWrite(SOL_RED_LED,LOW);
      digitalWrite(SOL_GREEN_LED,HIGH);  // Sun Light is available and charger is ready for charging
  } 
  else 
  {
      digitalWrite(SOL_GREEN_LED,LOW);
      digitalWrite(SOL_RED_LED,HIGH);  //Sun light is not available for charging
      
  }  
     }
//----------------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////BATTERY LED INDICATION/////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------------
void battery_led(void)
{
  
   if( (bat_volt > system_volt) && ( bat_volt <bulk_charge_sp))
  {   
      leds_off_all();
      digitalWrite(BAT_GREEN_LED,HIGH);  // battery voltage is healthy
  } 
  else if(bat_volt >= bulk_charge_sp) 
  {
      leds_off_all();
      digitalWrite(BAT_BLUE_LED,HIGH);  //battery is fully charged
  }
   else if(bat_volt < system_volt)
  {
      leds_off_all();
      digitalWrite(BAT_RED_LED,HIGH);  // battery voltage low
  }
}
//----------------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////LOAD LED INDICATION/////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------------  
  
  void load_led()

  {
    if(load_status==1)
    {
      digitalWrite(LOAD_RED_LED, LOW);
      digitalWrite(LOAD_GREEN_LED,HIGH);
    }
    else if(load_status==0)
    {
      digitalWrite(LOAD_GREEN_LED, LOW);
      digitalWrite(LOAD_RED_LED,HIGH);
    }
   }

//------------------------------------------------------------------------------------------------------
//////////////////////// TURN OFF ALL THE LED///////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------
void leds_off_all(void)

{ 
  
  digitalWrite(BAT_RED_LED,LOW);
  digitalWrite(BAT_GREEN_LED,LOW);
  digitalWrite(BAT_BLUE_LED,LOW);
  
  
}
//------------------------------------------------------------------------------------------------------
//////////////////////// LCD DISPLAY///////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------
void lcd_display()
{
 // Display Solar Panel Parameters
 lcd.setCursor(0, 0);
 lcd.write(1);
 lcd.setCursor(2, 0);
 lcd.print(solar_volt,1);
 lcd.print("V");
 lcd.setCursor(8, 0);
 lcd.print(solar_current,1);
 lcd.print("A");
 lcd.setCursor(14,0);
 lcd.print(solar_watts,1);
 lcd.print("W");

 // Display Battery Parameters
 lcd.setCursor(0,1);
 lcd.write(2);
 lcd.setCursor(2, 1);
 lcd.print(bat_volt,1);
 lcd.print("V");
 lcd.setCursor(8, 1);
 lcd.write(5);
 lcd.setCursor(9, 1);
 lcd.print(temperature);
 lcd.write(0b11011111);
 lcd.print("C");

 lcd.setCursor(14, 1);
 lcd.write(2);
 if((charge_stage==1) | (charge_stage== 2) | (charge_stage== 3))
 {
 lcd.write(6);
 }
 else
 {
 lcd.write(7);
 }

 // Display Load Parameters

 lcd.setCursor(0,2);
 lcd.print("L");
 lcd.setCursor(2,2);
 lcd.print(load_current,1);
 lcd.print("A");
 lcd.setCursor(8,2);
 lcd.print(load_watts,1); 
 lcd.print("W");
 lcd.setCursor(14,2);
 if(load_status==1)
    {
      lcd.print("   "); 
      lcd.setCursor(14,2);
      lcd.print("ON"); 
    }
    else if(load_status==0)
    {
      lcd.print("OFF"); 
    }

 // Display Energy 
 lcd.setCursor(0,3);
 lcd.write(3);
 lcd.setCursor(2,3);
 lcd.print(solar_wattHours,1);
 lcd.print("Wh");  
 lcd.setCursor(8,3);
 lcd.print(load_wattHours,1);
 lcd.print("Wh");
  
}
