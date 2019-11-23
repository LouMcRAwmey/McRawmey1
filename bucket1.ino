

#include <Nokia_LCD.h>

#include <SPI.h>

#include <OneWire.h>

#include <EEPROM.h>

#include <Adafruit_GFX.h>

#include <Adafruit_SPITFT.h>

#include <Adafruit_SPITFT_Macros.h>

#include <gfxfont.h>

#include <Wire.h>

#include <Adafruit_BME280.h>

#include <Adafruit_Sensor.h>

#include <DallasTemperature.h>

#include <math.h>


#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 28
#define BME_MISO 27
#define BME_MOSI 2
#define BME_CS 3

//Bosch Bme280 Humidity/température
//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI



//#define TEST

//sonde D18B20
#define ONE_WIRE_BUS 26
const int TempProbePositive =22;  //Temp Probe power connected to pin 6
const int TempProbeNegative=24;    //Temp Probe Negative connected to pin 5
float Temperature=0.0;
float MinT=100.0;
float MaxT=0.0;

//pH meter Analog output to Arduino Analog Input 0
int PHPin =A15;
////The calibration constant for the PH probe
//float K_PH=3.64;       
//used for min/max logs
//float MinPH=10;
//float MaxPH=0;


//*********************** User defined variables ****************************//

//The Number of Days before the LCD will tell you its time to Calibrate

int Cal_Period_Warning=30;

//I got this from shorting the nbc's cnetre pin to outer pin [simulated the perfect probe] open serial and its the mv reading that you want to put here

float Healthy1_mv=1.96;    

//************** Some values for working out the ph*****************//

float mvReading=0.0;

float Vs=5.0;

float phValue=0.0;

int i=0;

long reading=0;

unsigned long sum=0;

float average=0.0;

//used for min/max logs

float MinPH=10.0;

float MaxPH=0.0;

//************** Variables used to determin probe health **********//

//there are three ways we can measure a proble life during calibration:

//1:Asymmetry potential (Eo), the Millivoltes at pH7 [mV reading in pH 7.00 buffer ± 25 mV]

//2:Slope, mv per Ph change [mV reading in pH 7.00 buffer - mV reading in pH 4.00 buffer 160 – 180 mV]

//3:Drift mV reading in pH 4.00 buffer (1 min) – mV reading in pH 4.00 buffer (2 min) ± 1.5 mV  [we are checking for stable readings]

int ProbeLife1=0;

float mvReading_7=0.0;

//Some major brands will have this set at 35mv, we have a gain of around 8 in our op amp

float Health1_range =0.28;

float Healthy1_mv2=1.96;

//I got these from a healthy probe and extrapolating the slope deviation

int ProbeLife2=0;

float Slope=0.0;

float mvReading_4=0;

float Healthy2_Slope=2.15;

float Healthy2_range=0.25;

int ProbeLife3=0;

float Drift=0.0;

float mvReading_4_Delayed=0.0;

float Healthy3_Drift=0.0;

float Healthy3_range=0.02;

//some Variables for Probe Calibration Time

int Last_Day=0;

int Days_Since_Calibration=0;

int CalibrationWarning=0;

float offset=0.0;

//Nokia Lcd 5110
//Adafruit_PCD8544 display = Adafruit_PCD8544(30, 32, 8, 34, 7);
Nokia_LCD lcd (52 /* CLK */, 51  /* DIN */ , 8 /* DC */, 7 /* CE */, 6 /* RST */);
//(52 /* CLK */, 51  /* DIN *
//Arduino Mega SPI pins:  51 (MOSI)DIN, 52 (SCK),
// pin 8 - Data/Command select (D/C)
// pin 7 - LCD chip select (CS)
// pin 6 - LCD reset (RST)

boolean backlight = true;
int contrast=50;
#define XPOS 0
#define YPOS 1

// define some values used by the panel and buttons
//int display_key     = 0;

//int adc_key_in  = 0;
//

//buttons

const int buttonMENU = 20;

volatile int presses = 0;
volatile bool pressed = false;
volatile unsigned long pushTime = 0;
//
const int buttonRIGHT =37;
const int buttonUP =39;
const int buttonDOWN =41;
const int buttonLEFT =43;
const int buttonSELECT =45;
//const int buttonNONE =47;

int button =0;


//volatile boolean up = false; 
//volatile boolean down = false; 
//volatile boolean middle = false; 

int buttonUPState = 0; 
int buttonSELECTState= 0;  
int lastbuttonDOWNState = 0; 
int lastbuttonSELECTState = 0; 
int lastbuttonUPState = 0;
int lastbuttonRIGHTState = 0;
int lastbuttonLEFTState = 0;
int buttonLEFTState = 0;
int buttonRIGHTState = 0;
int buttonDOWNState = 0;
int buttonState = 0;

//#define buttonRIGHT 1

//#define buttonUP 3

//#define buttonDOWN 2

//#define buttonLEFT 4

//#define buttonSELECT 5

//#define button NONE 6

//#define buttonMENU   7

int Screen =1;

//Max number of screens on lcd

const int Last_Screen_no =12;

//used to debounce input button

int buttonLast=0;
//************************** Just Some basic Definitions used for the Up Time LOgger ************//

long Day=0;

int Hour =0;

int Minute=0;

int Second=0;

int HighMillis=0;

int Rollover=0;

//Used For Calibration timing

unsigned long StartCalibration1=0;

//***************** Specifying where to sotre the calibration value [non volatile memory **//

int value=0; //we use this to check if memory has been writen or not

int addresCalibrationPH4=0;

int addresCalibrationPH7=50;

int addresProbleLife1=100;

int addresProbleLife2=150;

int addresProbleLife3=200;

int addresseCalibrationDays=250;

//fans
const int sinkPin = 12;
const int intakePin = 11;
const int caseFan = 10;
const int exhaustFan = 9;
int sinkP = 15;
int intakeP = 15;

const int sinkAim = 30;
const int sinkHys = 5;

const int thermistor = A0;
const int beta = 3950;
float T=thermistor;
int h=bme.readHumidity();
int t=bme.readTemperature();

int pressure;
int alt;

//timing
unsigned long lastFanTime = 0;

//lighting
bool flower = false;
bool automatic = true;
bool status = 0;
const int lights = 25;
unsigned long lastLightTime, lightsOffPeriod;

////RGB LED
//char color;
//const int rgb[4] = {13, 4, 39, 41};

//EC METRE
float TankSize =20.0; //SystemVolume in Liters

float PumpRate=1.5; //Flow rate of dosing pump in L per min

long DosingInterval =60; //How often you want to dose the tank in minutes, hour intervals will be fine for most systems

long MixingTime =30; //The time for mixing of nutrients in minutes [I have a lot of water flow so 60 seonds is fine for me, if in doubt increase it to half a hour]



int Pump =14; //Pin Controlling relay for nutrient Pump

int OFF =0;

int ON=1;

float ECSetpoint =1.0; // How Strong you want the water system to be nutrient wise to begin with

float MinECSetpoint =0.5;

float MaxECSetpoint =2.5;


//the code will optimise this number after every dosing, so we are selecting the highest end and allowing the arduino to work the rest out its self

//You are using a nutrient that states the NPK and not a overpriced one that used the words "Boost" "flower" "Super" but hides the important NPK ratios?

//If not Save yourself a fortune and switch to a cheap one that is honest about its NPK

//We need to put in a estimated start point for the controller to start with

//For an estimate take the EC Estimate= (N*20) + (P*8.8) +(K*16)


float ECFluidEstimate =100.0; //if you dont know the value stick with 1000, it will sort itself out



float CalibrationEC=1.380; 

int R1= 1000; //The resitor we placed in voltage divider
int Ra=25; //Resistance of Arduino Digital Pins
int ECPin= A8;
int ECGround=A9;
int ECPower =A12;
float TemperatureCoef = 0.019;

OneWire oneWire(ONE_WIRE_BUS);// Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);// Pass our oneWire reference to Dallas Temperature.

float TemperatureFinish=0.0;
float TemperatureStart=0.0;
float EC=0.0;
int ppm =0;
//int i=0;
float buffer=0.0; 

float PPMconversion=0.5;
float EC25 =0.0;
float raw= 0.0;
float Vin= 5.0;
float Vdrop= 0.0;
float Rc= 0.0;
float Kt=0.0;

float K=2.88;
//float TemperatureCoef = 0.02;
float PumpTime=0; //Variable for pumping duration

long MixingInterval=0; //variable for when to check new EC
float PostDocingEC=0.0;
float PreDosingEC =0.0;
long StartDosingMillis =0;

int NutesAdded=0;

long time=0;

float MinEC=100.0;
float MaxEC=0.0;

float LastGoodEstimate= 0.0;

float PumECRateStart=0.0;
float PumECRate=0.0;
int error=6; // 6 means no error
long Doses=0;
long LastRun=0;
int ECHold=0;

//int value;
int addresSetpoint=10;
int addresCalibration=0;


void setup() {


  Serial.begin(9600);
  
  Read_Eprom();
  Slope_calc();
  Splash_Screen();
  
  
  
  pinMode(TempProbeNegative , OUTPUT ); //seting ground pin as output for tmp probe
  digitalWrite(TempProbeNegative , LOW );//Seting it to ground so it can sink current
  pinMode(TempProbePositive , OUTPUT );//ditto but for positive
  digitalWrite(TempProbePositive , HIGH );
  pinMode(ECPin,INPUT);
  pinMode(ECPower,OUTPUT);//Setting pin for sourcing current
  pinMode(ECGround,OUTPUT);//setting pin for sinking current
  pinMode(Pump,OUTPUT);//setting pin for sinking current
  digitalWrite(Pump,OFF); //makes sure we are starting from a stop
  digitalWrite(ECGround,LOW);//We can leave the ground connected permanantlyread_Temp();// getting rid of the first bad reading
  
  
   
  if(Days_Since_Calibration>=Cal_Period_Warning) CalibrationWarning=1;


//Adjusting some values for use later in map function

Healthy1_mv2=Healthy1_mv*1000;
Health1_range =Health1_range*1000;

Healthy2_Slope=Healthy2_Slope*1000;
Healthy2_range=Healthy2_range*1000;

Healthy3_Drift=Healthy3_Drift*1000;
Healthy3_range=Healthy3_range*1000;

  delay(100);// gives sensor time to settle
  sensors.begin();
  
  //****************Converting to milli seconds********************//
DosingInterval =DosingInterval*60000; //Coveerting to milliseconds
MixingTime =MixingTime*60000;      // Converting to milliseconds
MixingInterval=DosingInterval+MixingTime;
PumpRate=PumpRate/60.0; //converting to L.Second-1


PumECRate=ECFluidEstimate*(PumpRate/TankSize);    //  EC.Second-1
PumECRate=PumECRate*2.0;//Just incase people overestimated some values
LastGoodEstimate=PumECRate;
PumECRateStart=PumECRate;

R1=(R1+Ra);
  delay(100);
 
GetEC(); //gets first reading for LCD and then resets max/min
MinEC=100.0;
MaxEC=0.0;
MinT=100.0;
MaxT=0.0;  
  

  //pins
  pinMode(sinkPin, OUTPUT);
  pinMode(intakePin, OUTPUT);
  pinMode(caseFan, OUTPUT);
  analogWrite(caseFan, 175);
  pinMode(exhaustFan, OUTPUT);
  analogWrite(exhaustFan, 175);
  //for (int i = 0; i <= 3; i++) {
    //pinMode(rgb[i], OUTPUT);
    //digitalWrite(rgb[i], 0);
  //}
 
  //pinMode(buttonNONE, INPUT_PULLUP);


  pinMode(buttonLEFT, INPUT_PULLUP);
  pinMode(buttonRIGHT, INPUT_PULLUP);
  pinMode(buttonUP, INPUT_PULLUP);
  pinMode(buttonDOWN, INPUT_PULLUP);
  pinMode(buttonSELECT, INPUT_PULLUP);
  pinMode(buttonMENU, INPUT_PULLUP);
 
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);

  //fast pwm
  //waveform generation mode 
  TCCR1A |= bit(WGM10);
  TCCR1A &= ~bit(WGM11);
  TCCR1B |= bit(WGM12);
  TCCR1B &= ~bit(WGM13);

  //clock select: 1 (no prescaler)
  TCCR1B |= bit(CS10);
  TCCR1B &= ~(bit(CS11) | bit(CS12));

  
  bme.begin();
  attachInterrupt(digitalPinToInterrupt(buttonMENU), buttonISR, LOW);
  
  Serial.begin(9600);
  lcd.begin();      
  

GetEC(); //gets first reading for LCD and then resets max/min
MinEC=100.0;
MaxEC=0.0;
MinT=100.0;
MaxT=0.0;


};

void loop(){ 

if(millis()<=5000){

  NutesAdded=0;

  digitalWrite(Pump,OFF);

    delay(5000);

};

if( (buttonRIGHTState==LOW || millis()%DosingInterval<=1000) && (error>=4 && EC25<=(ECSetpoint-0.1) && NutesAdded==0) ){

ECTuning();

NutrientAddition();

NutesAdded=1;

Doses++;

};

if((millis()>=(PumpTime+MixingTime+StartDosingMillis)) && NutesAdded==1){

EstimateEC();                                        // Goes to the script to estimate the nutrient source EC

NutesAdded=0;

};

if (millis()%5000<=1000 && ECHold==0){

GetEC();

ECHold=1;

};

if (millis()%5000>=1000){

ECHold=0;

};

if (millis()>=StartDosingMillis+PumpTime){  //Still need to take care of millis rollover!?

digitalWrite(Pump,OFF);

};



//All these functions are put below the main loop, keeps the loop logic easy to see
read_buttons();
startupinfo();
readSensors();
outputs();
read_Temp();
Log_Min_MaxTemp();
ReadPH();
Log_Min_MaxPH();
uptime();


//Used to see hoe many days ago the probe was calibrated

Calibration();
ChangeECSetpoint();
Error();
Day_Change();
CalibratePH();
PrintReadings();
delay(100);

if(NutesAdded==0) Log_Min_MaxEC();
delay(50);


  i=1;
  buffer=0;
  
while(i<=10){
digitalWrite(ECPower,HIGH);
raw= analogRead(ECPin);
raw= analogRead(ECPin);// This is not a mistake, First reading will be low
digitalWrite(ECPower,LOW);
buffer=buffer+raw;
i++;
delay(5000);
};
raw=(buffer/10);
//
//sensors.requestTemperatures();// Send the command to get temperatures
//TemperatureFinish=sensors.getTempCByIndex(0); //Stores Value in Variable
  
EC =CalibrationEC*(1+(TemperatureCoef*(TemperatureFinish-25.0))) ;  
Vdrop= (((Vin)*(raw))/1024.0);
Rc=(Vdrop*R1)/(Vin-Vdrop);
Rc=Rc-Ra;
K= 1000/(Rc*EC);  
  
Serial.print("Calibration Fluid EC: ");
Serial.print(CalibrationEC);
Serial.print(" S  ");  //add units here
Serial.print("Cell Constant K");
Serial.print(K);


if (TemperatureStart==TemperatureFinish){
  Serial.println("  Results are Trustworthy");
  Serial.println("  Safe To Use Above Cell Constant in Main EC code");

}
else{
  Serial.println("  Error -Wait For Temperature To settle");
}

  
  delay(1000);
  //==== adjust fan speeds ======= every 2min 
  if (millis() - lastFanTime >= 120000ul) {
    lastFanTime = millis();
    adjustPower();
  }

  //set value
  analogWrite(sinkPin, sinkP * 255 / 100);
  analogWrite(intakePin, intakeP * 255 / 100);

  //==== light control ========== 
  if (!flower) lightsOffPeriod = 21600000ul;    //6h
  else lightsOffPeriod = 43200000ul;        //12h

#ifdef TEST
  digitalWrite(lights, 1);
#else
  //rest of whole days
  if ((automatic && millis() % 86400000ul > lightsOffPeriod) || (!automatic && status)) digitalWrite(lights, 1);
  else digitalWrite(lights, 0);
#endif
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

float readT() {
  //NTC calculation
  float vcc = readVcc() / 1000.0;
  float v = vcc * (float) analogRead(thermistor) / 1023;
  float r = -4700 * (v / (v - vcc));
  float revT = (float) 1 / 298 + (float) 1 / beta * (float) log(r / 4700);
  return (1 / revT - 277);
}

void startupinfo(){

Serial.println("pH Probe Script for arduino");

Serial.println("Released under GNU by Michael Ratcliffe");

Serial.println("www.MichaelRatcliffe.com");

Serial.println("Element14 'Adapted_Greenhouse'");

Serial.println("Using DFRobot PH Probe Pro ");

Serial.println("How to Use:");

Serial.println("1:Place Probe into pH7 calibration fluid, open serial ");

Serial.println("2:Take Recomened cell constand and change it in the top of code");

Serial.println("3:Rinse Probe and place in pH4 calibration fluid");

Serial.println("4:Adjust potentiometer on pH meter shield until ph reading in serial is 4");

Serial.println("    ");

Serial.println("Thats it your calibrated and your readings are accurate!");
}

void readSensors() {

  h = bme.readHumidity() ;
  t = bme.readTemperature();
  T = readT();
  alt = bme.readPressure() / 100.0F;
  pressure = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void read_Temp(){

sensors.requestTemperatures();// Send the command to get temperatures
Temperature=sensors.getTempCByIndex(0); //Stores Value in Variable
}

void ReadPH(){

  i=0;

  sum=0;

  while(i<=20){

  reading=analogRead(PHPin);

  sum=sum+reading;

    delay(10);

    i++;

  }

average=sum/i;

//Converting to mV reading and then to pH

mvReading=average*Vs/1024;

//phValue=mvReading*K_PH;  

phValue=(7-((mvReading_7-mvReading)*Slope));

}

void read_buttons(){

int buttonRIGHT=HIGH;
int buttonUP=HIGH;
int buttonLEFT=HIGH;
int buttonDOWN=HIGH;
int buttonSELECT=HIGH;



buttonRIGHTState = digitalRead(buttonRIGHT);
buttonLEFTState = digitalRead(buttonLEFT);
buttonUPState = digitalRead(buttonUP);
buttonDOWNState = digitalRead(buttonDOWN);
buttonSELECTState = digitalRead(buttonSELECT);

//adc_key_in = analogRead(0);      // read the value from the sensor

// my buttons when read are centered at these valies: 0, 144, 329, 504, 741

// we add approx 50 to those values and check to see if we are close

if (buttonState=0; 
   buttonState=button);


//> 1000)  

else if (buttonState!=buttonRIGHTState ;
         buttonRIGHTState=LOW);

else if (buttonState!=buttonUPState;
          buttonUPState=LOW) ;

else if (buttonState!=buttonDOWNState;
         buttonDOWNState=LOW);

else if (buttonState!=buttonLEFTState ; 
          buttonLEFTState=LOW) ;

else if (buttonState!=buttonSELECTState;  
         buttonSELECTState=LOW) ;




if (Screen>=12) Screen=12;

if(Screen<=1) Screen=1;


//Second bit stops us changing screen multiple times per input

if(buttonUPState==LOW&&buttonLast!=buttonState){

Screen++;

}

else if (buttonDOWNState==LOW&&buttonLast!=buttonState){

Screen--;

};

if (Screen>=Last_Screen_no) Screen=Last_Screen_no;
if(Screen<=1) Screen=1;
buttonLast=buttonState;

};

void uptime(){

//** Making Note of an expected rollover *****// 

if(Day>=30){

HighMillis=1;

}

//** Making note of actual rollover **//

if(millis()<=100000&&HighMillis==1){

Rollover++;

HighMillis=0;

}

//Calculating the uptime

long secsUp = millis()/1000;

Second = secsUp%60;

Minute = (secsUp/60)%60;

Hour = (secsUp/(60*60))%24;

Day = (Rollover*50)+(secsUp/(60*60*24));  //First portion takes care of a rollover [around 50 days]

};

//************************** Printing somthing useful to LCd on start up **************************//

void Splash_Screen(){

lcd.begin();              // start the library
lcd.clear();
lcd.setCursor(0,0);
delay(1000);
lcd.print("PH Meter");
lcd.setCursor(0,1);
delay(1000);
lcd.print("EC Meter");
delay(1000);
lcd.setCursor(0,2);
lcd.print("Nutrient Doser");
delay(1000);
lcd.clear();
lcd.setCursor(8,0);
lcd.print("McRawmey V2");
delay(1000);
lcd.setCursor(0,2);
lcd.print("To Navigate");
delay(1000);
lcd.setCursor(0,3);
lcd.print("Use Up-Down");
delay(1000);
lcd.setCursor(0,4);
lcd.print("To Calibrate");
delay(1000);
lcd.setCursor(0,5);
lcd.print("Hold Select");
delay(5000);
lcd.clear();
};

//******************** calculating the PhMeter Parameters ***************************************//

void Slope_calc(){

offset=Healthy1_mv-mvReading_7;

Slope=3/(Healthy1_mv-mvReading_4-offset);

}

//******************************* LOGS Min/MAX Values*******************************//
void Log_Min_MaxEC(){
                            
                            
//**********So We dont Get First Few Readings**********//
                                                                
if( Doses==5){
MinEC=100.0;
MaxEC=0.0;
};

if(EC25>=MaxEC)  MaxEC=EC25;
if(EC25<=MinEC)  MinEC=EC25;
                            
                                  
};




void Log_Min_MaxTemp(){
                                                   
if(Temperature>=MaxT) MaxT=Temperature;

if(Temperature<=MinT) MinT=Temperature;
                                                                                              
};


//******************************* LOGS Min/MAX Values*******************************//

void Log_Min_MaxPH(){
                         
                        
if(phValue>=MaxPH) MaxPH=phValue;

if(phValue<=MinPH) MinPH=phValue;
                                            
                                               
};

void PrintReadings(){
              
Serial.print("pH:"); 
Serial.print(phValue);
Serial.print(Temperature);
Serial.println(" *C");
Serial.print("mv: "); 
Serial.print(mvReading);
Serial.print("   mvPh7: ");
Serial.print(mvReading_7);
Serial.print("   mvPh4: ");
Serial.println(mvReading_4);
Serial.print("H1: "); 
Serial.print(ProbeLife1);
Serial.print("   H2: ");
Serial.print(ProbeLife2);
Serial.print("H3: "); 
Serial.println(ProbeLife3);
Serial.print("Temp/led =   ");      
Serial.println(T);
Serial.println("         °C");
Serial.print("Slope ");
Serial.print(Slope);
Serial.print("Temperature = ");
Serial.print(bme.readTemperature());
Serial.println(" *C");
Serial.print("Pressure = ");
Serial.print(bme.readPressure() / 100.0F);
Serial.println(" hPa");
Serial.print("Approx. Altitude = ");
Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
Serial.println(" m");
Serial.print("Humidity = ");
Serial.print(bme.readHumidity());
Serial.println(" %");
Serial.print("EC: ");   
Serial.print(EC25);
Serial.print(" Simens  ");   
Serial.print(ppm);
Serial.print(" ppm  ");   
Serial.print(Temperature);
Serial.print(" *C  ");  
Serial.print("PumpRate EC/s: ");   
Serial.println( PumECRate);
                
//** First Screen Shows Temp and hum(bme280)**//

  
  if(Screen==1){

  lcd.setCursor(14,0);
  lcd.print("McRawmeyV2");
  
  lcd.setCursor(0,1);
  lcd.print("T/air = ");
  lcd.print(t);
  lcd.println(" C");
  
  lcd.setCursor(0,5);
  lcd.print("Press = ");
  lcd.print(pressure);
  lcd.println("hPa");

  lcd.setCursor(0,3);
  lcd.print("T/led = ");
  lcd.print(T);
  lcd.println(" C");

  lcd.setCursor(0,2);
  lcd.print("Hum = ");
  lcd.print(h);
  lcd.println(" %");

  lcd.setCursor(0,4);
  lcd.print("Alt = ");
  lcd.print(alt);
  lcd.println(" m");


}

 //Second screen shows ph/EC/Tempeau/PPM
  if(Screen==2){
lcd.setCursor(14,0);
lcd.print("McRawmeyV2");
lcd.setCursor(3,1);
lcd.print("PH/EC/PPM");
lcd.setCursor(3,2);
lcd.print("PH: ");
lcd.setCursor(20,2);
lcd.print(phValue);


lcd.setCursor(0,3);
lcd.print("EC: ");
lcd.setCursor(20,3);
lcd.print(EC25);

lcd.setCursor(0,4);
lcd.print("PPM: ");
lcd.setCursor(22,4);
lcd.print(ppm);
lcd.setCursor(0,5);
lcd.print("T/eau: ");
lcd.setCursor(34,5);
lcd.print(Temperature);
lcd.print(" C");
  }
  

                                     
else if(Screen==9){
                                    
lcd.setCursor(0,0);
lcd.print("EC.Sec-1: ");
lcd.setCursor(46,0);
lcd.print(PumECRate,5);

lcd.setCursor(16,2);
lcd.print("Doses: ");
lcd.setCursor(28,2);
lcd.print(Doses);

}

else if(Screen==4){

lcd.setCursor(0,0);
lcd.print("MinPH: ");
lcd.setCursor(32,0);
lcd.print(MinPH);
lcd.setCursor(0,1);
lcd.print("MaxPH: ");
lcd.setCursor(32,1);
lcd.print(MaxPH);

lcd.setCursor(0,2);
lcd.print("MinEC: ");
lcd.setCursor(32,2);
lcd.print(MinEC);

lcd.setCursor(0,3);
lcd.print("MaxEC: ");
lcd.setCursor(32,3);
lcd.print(MaxEC);

lcd.setCursor(0,4);
lcd.print(MinT);
lcd.print(" C");

lcd.setCursor(0,5);
lcd.print(MaxT);
lcd.print(" C");



}

                                                     
else if(Screen==7){
                                    
lcd.setCursor(0,0);
lcd.print("Calibrate pH");
lcd.setCursor(0,1);
lcd.print("Hold Select");

                                   
}

else if(Screen==5){

lcd.setCursor(0,0);
lcd.print("Ph/Health %");
lcd.setCursor(0,1);
lcd.print("H1: ");
lcd.setCursor(24,1);
lcd.print(ProbeLife1);
lcd.setCursor(0,2);
lcd.print("H2: ");
lcd.setCursor(24,2);
lcd.print(ProbeLife2);

if(millis()%6000<3000){
lcd.setCursor(0,3);
lcd.print("H3: "); 
lcd.setCursor(24,3);
lcd.print(ProbeLife3);
}                                    

}

else if(Screen==6){
                               
lcd.setCursor(0,0);
lcd.print("Days since cal:");
lcd.setCursor(0,1);
lcd.print("                       ");
lcd.setCursor(0,1);
lcd.print(Days_Since_Calibration);

                                      
                                   
   if((millis()%10000<=2000) && (CalibrationWarning==1)){

lcd.setCursor(0,0);
lcd.print("Calibrate");
lcd.setCursor(0,1);
lcd.print(" the probe !");
                                                           
                           };         
}

else if(Screen==11){

lcd.setCursor(0,0);
lcd.print("Uptime Count: ");
lcd.setCursor(0,1);
lcd.print("                                    ");//Clearing LCD
lcd.setCursor(0,1);
lcd.print(Day);
lcd.setCursor(12,1);
lcd.print("Days");
lcd.setCursor(26,1);
lcd.print(Hour);

lcd.setCursor(38,1);
lcd.print(Minute);

lcd.setCursor(52,1);
lcd.print(Second);
                                                                  
}

                  
else if(Screen==10){


lcd.setCursor(16,0);
lcd.print("Factors:");
lcd.setCursor(0,2);
lcd.print("PPMC:");
lcd.setCursor(28,2);
lcd.print(PPMconversion);

lcd.setCursor(0,3);
lcd.print("K: ");
lcd.setCursor(20,3);
lcd.print(K);
lcd.setCursor(0,4);
lcd.print("a:");
lcd.setCursor(20,4);
lcd.print(TemperatureCoef);

}
else if (Screen==3){
    
  lcd.setCursor(16,0);
  lcd.print("Fan's power:");
  lcd.setCursor(0,1);
  lcd.print("Sink: ");
  lcd.setCursor(28,1);
  lcd.print((int) sinkP / 10, 100);
  delay(1000);
  lcd.print((int) sinkP % 10, 100);
  delay(2000);
  
  lcd.setCursor(0,2);
  lcd.print("Intake: ");
  lcd.setCursor(36,2);
  lcd.print((int) intakeP / 10, 100);
  delay(1000);
  lcd.print((int) intakeP % 10, 100);
  //current control mode
   
 
  if (!flower) {
    lcd.setCursor(0,4);
    lcd.print("Lights: ");
   lcd.setCursor(38,4); 
   lcd.print("Veg");
  }
   else {
   lcd.setCursor(36,4); 
   lcd.print("Flower");
  }
  
  
  automatic = !automatic;

  //updated mode
  if (automatic){ 
    lcd.setCursor(0,5);
    lcd.print("Mode: ");
    lcd.setCursor(28,5);
    lcd.print("Auto"); 
  }
    else lcd.setCursor(0,5);
    lcd.print("Mode: ");
    lcd.setCursor(28,5);
    lcd.print("Manual");
  
}

else if(Screen==8){


lcd.setCursor(0,0);
lcd.print("EC Setpoint:");
lcd.setCursor(0,1);
lcd.print(ECSetpoint,3);
                                 
lcd.setCursor(0,3);
lcd.print("Hold Select");
lcd.setCursor(0,4);
lcd.print("to change");                   
}
 


 

if((millis()%6000)<=3000){

            
//************************* Printing any errors we have ***********//

if  (error==1) {
  
lcd.setCursor(0,0);
lcd.print("Error: ");
lcd.setCursor(32,0);
lcd.print(error);
lcd.setCursor(0,1);
lcd.print("Probe Problem"); 
}

                                  

else if (error==2) {
  
lcd.setCursor(0,0);
lcd.print("Error: ");
lcd.setCursor(32,0);
lcd.print(error);
lcd.setCursor(0,1);
lcd.print("L Overshoot"); 
}

                                  

else if (error==3) {
  
lcd.setCursor(0,0);
lcd.print("Error: ");
lcd.setCursor(32,0);
lcd.print(error);
lcd.setCursor(0,1);
lcd.print("Temp Problem"); 
}

                                  
else if (error==4) {
  
lcd.setCursor(0,0);
lcd.print("Error: ");
lcd.setCursor(32,0);
lcd.print(error);
lcd.setCursor(0,1);
lcd.print("Pump Problem"); 
}

                                  

else if (error==5  ) {
  
lcd.setCursor(0,0);
lcd.print("Error: ");
lcd.setCursor(32,0);
lcd.print(error);
lcd.setCursor(0,1);
lcd.print("System Learnin"); 
}

else;
             
              
};
                                             
}

//************************************* Used For Probe time Since Last Calibration *************************************//

//This wont keep counting if unit has been powered off, so it is only really useful for use when in permanant use

void Day_Change(){

if(Day!=Last_Day){

    Last_Day=Day;

    Days_Since_Calibration++;

    EEPROM.write(addresseCalibrationDays,Days_Since_Calibration);

    if(Days_Since_Calibration>=Cal_Period_Warning) CalibrationWarning=1;

};

};

//********************** The Section Below will give you a read out of probe Life ****************************************//

//there are three ways we can measure a proble life during calibration:

//1:Asymmetry potential (Eo), the Millivoltes at pH7 [mV reading in pH 7.00 buffer ± 25 mV]

//2:Slope, mv per Ph change [mV reading in pH 7.00 buffer - mV reading in pH 4.00 buffer 160 – 180 mV]

//3:Drift mV reading in pH 4.00 buffer (1 min) – mV reading in pH 4.00 buffer (2 min) ± 1.5 mV  [we are checking for stable readings]

void ProbeLife_Check_1(){

//Structure, get pH7 mv reading and map life vs mv reading , variables Good value, band for bad reading

//We already just got the mvReading int he calibration function

//map(value, fromLow, fromHigh, toLow, toHigh);

value=mvReading_7*1000;

if (value>=Healthy1_mv2) ProbeLife1=map(value, Healthy1_mv2, (Healthy1_mv2+Health1_range), 100, 0);

if (value<Healthy1_mv2) ProbeLife1=map(value, Healthy1_mv2, (Healthy1_mv2-Health1_range), 100, 0);

if (ProbeLife1<=1) ProbeLife1=0;

EEPROM.write(addresProbleLife1,ProbeLife1);

};

void ProbeLife_Check_2(){

//structure compare ph7 mv to ph4 mv , variables Good value Band for bad reading


value=Slope*1000;

if (value>=Healthy2_Slope) ProbeLife2=map(value, Healthy2_Slope,(Healthy2_Slope+ Healthy2_range), 100, 0);

if (value<Healthy2_Slope) ProbeLife2=map(value, Healthy2_Slope,(Healthy2_Slope-Healthy2_range), 100, 0);

if (ProbeLife2<=1) ProbeLife2=0;

EEPROM.write(addresProbleLife2,ProbeLife2);

};


void ProbeLife_Check_3(){

//structure, leave probe in ph 4 for 1 minute and then take another average at 2 mins map from 0 drift to 5mv for probe life

Drift=(mvReading_4-mvReading_4_Delayed);

value=Drift*1000;

if (value>=Healthy3_Drift) ProbeLife3=map(value, Healthy3_Drift,(Healthy3_Drift+ Healthy3_range), 100, 0);

if (value<Healthy3_Drift) ProbeLife3=map(value, Healthy3_Drift,(Healthy3_Drift- Healthy3_range), 100, 0);

if (ProbeLife3<=1) ProbeLife3=0;

EEPROM.write(addresProbleLife3,ProbeLife3);

}

void adjustPower() {
  //HEAT SINK
  //if lamps off: fan off
  if (T <= 27) sinkP = 0;
  //low enough temp
  else if (T <= sinkAim - sinkHys && sinkP >= 1) sinkP--;
  //higher temp
  else if (T > sinkAim + sinkHys && sinkP <= 85) sinkP += 10;


  //INTAKE
  //too hot or humid: crank it!
  if ((t >= 25 || h > 80) && intakeP < 85) intakeP += 5;
  //cool enough: calm it! 
  else if (t < 25 && intakeP > 15) intakeP--;
}

void changeMode() {
  //digitalWrite(rgb[0], 1);
  //analogWrite(rgb[2], 150);
  //analogWrite(rgb[3], 75);
  //digitalWrite(rgb[1], 0);

  //wait 5s
  for (int i = 0; i < 500; i++) delay(10);
  //digitalWrite(rgb[1], 1);
  delay(800);

  if (pressed && presses == 3 + 2) {
    flower = !flower;
    //confirmation
    if (!flower) lcd.print("Veg"); else if (flower) lcd.print("Flower");
    lcd.clear();

    pressed = false;
    presses = 0;
   
  }
}

void switchLightControl() {
  //current control mode
  if (automatic) lcd.print("Automatic"); else lcd.print("Manual");
  
  automatic = !automatic;

  //updated mode
  if (automatic) lcd.print("Automatic"); else lcd.print("Manual");
 
}

void outputs() {
  if (pressed) {

    //wait 4s for other presses
    for (int i = 0; i < 400; i++) delay(10);

    // 5sec are over,  display stuff:
    switch (presses) {
      //case 1: displayTempHum(); break;
//      case 2: displayFans(); break;
      case 1: status = !status; break;
      case 2: switchLightControl(); break;
      case 3: changeMode(); break;
      default:
       lcd.print("Flower");
       lcd.clear();

    }

    // reset communication with buttonISR
    presses = 0;
    pressed = false;
  }
}

void buttonISR() {
  if (millis() - pushTime >= 200) {
    presses++;

    //set flag
    pressed = true;
    pushTime = millis();
    
  }
}

void NutrientAddition(){

              
PreDosingEC =EC25; //Makes note of the initial EC

PumpTime=1000*((ECSetpoint-PreDosingEC)/PumECRate);

StartDosingMillis=millis();

digitalWrite(Pump,ON);
             
};

void EstimateEC(){
              
PostDocingEC=EC25;  //Makes Note of the EC at the end of the addition
            
//*********Rapidly change EC Estimation on startup************//                        

if(PostDocingEC>=(PreDosingEC+0.05)){
            
PumECRate =(1000.0*((PostDocingEC-PreDosingEC)/PumpTime));
LastGoodEstimate=PumECRate;

}                          
else PumECRate=PumECRate/2.0;                          
};

void GetEC(){
  
raw= analogRead(ECPin);

//sensors.requestTemperatures();// Send the command to get temperatures
//Temperature=sensors.getTempCByIndex(0); //Stores Value in Variable

digitalWrite(ECPower,HIGH);
raw= analogRead(ECPin);
raw= analogRead(ECPin);// This is not a mistake, First reading will be low beause if charged a capacitor
digitalWrite(ECPower,LOW);

Vdrop= (Vin*raw)/1024.0;
Rc=(Vdrop*R1)/(Vin-Vdrop);
Rc=Rc-Ra;
EC = 1000/(Rc*K);

//*************Compensating For Temperaure********************//

EC25  =  EC/ (1.0+ TemperatureCoef*(Temperature-25.0));

ppm=(EC25)*(PPMconversion*1000);
                                       

//********** end of Debugging Prints *********
          
};


void Uptime (){                                                                              

if(EC25>=MaxEC)  MaxEC=EC25;

if(EC25<=MinEC)  MinEC=EC25;
                      
if(Temperature>=MaxT) MaxT=Temperature;

if(Temperature<=MinT) MinT=Temperature;

                      

//** Making Note of an expected rollover *****//

if(millis()>=3000000000){

HighMillis=1;

                      
}

//** Making note of actual rollover **//

if(millis()<=100000&&HighMillis==1){

Rollover++;

HighMillis=0;

}

                      

long secsUp = millis()/1000;

Second = secsUp%60;
Minute = (secsUp/60)%60;
Hour = (secsUp/(60*60))%24;
                     
Day = (Rollover*50)+(secsUp/(60*60*24));  //First portion takes care of a rollover [around 50 days]
                                              

};

//******************************* Checks if Select button is held down and enters Calibration routine if it is ************************************//

void Calibration(){

if(Screen!=8) return;

if(buttonState!=buttonSELECTState==LOW) return;

else delay(1000);

read_buttons();

if(buttonState!=buttonSELECTState==LOW) return;


while(1){

read_buttons();
lcd.setCursor(0,0);
lcd.print("Set Cal EC");
lcd.setCursor(0,1);
lcd.print("EC: ");
lcd.setCursor(3,1);
lcd.print(CalibrationEC);


if (buttonUPState==LOW) CalibrationEC=CalibrationEC+0.01 ;
if(buttonDOWNState==LOW)  CalibrationEC=CalibrationEC-0.01;
if(buttonRIGHTState==LOW) break;
delay(100);

      };

lcd.setCursor(0,0);
lcd.print("Calibrating");
lcd.setCursor(0,1);
lcd.print("EC:                ");
lcd.setCursor(3,1);
lcd.print(CalibrationEC);

i=1;
buffer=0;
sensors.requestTemperatures();// Send the command to get temperatures
TemperatureStart=sensors.getTempCByIndex(0); //Stores Value in Variable


//************Estimates Resistance of Liquid ****************//

while(i<=10){
                     
digitalWrite(ECPower,HIGH);

raw= analogRead(ECPin);
      
digitalWrite(ECPower,LOW);

buffer=buffer+raw;

i++;

delay(5000);
};
       

raw=(buffer/10.0);

sensors.requestTemperatures();// Send the command to get temperatures

TemperatureFinish=sensors.getTempCByIndex(0); //Stores Value in Variable


//*************Compensating For Temperaure********************//

EC =CalibrationEC*(1+(TemperatureCoef*(TemperatureFinish-25.0))) ;

//***************** Calculates R relating to Calibration fluid **************************//

Vdrop= (((Vin)*(raw))/1024.0);
Rc=(Vdrop*R1)/(Vin-Vdrop);
Rc=Rc-Ra; //Taking into account pin resistance
Kt= 1000/(Rc*EC);



if (TemperatureStart==TemperatureFinish ){
Serial.println("Results are Trustworthy");
Serial.print("Calibration Fluid EC: ");
Serial.print(CalibrationEC);
Serial.print(" S  ");  //add units here
Serial.print("Cell Constant K");
Serial.print(K);

lcd.setCursor(0,0);
lcd.print("Good Results");                

lcd.setCursor(0,1);
lcd.print("EC:");
lcd.setCursor(4,1);
lcd.print(CalibrationEC);
lcd.setCursor(0,2);
lcd.print("K:");
lcd.setCursor(3,2);
lcd.print(Kt);

                  

while (1) { // wee need to keep this function running until user opts out with return function

read_buttons();

if(buttonLEFTState==LOW) return; //exits the loop without saving becauser user asked so

if(buttonSELECTState==LOW){

K=Kt; //saving the new cell constant

                          
//******8*Saving the new value to EEprom**********//

value=K/0.02;

EEPROM.write(addresCalibration, value);

lcd.setCursor(0,0);
lcd.print("Saved Cal");
lcd.setCursor(0,1);
lcd.print("K:");
lcd.setCursor(3,1);
lcd.print(Kt);
delay(2000);
return;

}
                         
if(millis()%4000>=2000){
lcd.setCursor(0,0);
lcd.print("Good Results");                                                  

lcd.setCursor(0,1);
lcd.print("EC:");
lcd.setCursor(4,1);
lcd.print(CalibrationEC);
lcd.setCursor(0,2);
lcd.print("K:");
lcd.setCursor(3,2);
lcd.print(Kt);                          
}

else{
lcd.setCursor(0,0);
lcd.print("Select To Save");
lcd.setCursor(0,1);
lcd.print("Down to Exit");
};
                                                      
}
                 
}

                    
else{

Serial.println("  Error Wait For Temperature To settle");

while (1) {
read_buttons();
if(buttonUPState==LOW) Calibration();
if(buttonDOWNState==LOW) return;

lcd.setCursor(0,0);
lcd.print("Bad Results");
lcd.setCursor(0,1);
lcd.print("Press DWN Exit");

                                    
  }

                 
}
};

//******************************* Checks if Select button is held down and enters Calibration routine if it is ************************************//

void CalibratePH(){

//we check if we are on ph screen and the select button is held

if(Screen!=7) return;

if(buttonState!=buttonSELECTState==LOW) return;

else delay(1000);

read_buttons();

if(buttonState!=buttonSELECTState==LOW) return;

//we need to stop in this loop while the user calibrates

while(1){

read_buttons();

 lcd.setCursor(0,0);
 lcd.print("Probe in pH7");
 lcd.setCursor(0,1);
 lcd.print("Press Right");

//user pressed right?

if(buttonRIGHTState==LOW) break;

delay(100);

       };

StartCalibration1=millis();

//We are giving the probe 1 minute to settle

while(millis()<StartCalibration1+60000){

  lcd.setCursor(0,0);
  lcd.print("Calibrating");
  lcd.setCursor(0,1);
  lcd.print("                      ");
  lcd.setCursor(0,1);
  lcd.print(StartCalibration1+60000-millis());
  lcd.setCursor(0,2);
  lcd.print(":mSeconds");

delay(1000);

};

ReadPH();

delay(100);

ReadPH();

//*******Saving the new value to EEprom**********//  

   mvReading_7=mvReading;

while (1) { // wee need to keep this function running until user opts out with return function

                            
  read_buttons();

  if(buttonDOWNState==LOW) return; //exits the loop without saving becauser user asked so
                       
if(millis()%4000>=2000){

lcd.setCursor(0,0);
lcd.print("Calibrated");
lcd.setCursor(0,1);
lcd.print("Select to Save");                            

}                             
else{

lcd.setCursor(0,1);
lcd.print("Down to Exit");

                              };
if (buttonSELECTState==LOW) break;

}                              

//read the ph probe

//Saving the value steight from ADC, removes conversion errors                              

   value= average/4;

   EEPROM.write(addresCalibrationPH7,value);

   //Resetting the days sinc calibration

   Days_Since_Calibration=0;

   CalibrationWarning=0;

   Slope_calc();

   ProbeLife_Check_1();

   EEPROM.write(addresseCalibrationDays,Days_Since_Calibration);

   //Displaying the new offset

 lcd.setCursor(0,0);
 lcd.print("Saved Cal");
 lcd.setCursor(7,1);
 lcd.print("Offset");
 lcd.setCursor(3,1);
 lcd.print(offset);

delay(2000);

   //move onto pH4 Calibration
   lcd.setCursor(0,0);
   lcd.print("Rinse and");
   lcd.setCursor(0,1);
   lcd.print("Place in pH4");

   delay(4000);

   while(1){

  lcd.setCursor(0,0);
  lcd.print("Press Right");                                                         
  lcd.setCursor(0,1);
  lcd.print("If Probe in 4");

   //move onto next stage if select is held

   read_buttons();

   if (buttonRIGHTState==LOW) break;

   }

StartCalibration1=millis();

//We are giving the probe 1 minute to settle

while(millis()<StartCalibration1+60000){

lcd.setCursor(0,0);
lcd.print("Calibrating");
lcd.setCursor(0,1);
lcd.print("                      ");
lcd.setCursor(0,1);
lcd.print(StartCalibration1+60000-millis());
lcd.setCursor(0,2);
lcd.print(":mSeconds");

delay(1000);

};

ReadPH();

delay(100);

ReadPH();

mvReading_4=mvReading;

//We are giving the probe 1 minute to settle

  StartCalibration1=millis();

  while(millis()<=StartCalibration1+60000){

  lcd.setCursor(0,0);

  lcd.print("Health Check");

  lcd.setCursor(0,1);

  lcd.print("                      ");

  lcd.setCursor(0,1);

  lcd.print(StartCalibration1+60000-millis());

lcd.setCursor(0,2);

lcd.print(":mSeconds");

  delay(1000);

};

ReadPH();

delay(100);

   ReadPH();

   mvReading_4_Delayed=mvReading;

   //Saving ADc readout, to remove conversion errors

   value= average/4;

EEPROM.write(addresCalibrationPH4,value);

   Slope_calc();

   ProbeLife_Check_2();
   ProbeLife_Check_3();

   //Put back to main screen and exit calibration
   
  lcd.setCursor(0,0);
  lcd.print("Saved Cal");

  delay(1000);

//Informing the use about the probe life

while(1){

read_buttons();

if(ProbeLife2>=50 && ProbeLife2>=50 &&ProbeLife3>=50){
lcd.setCursor(0,0);
lcd.print("ProbeCondition:");
lcd.setCursor(0,1);
lcd.print("Good"); }

if(ProbeLife2<50 || ProbeLife2<50 || ProbeLife3<50){

lcd.setCursor(0,0);

lcd.print("ProbeCondition:");

lcd.setCursor(0,1);

lcd.print("Faulty"); }





if(millis()%6000 <=3000){

lcd.setCursor(0,0);

lcd.print("Press Right");

                                                          

lcd.setCursor(0,1);

lcd.print("To exit");

};


//user pressed right?

if(buttonRIGHTState==LOW) break;

delay(100);

       };

   Screen=1;

   return;                                                       


};


//**************************************** End OF CALIBRATION ROUTINE ******************************//

//**************************************Start OF Function To Change EC Setpoint*********************//

void ChangeECSetpoint(){

if(Screen!=7) return;
if(buttonState!=buttonSELECTState==LOW) return;

else delay(1000);
read_buttons();
if(buttonState!=buttonSELECTState==LOW) return;


while(1){
read_buttons();
lcd.setCursor(0,0);
lcd.print("Set EC ");
lcd.setCursor(0,1);
lcd.print("R:exit EC:");
lcd.setCursor(0,2);
lcd.print(ECSetpoint);


if(buttonUPState==LOW) ECSetpoint=ECSetpoint+0.01 ;
if(buttonDOWNState==LOW)  ECSetpoint=ECSetpoint-0.01;
if(buttonRIGHTState==LOW) {

  value=ECSetpoint/0.02;
  EEPROM.write(addresSetpoint, value);
break;

}
delay(100);

  };

};

//*********************************Function to Check for any errors**************************************//

void Error(){

error=6; //For Some Reason the Errors are not refreshing like they shoud, this fixes it

//*********Checking if the Probe is in the Solution********//

  if(EC25<=0.05){

  Serial.println("Probe Problem: Check it is in the liquid and not shorted ");

  error=1;

  }

//***********Checking if we seriosly overshot the setpoint**//

  else if((EC25>=(ECSetpoint*1.3))&& NutesAdded==0){

  Serial.println("Dosing Problem: We overshot the setpoint, Will attempt to fix its self ");

  error=2;

  }

//******** Checking Temp Probe is In range ****************//

else if((Temperature>=50.0) || (Temperature<=0.5)){

  Serial.println("Temperature Problem: Check Wiring [Also Check Probe has pull up] ");

  error=3;

}

//****** Checking Dosing Pump Is working******************//

else if( (PostDocingEC<=(PreDosingEC+0.05) ) && (NutesAdded==0 && EC25<=ECSetpoint-0.2) ){

Serial.println("Dosing Problem: Check Nute Tank is full and pump is not broken ");

if(Doses<=10){

//Still Geting a good estimate for the EC

//error=5;


};

if(Doses>=11) {

//Proberbly a pump problem

error=4;

};
     }
else error=6;

};

//***************** EC Optimisation ***************************//

void ECTuning(){

//stops us changing to early on [may not be needed

if(Doses<=10) return;

if(PostDocingEC>=EC25)ECSetpoint=ECSetpoint+0.1;

if(PostDocingEC<=EC25)ECSetpoint=ECSetpoint-0.1;

//Stoping the system getting a bit giddy

if (ECSetpoint>=MaxECSetpoint) ECSetpoint=MaxECSetpoint;

if(ECSetpoint<=MinECSetpoint) ECSetpoint=MinECSetpoint;

};

void Read_Eprom(){

//************** Restart Protection Stuff ********************//

  //the 254 bit checks that the adress has something stored to read [we dont want noise do we?]

   value = EEPROM.read(addresCalibrationPH7);

   mvReading_7=value*Vs/256;

   delay(10);

value = EEPROM.read(addresCalibrationPH4);

   mvReading_4=value*Vs/256;

   delay(10);

//Checking for Probe Life 1 Indicator

value = EEPROM.read(addresProbleLife1);

   ProbeLife1=value;

   delay(10);

//Probe:ife 2 saved

   value = EEPROM.read(addresProbleLife2);

   ProbeLife2=value;

   delay(10);

//Probe:ife 3 saved

   value = EEPROM.read(addresProbleLife3);

   ProbeLife3=value;

   delay(10);

//Checking memory slot for probe calibration time

  Days_Since_Calibration = EEPROM.read(addresseCalibrationDays);

//Setpoint
   value = EEPROM.read(addresSetpoint);
   //the 254 bit checks that the adress has something stored to read [we dont want noise do we?]
   if (value <=254) ECSetpoint=value*0.02;
   //Calibration 
   value = EEPROM.read(addresCalibration);
   if (value <=254) K=value*0.02;

};
