/*
//*****************************************************************************
 
 SolarViewMeter						 
 		
 v. 1.3 - Display current performance of a solar cell
 
 2011 - Nicola Ferralis - ferralis@mit.edu					  
 
 This program (source code and binaries) is free software; 
 you can redistribute it and/or modify it under the terms of the
 GNU General Public License as published by the Free Software 
 Foundation, in version 3 of the License.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You can find a complete copy of the GNU General Public License at:
 
 http://www.gnu.org/licenses/gpl.txt 
 
 //**********************************************************************************
 
 User Notes: 
 
 1. Resistors for voltage dividers are labeled Rv1 and Rv2, while shunts for current 
 measurements are labeled Ri. They are arrays of floats, with number defined by the
 variable numCell. Therefore the number of values in the arrays needs to be properly
 adjusted according to numCell. 
 
 2. Connections for MEGA and UNO boards are the same, apart from the SPI connections:
 ---------------------------------------------------
 |  Pin potentiometer     |  Pin MEGA  |  Pin UNO  |
 ---------------------------------------------------
 |     8 - CLK (SCK)      |     52     |     13    |
 |     7 - SDI (MOSI)     |     51     |     11    |
 |     5 - CS (SS)        |     53     |     10    |
 ---------------------------------------------------
 
 3. Connect SDA 4 for UNO, 20 for MEGA; 
    Connect SCL 5 for UNO, 21 for MEGA.
 //**************************************************************************************
 */

//#include <SD.h> 
#include <Wire.h>
#include <SPI.h>
//#include "RTClib.h"

#define TERM   // comment for serial comm via LCD. Uncomment for regular use with Serial Terminal

//#define MEGA // comment for Arduino UNO, uncomment for MEGA

//#define VSC // Uncomment this to measure Vsc

//#define SPIPOT //comment to use with I2C bus potentiometer (AD5254), uncomment for SPI (AD5206)

//------------------
// Name and version 
//------------------

String nameProg = "SolarViewMeter";
String versProg = "1.3";
String versDate = "20111007";
String developer = "Nicola Ferralis - ferralis@mit.edu";


//------------------
//Program variables
//------------------

void(* resetFunc) (void) = 0; //declare reset function @ address 0
int numCell = 1;        // Max number of cells to be measured

int voltIN = 0;  // Applies the correction to the input voltage for a particular cell is the voltage divider is present (0), 
// or leave it with no correction (1).

float Rv1 = 10000.0;        //resistor for voltage divider (top to Vin) 
float Rv2 = 10000.0;         //resistor for voltage divider (bottom, to ground)

float Ri = 1.0;    // resistor for current measurement A.

float Ri1 = 1000.0;         //resistor for current amplification (fixed resistor 1K) 
float Ri2 = 100000.0;     //resistor for current amplification (value of resistor determines the amplification factor) 

float Vcv = 0.0;
float Vci = 0.0;
// Because of the SD/RTC shield, the first available analog pin is #8
int Vp=4; //voltage pin
int Ip=5; //current pin

float maxVolt = 0.0; // Max voltage on scale. 
float lowV = 1.0;     // min voltage for LED warning

int avNum = 20;     // number of averages to be taken over an analog input
int ledPin = 13;       // on actual arduino boards this is pre-hooked up.

const int T1= 6;      // Transistor for Voc1

unsigned long restTime = 12;  //Time in between IV scans (minutes)
unsigned int delayTime = 1000; // Generic time delay (ms)


//-----------------------------
// SPI specific (digital pot)
//-----------------------------

#ifdef MEGA 
const int slaveSelectPin = 53;
#else
const int slaveSelectPin = 10;
#endif

//--------------------
// buttons definition
//--------------------

volatile int perfbtn = 0;
volatile int IVbtn = 0;

boolean wp = true;


//////////////////////////////////////////////////////////
// SETUP 
///////////////////////////////////////////////////////////

void setup()
{ //----------------------------------------
  // Initialize serial port 
  //----------------------------------------
  Serial.begin(9600);
  
  
  #ifdef TERM
  #else
  backlightOn();
  #endif

  //----------------------------------------
  // set the transistor pin as output
  //----------------------------------------
  pinMode(T1, OUTPUT);

  //----------------------------------------
  // set the slaveSelectPin as an output:
  //----------------------------------------
  pinMode (slaveSelectPin, OUTPUT);

  //----------------------------------------
  // initialize SPI:
  //----------------------------------------
  #ifdef SPIPOT
  SPI.begin(); 
  #else
  Wire.begin();  
  #endif

  //----------------------------------------
  //Initialize reference voltage
  //----------------------------------------
  analogReference(DEFAULT);  //0 to 5 V
  maxVolt = 5.0;  //Measured with DVM
  //analogReference(INTERNAL2V56); //0-2.56V
  //maxVolt = 2.56;
  //analogReference(INTERNAL1V1); //0-1.1V
  //maxVolt = 1.1;

  //---------------------------------------------------
  //Stamp header with program details and data labels.
  //----------------------------------------
  delay(200);
  
  #ifdef TERM
  firstRunSerial();
  #else
  clearLCD();
     selectLineOne();
     Serial.print("SolarViewMeter");
     selectLineTwo();  
     Serial.print("v. ");
     Serial.print(versProg);
     delay(2000);
  #endif   

  
    ////////////////////////////////////////////////////////////////////
  // Setup coeffcients for voltage dividers and amplification factors
  ////////////////////////////////////////////////////////////////////
  

    if(voltIN==0)
    {
      // set the coefficient to get the real voltage V before the divider  
      Vcv=Rv2/(Rv2+Rv1); 
    }
    else
    {
      Vcv=1.0;
    }
  
  // set the amplification factor for voltage Vi of the shunt 
  Vci=(Ri1 + Ri2)/Ri1;
  //Vci = 100.0;

   #ifdef TERM 
   mainMenu();
   #endif  
  
    perfbtn = 0;
    IVbtn=1;
  
}

///////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////

void loop()
{
  //////////////////////////////////////////////
  //  Enable the following for Serial terminal
  //////////////////////////////////////////////
  
  //Serial via Terminal

  #ifdef TERM
    
  // Accept input from serial 
  int inSerial = 0;    // variable for serial input
  int inSerial2 = 0;    // variable for serial input


  if (Serial.available() > 0) {
    inSerial = Serial.read();

    if(inSerial==49)
    { inSerial2 = 0;
      Serial.println("Measure performance");
   
      
      
      while(inSerial2!=51)
      {  inSerial2=Serial.read(); 
         perform();
         delay(500);
      }
      
    }

    //start acquiring IV (1): Single  - (2): iv
    if(inSerial==50)
    { 
      Serial.println("Collecting Single IV");
      ivSingle();
    }

   
    // Reset device (0)
    if(inSerial==48)  
    {
      Serial.println("Resetting device");
      Serial.println();
      resetFunc();
    }


  }
  
  #else
  
 
  
  perfbtn = 0;
  IVbtn=0;
  attachInterrupt(1, perfb, CHANGE);
  attachInterrupt(0, IVb, CHANGE);

  while(perfbtn==0) {
  wp=true;
  perform();
  
  delay(1000);}
  //perfbtn = 0;
  
   while(IVbtn==0) {
   if(wp==true) {
    selectLineOne();
    Serial.print("IV in progress  ");
    selectLineTwo();
    Serial.print("Wait, please    ");
   }
     ivSingle();
  delay(1000);
  wp=false;
  }
  //IVbtn = 0;
  #endif
}

void perfb() {
perfbtn=!perfbtn;

}

void IVb() {
IVbtn=!IVbtn;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Additional libraries
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//////////////////////////////////
// single measurement performance
///////////////////////////////////

void perform() {
  
  float V, Vi;
  float Voc, Isc;
  //int ip=fp;


  // Close circuit with transistor to allow full measurement.

  digitalWrite(T1, HIGH);
  
  #ifdef SPIPOT
    for (int g=0; g<6; g++) {
      digitalPotWriteSPI(g, 255);
    }
    #else
    //// Make sure this works first depending on the number of pots. 
    //for (int g=44; g<47; g++) {
      digitalPotWriteI2C(44,0x00, 255);
      digitalPotWriteI2C(44,0x01, 255);
    //}
    #endif
      
   delay(20);   
      

  Vi=avoltage(Ip, maxVolt, avNum)/Vci;
  Isc = Vi/Ri;
  //Isc = Vi;
 // delay(20);
  
  #ifdef VSC
  #else
  digitalWrite(T1, LOW);
  delay(10);
  #endif
  
    Voc= avoltage(Vp, maxVolt, avNum)/Vcv;
    //Ioc=avoltage(ip+1, maxVolt, avNum)/Vci;
    //ip+=2;
  
  #ifdef VSC
  digitalWrite(T1, LOW);
  #endif
  
  // This part of the code needs to be replicated to add support for the LCD.
 
   //  analysisSerial(Voc, Isc*1000, Vmax, Imax, Pmax, Vmax*Imax/(Voc*Isc*1000), jmax);
 
  #ifdef TERM
  #ifdef VSC
  Serial.print("Vsc=");
  #else
  Serial.print("Voc=");
  #endif
  Serial.print(Voc);
  Serial.print("V Vi=");
  Serial.print(Vi*1000);
  Serial.print("V  I=");
  Serial.print(Isc*1000); 
  Serial.println("mA");
  Serial.print("P=");
  Serial.print(Isc*Voc*1000);
  Serial.println("mW");
  Serial.println();
  #else
  clearLCD();
  selectLineOne();
  Serial.print("V=");
  Serial.print(Voc);
  Serial.print("V I=");
  Serial.print(Isc*1000); 
  Serial.print("mA");
  selectLineTwo();
  Serial.print("P=");
  Serial.print(Isc*Voc*1000);
  Serial.print("mW ");
  #endif
}


/////////////////////////////
// IV acquisition
/////////////////////////////

void ivSingle() {
  #ifdef TERM
  header();
  #endif
  /////////////////////////////////
  // DATA ACQUISITION AND SAVING
  /////////////////////////////////

  float V, Vi;
  float Voc, Ioc, Isc, Pmax, Vmax, Imax;
  int jmax;

  // Close circuit with transistor to allow full measurement.

  digitalWrite(T1, HIGH);

  //////////////////////////////
  //Measure IV
  //////////////////////////////

  //Setup for Isc, Pmax 
    Isc=0.0;
    Pmax = 0.0;
    jmax=0;


  // Crank up potentiometer

   for (int j=0; j< 256; j++)
  { 
    #ifdef SPIPOT
    for (int g=0; g<6; g++) {
      digitalPotWriteSPI(g, 255-j);
    }
    #else
    //// Make sure this works first depending on the number of pots. 
    //for (int g=44; g<47; g++) {
      digitalPotWriteI2C(44, 0x00, 255-j);
      digitalPotWriteI2C(44, 0x01, 255-j);
    //}
    #endif
      
    
    

    delay(20);

    #ifdef TERM
    Serial.print(j);
    Serial.print(",");
  
    #endif  

 
      // Acquire, average, and rescale with appropriate divider coefficient
      V=avoltage(Vp, maxVolt, avNum)/Vcv;  
      Vi=avoltage(Ip, maxVolt, avNum)/Vci;



      Isc=max(Isc,Vi/Ri); 

      if(Pmax<=V*Vi*1000/Ri)
      {
        Pmax=V*Vi*1000/Ri;
        Vmax=V;
        Imax=Vi*1000/Ri;
        jmax=j;
      }

      #ifdef TERM
      writeIVSerial(V,Vi,Ri);  
      #endif
    }
    
    
    #ifdef TERM
    //Serial.println();
    // Eventually insert T measurement here if T needs to be measured at every IV step 
  
  ////////////////////////
  // Measure and save Voc
  /////////////////////////

  Serial.print("\"Voc\"");
  Serial.print(",");
  //writeDateSerial();  
  #endif



  //for (int i=0; i<numCell; i++) {

    digitalWrite(T1, LOW);
    delay(20);

    Voc= avoltage(Vp, maxVolt, avNum)/Vcv;
    Ioc=avoltage(Ip, maxVolt, avNum)/Vci;


    #ifdef TERM
    writeIVSerial(Voc,Ioc,Ri);
    
  //}


  Serial.println();

  ////////////////
  // Analyse data
  ////////////////


  analysisHeaderSerial();

  //for (int i=0; i<numCell; i++) {
    
    analysisSerial(Voc, Isc*1000, Vmax, Imax, Pmax, Vmax*Imax/(Voc*Isc*1000), jmax);
 // }
  
  #else
  
  clearLCD();
  delay(20);
  selectLineOne();
  Serial.print("V=");
  Serial.print(Vmax);
  Serial.print("V I=");
  Serial.print(Imax); 
  Serial.print("mA");
  selectLineTwo();
  Serial.print("P=");
  Serial.print(Pmax);
  Serial.print("mW  ");
  Serial.print(Vmax*Imax/(Voc*Isc*1000));
  #endif
  
}



//////////////////////////////////////////////
// convert analog output into actual voltage.
//////////////////////////////////////////////

float voltage(int analogPin, float volt)
{
  int v = analogRead(analogPin); 
  float vf = volt*((float)v/(float)1024);  //rescale channel with max voltage
  return vf;
}


//////////////////////////////////////////////////////////
// collect numAver times the analog output, average them, 
// and converts them into actual voltage.
/////////////////////////////////////////////////////////

float avoltage(int analogPin, float Volt, int numAver)
{
  float vt=0.0;
  for(int i = 0; i < numAver; ++i) 
  {
    float vf=voltage(analogPin,Volt);
    vt += vf;
  }
  return vt/(float)numAver;
}


#ifdef SPIPOT
//////////////////////////////////////////////
// set potentiometer
//////////////////////////////////////////////

int digitalPotWriteSPI(int address, int value) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin,HIGH);
  }
#endif  
  
#ifdef SPIPOT  
  #else
int digitalPotWriteI2C(int address, byte a1, int value) {
  Wire.beginTransmission(address); // transmit to device #44, 45, 46, 47 
                              // device address is specified in datasheet
  Wire.send(a1);            // sends instruction byte  
  Wire.send(value);             // sends potentiometer value byte  
  Wire.endTransmission();     // stop transmitting
  }
  #endif


#ifdef TERM
void writeIVSerial(float V, float Vi, float Ri){

  Serial.print(V*1000);
  Serial.print(","); 
  Serial.println(Vi*1000/Ri);
}



///////////////////////////////////////////
// Stamp data headers on Serial  
///////////////////////////////////////////

void header(){  
  Serial.println();
  Serial.print("\"#\",");
  Serial.print("\"V (mV)\",");
    Serial.println("\"I (mA)\"");
}


///////////////////////////////////////////
// Stamp analysis header on Serial
///////////////////////////////////////////

void analysisHeaderSerial() {
  //Serial.println();
  //Serial.print("\"Cell #\",");
  Serial.print("\"Voc (V)\",");
  Serial.print("\"Isc (mA)\",");
  Serial.print("\"Vmax (V)\",");
  Serial.print("\"Imax (mA)\",");
  Serial.print("\"Pmax (mW)\",");
  Serial.print("\"FF\",");
  Serial.print("\"j\",");
  Serial.println();
}


///////////////////////////////////////////
// Stamp analysis data on Serial
///////////////////////////////////////////

void analysisSerial(float V, float I, float V1, float I1, float P1, float FF, int j) {
  //Serial.print(i);
  //Serial.print(",");
  Serial.print(V);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print(V1);
  Serial.print(",");
  Serial.print(I1);
  Serial.print(",");
  Serial.print(P1);
  Serial.print(",");
  Serial.print(FF);
  Serial.print(",");
  Serial.print(j);
  Serial.println();
  // Serial.println();

}



///////////////////////////////////////////
// Stamp program info into Serial
///////////////////////////////////////////

void firstRunSerial()
{ Serial.println();
  Serial.println("--------------------------------------");
  Serial.print(nameProg);
  Serial.print(" - v. ");
  Serial.print(versProg);
  Serial.print(" - ");
  Serial.println(versDate);
  Serial.println(developer);
  Serial.println("--------------------------------------");
  Serial.println();
}

///////////////////////////////////////////
// Stamp program info into Serial
///////////////////////////////////////////

void mainMenu()
{ 
  Serial.println("--------------------------------------");
  Serial.println("SELECT FROM THE FOLLOWING OPTIONS:");
  Serial.println("1: start continous real time acquisition");
  Serial.println("2: start IV measurements");
  Serial.println("3: stop");
  Serial.println("0: reset");
  Serial.println("--------------------------------------");
  Serial.println();
}

#else  
///////////////////////////////////////////
// Routines for LCD
///////////////////////////////////////////


void selectLineOne(){  //puts the cursor at line 0 char 0.
   Serial.print(0xFE, BYTE);   //command flag
   Serial.print(128, BYTE);    //position
   delay(10);
}
void selectLineTwo(){  //puts the cursor at line 0 char 0.
   Serial.print(0xFE, BYTE);   //command flag
   Serial.print(192, BYTE);    //position
   delay(10);
}
void goTo(int position) { //position = line 1: 0-15, line 2: 16-31, 31+ defaults back to 0
if (position<16){ Serial.print(0xFE, BYTE);   //command flag
              Serial.print((position+128), BYTE);    //position
}else if (position<32){Serial.print(0xFE, BYTE);   //command flag
              Serial.print((position+48+128), BYTE);    //position 
} else { goTo(0); }
   delay(10);
}

void clearLCD(){
   Serial.print(0xFE, BYTE);   //command flag
   Serial.print(0x01, BYTE);   //clear command.
   delay(10);
}
void backlightOn(){  //turns on the backlight
    Serial.print(0x7C, BYTE);   //command flag for backlight stuff
    Serial.print(132, BYTE);    //light level.
    
    //command flag for turning off splash screen (leave commented for regular use

   // Serial.print(0x7C, BYTE);   //command flag for backlight stuff
   // Serial.print(9, BYTE);   
   }
void backlightOff(){  //turns off the backlight
    Serial.print(0x7C, BYTE);   //command flag for backlight stuff
    Serial.print(128, BYTE);     //light level for off.
   delay(10);
}
void serCommand(){   //a general function to call the command flag for issuing all other commands   
  Serial.print(0xFE, BYTE);
}

#endif

