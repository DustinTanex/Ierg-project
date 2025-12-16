/* Designed for CHUK-IE IERG4230 course
 * Created on 2016.01.20
 * Revised on 2018.01.12, 2021.07.20, 2021.08.10
 * Copyright reserved
 * 
 * Native Hardware Serial (Tools >>> Serial Monitor), 115200bps  for debug
 */


#include <Arduino.h>
#include <Wire.h>
SoftwareSerial xBeeSerial(9, 8); //RX=9, TX=8, use software for to XBee
#include "IERG4230.h"
#include <BlynkSimpleEsp8266.h> 
#include <SoftwareSerial.h>

SoftwareSerial EspSerial(2, 3);
#define BLYNK_AUTH_TOKEN    "5Q6mnUcFQOJPiJhVpFjcZ_gVS4P1O1Rd"  // Replace with your Blynk Auth Token
#define BLYNK_TEMPLATE_ID "TMPL45IMD5Kvt"
#define BLYNK_TEMPLATE_NAME "Hydroponic System"
char ssid[] = "Bruh";  // Replace with your Wi-Fi SSID
char pass[] = "12345222";  // Replace with your Wi-Fi password


// ==== define program information, it will be shown after reset
#define programMark "IERG4230 OS_LED"      //-- You can modify this line. 
#define versionMark 1234                    //-- Any number, You can modify this line. 

// === start #define =============================================================
#define debugPerformance            //Print Performance count, you can comment it 
#define YunI2CAddress  0x18         //Dont modify, YUN I2C address
#define ESP8266I2CAddress 0x66      //Dont modify, ESP8266 I2C address
#define xbeeI2CAddress 0x27         //Dont modify, XBee I2C address
#define keyI2CAddress  0x23         //Dont modify, 16-key keypad (4x4) I2C address
#define lcdI2CAddress  0x26         //Dont modify, LCD I2C address
unsigned int performanceCounter;    //Dont modify, for performance test
char xbeeId0, xbeeId1;              //Variables store XBee ID for DIP switches
char keyI2Cdata0, keyI2Cdata1;      //Variables store current 16-key keypad (4x4) status
char keyI2Cdata0old, keyI2Cdata1old;//Variables store pervious 16-key keypad (4x4) status
char keyI2Cchanged;                 //Variable indicates keypad is changed
LcdI2C8Bit Lcd(lcdI2CAddress);      //Dont modify, create object LCD
osEvent task0;                      //Dont modify, for performance test
// ===== end of #include, #define, essential object and golbal variables  ===========
// ===== create user defined objects
osEvent task1, task2, task3, task4, task5, task6, task7, task8, task9, task10; //10 tasks object are created.
dOut ledL(13);                       //create objects, LED(L)
dOut led1(3), led2(5), led3(6);      //create objects, three on project board LEDs as digital outputs
dOut pump1(A2);  
dOut pump2(A3);  
dIn sw11(2), sw12(7);                //create objects, two on project board switches as digital inputs
aIn aI0(A0), aI1(A1);
//, aI2(A2), aI3(A3), aI6(A6), aI7(A7); //create object, six analog inputs
aOut led1a(3), led2a(5), led3a(6); //create objects, three on board LEDs as analog PWM outputs
// == add your golbal variables below ==
int temp_i;    //-- uncomment this line for Exp-1.9

float water_temp=25;
float tds_value=0;
////////////////////////////////////////////////////////////////
void setup()
{
  Wire.begin(); //start I2C bus
  Wire.setClock(40000); //-- slow down I2C speed
  I2CRead2Byte(xbeeI2CAddress, &xbeeId0, &xbeeId1);  //read two bytes for XBee ID
  I2CRead2Byte(keyI2CAddress, &keyI2Cdata0old, &keyI2Cdata1old);  //read 16-key keypad (4x4) status
  keyI2Cchanged = 0;
  Serial.begin(115200);   //--enable hwUART for debug
  delay(1000);
  printProgramMark();
  xBeeSerial.begin(9600); //enable swUART
  osEvent::osTimer = millis();
  Lcd.setUp();
  //=== dont modify above setup
  //=== user can start task at starting
  LCDshowWelcomePage();
  task0.timerSet(100);  //the first time to exectue performance test after 100ms
  task1.timerSet(100);  //the first time to exectue task1 after 100ms
  task2.timerSet(100);
  task3.timerSet(1000); //for tds
  task4.timerSet(1000); //for tds
  task10.timerSet(100);  //the first time to exectue task10 after 100ms
  pump1.on();
  pump2.off();
  Lcd.displayString(3, "XBee ID:");   // (3)=4th line
  Lcd.showFigure(3, 8, 3,  (byte) xbeeId0); // (3)=4th line, (8)=9th position (3)=length
  Lcd.showFigure(3, 12, 3,  (byte) xbeeId1);  // (3)=4th line, (12)=3th position (3)=length

  EspSerial.begin(115200); // Start the ESP8266 communication
  Blynk.begin(BLYNK_AUTH_TOKEN, EspSerial, ssid, pass);
  Serial.println("Connecting to WiFi...");

}

void loop()
{
  Blynk.run();
  while (Serial.available())
  {
    //read data from hardware buffer
  } 
  while (xBeeSerial.available())
  {
    //read data from software buffer
  }
  performanceCounter++;   //dont remove, for performace test
  hal_eventUpdate();               //check Hardware events
  if (osEvent::osTimer != millis()) timeStampUpdate();
  if (task0.isSet()) task0_handler();  //for performance test
  
  //---- dont modify above lines, user add own task handlers below ---
  if (task1.isSet()) task1_handler(); //  when task-1 timeout, run task1_handler() subroutine
//    if (sw11.eventDown) sw11_downHandler();
//    if (sw11.eventUp) sw11_upHandler();
   if (task2.isSet()) task2_handler();
    if (task3.isSet()) task3_handler();
    if (task4.isSet()) task4_handler();
  if (task10.isSet()) task10_handler(); //  when task-10 timeout, run task1_handler() subroutine
//    if (aI0.value < 500) aI0_low_handler();
//    if (aI0.value > 600) aI0_high_handler();
//    if (aI7.value < 500) aI7_low_handler();     //--uncomment this line and use suitable value for Exp1.6
//    else aI7_high_handler();                    //--uncomment this line for Exp1.6
//    if (keyI2Cchanged) keyI2C_handler();        //--uncomment this line for Exp1.8
} 
//----------------------------------------
//--- user's add handlers and subroutines below here
void task1_handler(void)  //LED1 flashing
{
  task1.clean(); //-- clear the planning flag
  led1.toggle(); //-- toggle the LED1 
  task1.timerSet(1000); //-- set timer (1000ms), planning flag will be set when it timeout.
}
void task2_handler(void){
  led3a.setValue(temp_i);  //-- set PWM value to led3a
  temp_i++;
 // Lcd.showFigure(1,0,3,(byte) temp_i );
  task2.timerSet(40);
}

void task3_handler(void){
  task3.clean();
  int adc = aI1.value;
  const float VREF = 5.0;
  const int   ADC_RES = 1024;

  float voltage = adc * VREF / ADC_RES;
  float compensationCoefficient = 1.0 + 0.02 * (water_temp - 25.0);
  float compensatedVoltage = voltage / compensationCoefficient;

  tds_value =
    (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage
  - 255.86 * compensatedVoltage * compensatedVoltage
  + 857.39  * compensatedVoltage) * 0.5;

  Serial.print("ADC = ");
  Serial.print(adc);
  Serial.print("  Voltage = ");
  Serial.print(voltage, 3);
  Serial.print(" V  TDS = ");
  Serial.print(tds_value, 0);
  Serial.println(" ppm");
   Lcd.displayString(2, "ppm:");   
  Lcd.showFigure(2, 5, 3,  tds_value);
    task3.timerSet(1000);
  
}

void task4_handler(void) {
  task4.clean(); // clear the planning flag

  // Control pump 1
  pump1.toggle();


  task4.timerSet(5000);    // Wait for 5 seconds before toggling again
}


void task10_handler(void) //--LED(L) on Nano
{
  task10.clean(); //-- clear the planning flag
  ledL.toggle(); //-- toggle the LED(L) 
  task10.timerSet(100); //-- set timer (1000ms), planning flag will be set when it timeout.
}

void aI7_low_handler()
{
  //-- your Exp1.6 program here
}

void aI7_high_handler()
{
  //-- your Exp1.6 program here
}

void keyI2C_handler() //-- have not completed
{
  Lcd.showFigure(1,0,3,(byte) keyI2Cdata0 );  //--line:1, poisiton:0, digit:3
}

/*  Hints for Exp-1.9, haven't completed
void task1_handler(void)  
{
  led3a.setValue(temp_i);  //-- set PWM value to led3a
  task1.timerSet(40);
}
*/

//void sw11_downHandler(void) // when SW11 Down
//{
//  task3.clean();
//  sw11.eventDown=0;   // reset sw11 press down event
//}

// --- user LCD handlers ----
void LCDshowWelcomePage(void)
{
  Lcd.displayString(0, ". CUHK-IE IERG4230 .");
  Lcd.displayString(1, ".                  .");
  Lcd.displayString(2, ".                  .");
  Lcd.displayString(3, "....................");
}

//-------------------  basic OS subroutines -------------------
//---You may need to modify them if you change the hardware ---
void hal_eventUpdate(void)
{
  //==read I2C 16key keypad
  I2CRead2Byte(keyI2CAddress, &keyI2Cdata0, &keyI2Cdata1);
  if (keyI2Cdata0 != keyI2Cdata0old) keyI2Cchanged = 1;
  if (keyI2Cdata1 != keyI2Cdata1old)  keyI2Cchanged = 1;
  keyI2Cdata0old = keyI2Cdata0;
  keyI2Cdata1old = keyI2Cdata1;
  //==end of read I2C 16key keypad
  //== comment/uncomment the followings for used hardware
 // sw11.dataUpdate();
  //sw12.dataUpdate();
  //aI0.dataUpdate();   //-- comment this line will get higher performance, Analog In takes time
  aI1.dataUpdate();   //-- comment this line will get higher performance, Analog In takes time
  //aI2.dataUpdate();   //-- comment this line will get higher performance, Analog In takes time
  //aI3.dataUpdate(); //-- comment this line will get higher performance, Analog In takes time
  //aI6.dataUpdate(); //-- comment this line will get higher performance, Analog In takes time
  //aI7.dataUpdate();     //--uncomment this line for Exp1.6
}

void timeStampUpdate(void)   // no need to modify this function unless you know what you are doing.
{
  int i;
  unsigned long temp;
  temp = millis();
  if (osEvent::osTimer > temp) i = 1;
  else i = (int)(temp - osEvent::osTimer);
  osEvent::osTimer = temp;
  //---- user add their own tasks if necessary
  task0.timerUpdate(i);
  task1.timerUpdate(i);
  task2.timerUpdate(i);
  task3.timerUpdate(i);
  task4.timerUpdate(i);
  task5.timerUpdate(i);
  task6.timerUpdate(i);
  task7.timerUpdate(i);
  task8.timerUpdate(i);
  task9.timerUpdate(i);
  task10.timerUpdate(i);  
}

void task0_handler() // Dont modify, for performace test
{ 
  task0.clean();
  #ifdef debugPerformance
    Serial.print("Performance: ");
    Serial.println(performanceCounter/3);
  #endif
  performanceCounter = 0;
  task0.timerSet(3000);
}

void printProgramMark()   //-- Serial print Program Information for identification
{
  Serial.println("");
  Serial.print("Program: ");
  Serial.println(programMark);
  Serial.print("Version: ");
  Serial.println(versionMark);  
}
//====  End of OS ========================
