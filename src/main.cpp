

//baswi interrupt settings BEGIN
//Enable / Disable an interrupt source on INT1
/*#define LIS2DH_CTRL_REG3    0x22
bool LIS2DH::enableInterruptInt1(uint8_t _int) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG3,_int,true);
}

//Enable / Disable an interrupt source on INT2
 bool LIS2DH::enableInterruptInt2(uint8_t _int) {
  return this->writeMaskedRegister8(LIS2DH_CTRL_REG6,_int,true);
}*/
//baswi interrupt settings END

/*
  Reading and controlling the very low power LIS2DH12
  Author: Nathan Seidle
  Created: Septempter 18th, 2019
  License: This code is Lemonadeware; do whatever you want with this code.
  If you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  This example demonstrates how to read XYZ from the LIS2DH12.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  Edge: https://www.sparkfun.com/products/15170
  Edge 2: https://www.sparkfun.com/products/15420
  Qwiic LIS2DH12 Breakout: https://www.sparkfun.com/products/15760

  Hardware Connections:
  Plug a Qwiic cable into the Qwiic Accelerometer RedBoard Qwiic or BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

//*** serial debug***
// Example from ladyada.net/learn/arduino/lesson4.html
int a = 5;
int b = 10;
int c = 20;

// SerialDebug Library

// Disable all debug ? Good to release builds (production)
// as nothing of SerialDebug is compiled, zero overhead :-)
// For it just uncomment the DEBUG_DISABLED
//#define DEBUG_DISABLED true

// Disable SerialDebug debugger ? No more commands and features as functions and globals
// Uncomment this to disable it 
//#define DEBUG_DISABLE_DEBUGGER true

// Define the initial debug level here (uncomment to do it)
#define DEBUG_INITIAL_LEVEL DEBUG_LEVEL_VERBOSE

// Disable auto function name (good if your debug yet contains it)
//#define DEBUG_AUTO_FUNC_DISABLED true

// Include SerialDebug
#include "SerialDebug.h" // Download SerialDebug library: https://github.com/JoaoLopesF/SerialDebug
#include "mbed.h"


//#define Serial SerialUSB
#include <Wire.h>

#include "SparkFun_LIS2DH12.h" //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
SPARKFUN_LIS2DH12 accel;       //Create instance
//#include "lis2dh12.h"
//#include "lis2dh12_registers.h"

mbed::InterruptIn event1(p11); //p0.11 = ACCEL_INT1 //p11: accel.available -> INTR
mbed::InterruptIn event2(p15); //p0.15 = ACCEL_INT2 //p15: geeft soms bij hele wilde bewegingen INTR


volatile bool intr1_recvd = false;
volatile bool intr2_recvd = false;
void intr1_test() {
  intr1_recvd = true;
}
void intr2_test() {
  intr2_recvd = true;
}


void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

#ifndef DEBUG_DISABLE_DEBUGGER
  // Add Functions and global variables to SerialDebug

  // Add functions that can called from SerialDebug

  //debugAddFunctionVoid(F("function"), &function); // Example for function without args
  //debugAddFunctionStr(F("function"), &function); // Example for function with one String arg
  //debugAddFunctionInt(F("function"), &function); // Example for function with one int arg

  // Add global variables that can showed/changed from SerialDebug
  // Note: Only global, if pass local for SerialDebug, can be dangerous

//  debugAddGlobalInt(F("a"), &a);
//  debugAddGlobalInt(F("b"), &b);
//  debugAddGlobalInt(F("c"), &c);
#endif // DEBUG_DISABLE_DEBUGGER

  printlnA("LIS2 example");

  Wire.begin();
  delay(500);
  
  //NRF_GPIO->PIN_CNF[ACCEL_INT1] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);

/* disable Arduino method
  pinMode (ACCEL_INT1, INPUT_PULLDOWN);
  attachInterrupt ( digitalPinToInterrupt (ACCEL_INT1), intr1_test, CHANGE ) ;

  pinMode (ACCEL_INT2, INPUT_PULLDOWN);
  attachInterrupt ( digitalPinToInterrupt (ACCEL_INT2), intr2_test, CHANGE ) ; */

//use mebed style
  event1.rise(&intr1_test); //werkt;
  event2.rise(&intr2_test);

  if (accel.begin() == false)
  {
    Serial.println("Accelerometer not detected. Check address jumper and wiring. Freezing...");
    while (100)
      ;
  }
  printlnA("Accelerometer detected"); 

  //accel.enableTapDetection(); //must be AFTER accel.begin(); otherwise USB port freezess

//  lis2dh12_init(); //to make sure SPI is initialized and communicatates properly with LIS2DH12.
//  lis2dh12_reset() //to reset the memory contents
//  nrf_delay_ms(10); //* wait for 10 ms, i.e. `
//  lis2dh12_enable(); // * Enable X-Y-Z axes by
// * Initialize pin interrupts if your program is interrupt-driven, see nrf_nordic_pininterrupt for details.
// * Do not read sensor in interrupt context, you should rather schedule sensor read and let app process the request
}

void loop()
{
    // SerialDebug handle
  // Notes: if in inactive mode (until receive anything from serial),
  // it show only messages of always or errors level type
  // And the overhead during inactive mode is very low
  // Only if not DEBUG_DISABLED
  debugHandle();
  if (intr1_recvd == true) {
    Serial.print("INTER 1 RECEIVED\n");
    intr1_recvd = false;
  }
  if (intr2_recvd == true) {
    Serial.print("INTER 2 RECEIVED\n");
    intr2_recvd = false;
  }

  //Print accel values only if new data is available
/*  if (accel.available())
  {
    float accelX = accel.getX(); // this generates interrupts regulary
    float accelY = accel.getY();
    float accelZ = accel.getZ();
  //  float tempC = accel.getTemperature();

    printlnA("Acc [mg]: ");
    printlnA(accelX, 1);
    Serial.print(" x, ");
    Serial.print(accelY, 1);
    Serial.print(" y, ");
    Serial.print(accelZ, 1);
    Serial.print(" z, ");
    //Serial.print(tempC, 1);
    //Serial.print("C");
    Serial.println(); 
  } */
  //Serial.print("ff wachten\r\n");
  
  //delay(3000); 
}



