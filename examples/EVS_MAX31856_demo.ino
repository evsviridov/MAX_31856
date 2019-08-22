/***************************************************************************
  File Name: SEN30006_MAX31856_example.ino
  Processor/Platform: Arduino Uno R3 (tested)
  Development Environment: Arduino 1.6.1

  Designed for use with with Playing With Fusion MAX31856 thermocouple
  breakout boards: SEN-30007 (any TC type) or SEN-30008 (any TC type)

  Copyright © 2015 Playing With Fusion, Inc.
  SOFTWARE LICENSE AGREEMENT: This code is released under the MIT License.

  Permission is hereby granted, free of charge, to any person obtaining a
  copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.
* **************************************************************************
  REVISION HISTORY:
  Author		Date	    Comments
  J. Steinlage		2015Dec30   Baseline Rev, first production support

  Playing With Fusion, Inc. invests time and resources developing open-source
  code. Please support Playing With Fusion and continued open-source
  development by buying products from Playing With Fusion!
* **************************************************************************
  ADDITIONAL NOTES:
  This file contains functions to initialize and run an Arduino Uno R3 in
  order to communicate with a MAX31856 single channel thermocouple breakout
  board. Funcionality is as described below:
	- Configure Arduino to broadcast results via UART
        - call PWF library to configure and read MAX31856 IC (SEN-30005, any type)
	- Broadcast results to COM port
   Circuit:
     Arduino Uno   Arduino Mega  -->  SEN-30006
     DIO pin 10      DIO pin 10  -->  CS0
     DIO pin  9      DIO pin  9  -->  CS1
     DIO pin  8      DIO pin  8  -->  CS2
     DIO pin  7      DIO pin  7  -->  CS3
     DIO pin  6      DIO pin  6  -->  DR0 (Data Ready... not used in example, but routed)
     DIO pin  5      DIO pin  5  -->  DR1 (Data Ready... not used in example, but routed)
     DIO pin  4      DIO pin  4  -->  DR2 (Data Ready... not used in example, but routed)
     DIO pin  3      DIO pin  3  -->  DR3 (Data Ready... not used in example, but routed)
     MOSI: pin 11  MOSI: pin 51  -->  SDI (must not be changed for hardware SPI)
     MISO: pin 12  MISO: pin 50  -->  SDO (must not be changed for hardware SPI)
     SCK:  pin 13  SCK:  pin 52  -->  SCLK (must not be changed for hardware SPI)
     D03           ''            -->  FAULT (not used in example, pin broken out for dev)
     D02           ''            -->  DRDY (not used in example, only used in single-shot mode)
     GND           GND           -->  GND
     5V            5V            -->  Vin (supply with same voltage as Arduino I/O, 5V)
      NOT CONNECTED              --> 3.3V (this is 3.3V output from on-board LDO. DO NOT POWER THIS PIN!
  It is worth noting that the 0-ohm resistors on the PCB can be removed to
  free-up DIO pins for use with other shields if the 'Data Ready' funcionality
  isn't being used.
***************************************************************************/
#include "EVS_MAX31856.h"
#include "SPI.h"

#ifdef USE_LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#endif

uint8_t TC0_CS  =  7;
uint8_t TC0_DRDY  =  3;

uint8_t TC1_CS  =  8;
uint8_t TC1_DRDY  =  4;

uint8_t TC2_CS  =  9;
uint8_t TC2_DRDY  =  5;

uint8_t TC3_CS  =  10;
uint8_t TC3_DRDY  =  6;

uint8_t TC0_FAULT = 2;                     // not used in this example, but needed for config setup

EVS_MAX31856  thermocouple0(TC0_CS, TC0_FAULT, TC0_DRDY);
EVS_MAX31856  thermocouple1(TC1_CS, TC0_FAULT, TC1_DRDY);
EVS_MAX31856  thermocouple2(TC2_CS, TC0_FAULT, TC2_DRDY);
EVS_MAX31856  thermocouple3(TC3_CS, TC0_FAULT, TC3_DRDY);

EVS_MAX31856 *thermocouple[4];

struct var_max31856 tc_ptr[4];

void setup()
{
  delay(1000);                            // give chip a chance to stabilize
  Serial.begin(115200);                   // set baudrate of serial port
  Serial.println("Playing With Fusion: MAX31856, SEN-30007/8");
#ifdef USE_LCD
  lcd.begin();
  lcd.backlight();
#endif
  // setup for the the SPI library:
  SPI.begin();                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
  SPI.setDataMode(SPI_MODE3);             // MAX31856 is a MODE3 device
  thermocouple[0] = &thermocouple0;
  thermocouple[1] = &thermocouple1;
  thermocouple[2] = &thermocouple2;
  thermocouple[3] = &thermocouple3;
  for (int i = 0; i < 4; ++i)
  {
    thermocouple[i]->begin();
    thermocouple[i]->MAX31856_config(V_MODE_GAIN_32, AVG_SEL_1SAMP, (CMODE_AUTO | ONESHOT_OFF | OCFAULT_10MS | CJ_ENABLED | FAULT_AUTO | FAULT_CLR_DEF | CUTOFF_50HZ));
  }
}

void loop()
{
  static long newTimer, oldTimer;
  double tmp;
  bool isReady;
  char str[21];
  //  delay(10);
  do
  {
    isReady = 1;
    for (int i = 0; i < 4; ++i)
    {
      if (thermocouple[i]->getDRDY())
      {
        isReady = 0;
        break;
      }
    }
  } while (!isReady);
  //  Serial.println(isReady,BIN);
  for (int i = 0; i < 4; ++i)
    thermocouple[i]->MAX31856_update(&tc_ptr[i], true);

  //  Serial.print("--------------------------------------");            // Print TC0 header
  for ( int i = 0; i < 4; ++i)
  {
    tmp = tc_ptr[i].fValue;
    Serial.print(tmp, 3);
    Serial.print("  cj=");
    tmp = tc_ptr[i].fCJValue;
    Serial.print(tmp, 1);
    Serial.print(" C \t");

#ifdef USE_LCD
    lcd.setCursor(0, i);
    lcd.print((String)"U" + i + " ");
    lcd.print(tmp < 0 ? "-" : " ");
    lcd.print(abs(tmp), 3); lcd.print(" Tx ");
    tmp = tc_ptr[i].fCJValue;
    lcd.print(tmp, 1);
    lcd.print(" C ");
#endif
  }
  do
  {
    newTimer = micros();
  } while (0 && ((newTimer - oldTimer) < 1000000L));

  Serial.print("dT (msec)=");
  Serial.print((newTimer - oldTimer) / 1000);
  Serial.print("\tF(Hz)=");
  Serial.print(1000000.0 / (double)(newTimer - oldTimer));
  Serial.println();
  oldTimer = newTimer;

}
