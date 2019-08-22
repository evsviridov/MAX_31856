/***************************************************************************
* File Name: PlayingWithFusion_MAX31856.cpp
* Processor/Platform: Arduino Uno R3 (tested)
* Development Environment: Arduino 1.6.1
*
* Designed for use with with Playing With Fusion MAX31856 thermocouple
* breakout boards: SEN-30005, SEN-30006 (any TC type)
*
* Copyright © 2015 Playing With Fusion, Inc.
* SOFTWARE LICENSE AGREEMENT: This code is released under the MIT License.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* **************************************************************************
* REVISION HISTORY:
* Author			Date		Comments
* J. Steinlage		2015Aug10   First rev
* E.Sviridov        2017Oct04   Error correction in .h (averaging parameters)
*
* Playing With Fusion, Inc. invests time and resources developing open-source
* code. Please support Playing With Fusion and continued open-source
* development by buying products from Playing With Fusion!
* **************************************************************************
* ADDITIONAL NOTES:
* This file contains functions to initialize and run an Arduino Uno R3 in
* order to communicate with a MAX31856 single channel thermocouple breakout
* board. Funcionality is as described below:
*	- Initialize TC channel
*	- Read MAX31856 registers from Playing With Fusion SEN-30005 (any type)
*	- Properly unpack data into internal temp, TC temp and status variables
***************************************************************************/
#include "EVS_MAX31856.h"

EVS_MAX31856::EVS_MAX31856(void)
{
}

EVS_MAX31856::EVS_MAX31856(int8_t CSx, int8_t FAULTx, int8_t DRDYx)
{
  // Function to initialize thermocouple channel, load private variables
  config_pins( CSx,  FAULTx,  DRDYx);
}

void EVS_MAX31856::config_pins(int8_t CSx, int8_t FAULTx, int8_t DRDYx)
{
  // Function to initialize thermocouple channel, load private variables
  _cs = CSx;
  _fault = FAULTx;
  _drdy = DRDYx;
}

void EVS_MAX31856::begin(void)
{
  if(_cs >=0) pinMode(_cs, OUTPUT);
  if(_fault >=0) pinMode(_fault, INPUT);
  if(_drdy >=0) pinMode(_drdy, INPUT);
  
  // immediately pull CS pin high to avoid conflicts on SPI bus
  digitalWrite(_cs, HIGH);
}

uint8_t EVS_MAX31856::_sing_reg_read(uint8_t RegAdd)
{
	digitalWrite(_cs, LOW);						// set pin low to start talking to IC
	// next pack address byte
	// bits 7:4 are 0 for read, register is in bits 3:0... format 0Xh
	SPI.transfer((RegAdd & 0x0F));				// write address
	// then read register data
	uint8_t RegData = SPI.transfer(0x00); 		// read register data from IC
	digitalWrite(_cs, HIGH);					// set pin high to end SPI session
	
	return RegData;
}

void EVS_MAX31856::_sing_reg_write(uint8_t RegAdd, uint8_t BitMask, uint8_t RegData)
{
	// start by reading original register data (we're only modifying what we need to)
	uint8_t OrigRegData = _sing_reg_read(RegAdd);

	// calculate new register data... 'delete' old targeted data, replace with new data
	// note: 'BitMask' must be bits targeted for replacement
	// add'l note: this function does NOT shift values into the proper place... they need to be there already
	uint8_t NewRegData = ((OrigRegData & ~BitMask) | (RegData & BitMask));

	// now configure and write the updated register value
	digitalWrite(_cs, LOW);							// set pin low to start talking to IC
	// next pack address byte
	// bits 7:4 are 1000b for read, register is in bits 3:0... format 8Xh
	SPI.transfer((RegAdd & 0x0F) | 0x80);			// simple write, nothing to read back
	SPI.transfer(RegData); 							// write register data to IC
	digitalWrite(_cs, HIGH);						// set pin high to end SPI session
}

uint8_t EVS_MAX31856::getTCType(void)
{
	return _tc_type;
}

float EVS_MAX31856::convertResult_CJ(long CJResult) // Celsius
{	
	return (float)CJResult * 0.015625;
}

float EVS_MAX31856::convertResult_TC(long TCResult) // Celsius or mV
{	
	switch (_tc_type)
	{
		case V_MODE_GAIN_8:
			return (float)TCResult / (float)1677.7216; // to mVolts
			// 10xx = Voltage Mode, Gain = 8.  Code = 8 x 1.6 x 2^17 x VIN           8*1.6*2^17 = 1677721.6
			break;
		case V_MODE_GAIN_32:
			return (float)TCResult / (float)6710.8864; // to mVolts
			// 11xx = Voltage Mode, Gain = 32. Code = 32 x 1.6 x 217 x VIN         32*1.6*2^17 = 6710886.4
			break;
		default: 
			return (float)TCResult * 0.0078125;
			break;
	}
}

void EVS_MAX31856::MAX31856_config(uint8_t TC_TYPE, uint8_t AVG_MODE, uint8_t CR0_VALUE)
{
	uint8_t regdat = 0;		// set up paramater to compile register configs
	
	// set CR0 (REG_CR0)
//	regdat = (CMODE_AUTO | ONESHOT_OFF | OCFAULT_10MS | CJ_ENABLED | FAULT_AUTO | FAULT_CLR_DEF | FILT_FREQ);
	regdat = CR0_VALUE;
	_sing_reg_write(REG_CR0, 0xFF, regdat);	// write data to register
/*	CRO, 00h/80h:[7] cmode (0=off (default), 1=auto conv mode)
		[6] 1shot (0=off, default)
		[5:4] OCFAULT (table 4 in datasheet)
		[3] CJ disable (0=cold junction enabled by default, 1=CJ disabled, used to write CJ temp)
		[2] FAULT mode (0=sets, clears automatically, 1=manually cleared, sets automatically)
		[1] FAULTCLR   (0 - default, 1=see datasheet)
		[0] 50/60Hz (0=60hz (default), 1=50Hz filtering) + harmonics */
	_tc_type = TC_TYPE;
	// set CR1 (REG_CR1)
	regdat = (AVG_MODE | _tc_type);
	_sing_reg_write(REG_CR1, 0xFF, regdat);
/*	CR1, 01h/81h:[7] reserved
		[6:4] AVGSEL (0=1samp(default),1=2samp,2=4samp,3=8samp,0b1xx=16samp])
		[3:0] TC type (0=B, 1=E, 2=J, 3=K(default), 4=N, 5=R, 6=S, 7=T, others, see datasheet)*/
	
	// set MASK (REG_MASK) - PWF default masks all but OV/UV and OPEN from lighting LED
	regdat = (CJ_HIGH_MASK | CJ_LOW_MASK | TC_HIGH_MASK | TC_LOW_MASK); 
	_sing_reg_write(REG_MASK, 0x3F, regdat);
/*	MASK, 02h/82h: This register masks faults from causing the FAULT output from asserting,
				   but fault bits will still be set in the FSR (0x0F)
		           All faults are masked by default... must turn them on if desired
		[7:6] reserved
		[5] CJ high fault mask
		[4] CJ low fault mask
		[3] TC high fault mask
		[2] TC low fault mask
		[1] OV/UV fault mask
		[0] Open fault mask
		PWF example: 0x03 (OV/UV + open) */
	
	// LEAVE CJHFT/CJLFT AT DEFAULT VALUES FOR PWF EXAMPLE
	// note: these values would potentially be used to indicate material or component  
	//       limits have been exceeded for your specific measurement configuration
/*	CJHFT, 03h/83h: cold-jcn high fault threshold, default 0x7F (bit 7 is sign)
	CJLFT, 04h/84h: cold-jcn low fault threshold, default 0x00) */

	// LEAVE LTXFTX AT DEFAULT VALUES FOR PWF EXAMPLE
	// note: these values would potentially be used to indicate material limits 
	//       have been exceeded for your specific thermocouple
/*	LTHFTH, 05h/85h: Linearize temperature high fault thresh MSB (bit 7 is sign)
	LTHFTL, 06h/86h: Linearize temperature high fault thresh LSB
	LTLFTH, 07h/87h: Linearize temperature low fault thresh MSB (bit 7 is sign)
	LTLFTL, 08h/88h: Linearize temperature low fault thresh LSB */
}

int EVS_MAX31856::MAX31856_update(MAX31856_data_struct *tc_ptr, bool isCJread)
{
	// Start by reading SR for any faults, exit if faults present, though some
	// faults could potentially be dealt with
	uint8_t fault_status = _sing_reg_read(REG_SR);
	tc_ptr->status = fault_status;
	if(0 != fault_status)
	{
		// fault present, exit update
		// return -1;
	}// else no fault, keep reading
	if(isCJread)
	{
		// Read Cold Jcn temperature (2 registers)
		int16_t cj_temp =  (_sing_reg_read(REG_CJTH)<<8); // MSB, left shift 8
		cj_temp |= _sing_reg_read(REG_CJTL);			  // LSB read
		// now save sign, shift right 2 to align to type (see datasheet, pg 24 for reg packing)
		if(cj_temp & 0x8000)
		{
			cj_temp = (0xE000 | ((cj_temp & 0x7FFF)>>2));
		}
		else
		{
			cj_temp = (cj_temp & 0x7FFF)>>2;
		}
		tc_ptr->lCJValue=cj_temp;
		tc_ptr->fCJValue = convertResult_CJ(tc_ptr->lCJValue);					  // store result in struct
	}
	// Read Linearized TC temperature (3 registers)
	int32_t tc_temp = (_sing_reg_read(REG_LTCBH)<<8); 	// HSB, left shift 8
	tc_temp |= _sing_reg_read(REG_LTCBM);		  	  	// MSB
	tc_temp <<= 8;									  	// left shift 8
	tc_temp |= _sing_reg_read(REG_LTCBL);				// LSB
	// now save sign, shift to align to type (see datasheet, pg 25 for reg packing)
	if(tc_temp & 0x800000)
	{
		tc_temp = 0xFFFC0000 | ((tc_temp & 0x7FFFFF)>>5);
	}
	else
	{
		tc_temp = (tc_temp & 0x7FFFFF)>>5;
	}
	tc_ptr->lValue = tc_temp;
	tc_ptr->fValue = convertResult_TC(tc_ptr->lValue);
	
	return fault_status;
}
bool EVS_MAX31856::getDRDY(void)
{
	bool is_ready;
	if(_drdy >=0 )
		return digitalRead(_drdy);
	else
		return 0;
}

void EVS_MAX31856::MAX31856_CJ_offset(int8_t offset_val)	// offset is 2^-4 degC/bit
{
	/*	CJTO, 09h/89h: Cold Junction Temperature Offset (int8_t, default 0x00) */
	// This function could be used to add a temperature offset based on a known difference
	//      between the chip temp and the location of the TC metal to copper transition
	
	// might need to write special handling for the signedness of the offset_val...
	_sing_reg_write(REG_CJTO, 0xFF, (uint8_t) offset_val);
}

