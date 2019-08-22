/***************************************************************************
* File Name: PlayingWithFusion_MAX31856.h
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
*
* Playing With Fusion, Inc. invests time and resources developing open-source
* code. Please support Playing With Fusion and continued open-source
* development by buying products from Playing With Fusion!
*
* **************************************************************************/

#ifndef EVS_MAX31856_h
#define EVS_MAX31856_h

#include "Arduino.h"			// use "WProgram.h" for IDE <1.0
#include "avr/pgmspace.h"
#include "util/delay.h"
#include "stdlib.h"
#include "SPI.h"


// Registers
#define REG_CR0 	0x00	// Config Reg 0 - See Datasheet, pg 19
#define REG_CR1 	0x01	// Config Reg 1 - averaging and TC type
#define REG_MASK	0x02	// Fault mask register (for fault pin)
#define REG_CJHF	0x03	// Cold Jcn high fault threshold, 1 degC/bit
#define REG_CJLF	0x04	// Cold Jcn low fault threshold, 1 degC/bit
#define REG_LTHFTH	0x05	// TC temp high fault threshold, MSB, 0.0625 degC/bit
#define REG_LTHFTL	0x06	// TC temp high fault threshold, LSB
#define REG_LTLFTH	0x07	// TC temp low fault threshold, MSB, 0.0625 degC/bit
#define REG_LTLFTL	0x08	// TC temp low fault threshold, LSB
#define REG_CJTO	0x09	// Cold Jcn Temp Offset Reg, 0.0625 degC/bit
#define REG_CJTH	0x0A	// Cold Jcn Temp Reg, MSB, 0.015625 deg C/bit (2^-6)
#define REG_CJTL	0x0B	// Cold Jcn Temp Reg, LSB
#define REG_LTCBH	0x0C	// Linearized TC Temp, Byte 2, 0.0078125 decC/bit
#define REG_LTCBM	0x0D	// Linearized TC Temp, Byte 1
#define REG_LTCBL	0x0E	// Linearized TC Temp, Byte 0
#define REG_SR		0x0F	// Status Register

// CR0 Configs
#define CMODE_OFF		0x00
#define	CMODE_AUTO		0x80
#define ONESHOT_OFF		0x00
#define ONESHOT_ON		0x40
#define OCFAULT_OFF		0x00
#define OCFAULT_10MS	0x10
#define OCFAULT_32MS	0x20
#define OCFAULT_100MS	0x30
#define CJ_ENABLED		0x00
#define CJ_DISABLED		0x08
#define FAULT_AUTO		0x00
#define FAULT_MANUAL	0x04
#define FAULT_CLR_DEF	0x00
#define FAULT_CLR_ALT	0x02
#define CUTOFF_60HZ		0x00
#define CUTOFF_50HZ		0x01

// CR1 Configs
#define AVG_SEL_1SAMP	0x00
#define AVG_SEL_2SAMP	0x10
#define AVG_SEL_4SAMP	0x20
#define AVG_SEL_8SAMP	0x30
#define AVG_SEL_16SAMP	0x40
#define B_TYPE			0x00
#define E_TYPE			0x01
#define J_TYPE			0x02
#define K_TYPE			0x03
#define N_TYPE			0x04
#define R_TYPE			0x05
#define S_TYPE			0x06
#define T_TYPE			0x07
#define V_MODE_GAIN_8   0x08
#define V_MODE_GAIN_32  0x0C

// MASK Configs
#define CJ_HIGH_MASK	0x20
#define CJ_LOW_MASK		0x10
#define TC_HIGH_MASK	0x08
#define TC_LOW_MASK		0x04
#define OV_UV_MASK		0x02
#define OPEN_FAULT_MASK	0x01

typedef struct {
	long lValue;
	long lCJValue;
    float  fValue;	// linearized TC temperature, 0.0078125 decC/bit (2^-7)
    float  fCJValue;	// temp of chip ref jcn, 0.015625 deg C/bit (2^-6)
    uint8_t  status;		// TC status - valid/invalid + fault reason
} MAX31856_data_struct;

class EVS_MAX31856
{
 public:
  EVS_MAX31856(void);
  EVS_MAX31856(int8_t CSx, int8_t FAULTx, int8_t DRDYx);
  void config_pins(int8_t CSx, int8_t FAULTx, int8_t DRDYx);
  void begin(void);

  void MAX31856_config(uint8_t TC_TYPE, uint8_t AVG_MODE, uint8_t CR0_VALUE = 
                        (CMODE_AUTO | ONESHOT_OFF | OCFAULT_10MS | CJ_ENABLED | FAULT_AUTO | FAULT_CLR_DEF | CUTOFF_50HZ));

  void MAX31856_CJ_offset(int8_t offset_val);	// offset is 2^-4 degC/bit
  void SPIbus_Init(void);
  int MAX31856_update(MAX31856_data_struct *tc_ptr, bool isCJread);
  bool getDRDY(void);
  uint8_t getTCType(void);
  float convertResult_CJ(long CJResult);
  float convertResult_TC(long TCResult);
 private:
  int8_t _cs, _drdy, _fault;
  uint8_t _tc_type;
  uint8_t _sing_reg_read(uint8_t RegAdd);
  void _sing_reg_write(uint8_t RegAdd, uint8_t BitMask, uint8_t RegData);
};

#endif