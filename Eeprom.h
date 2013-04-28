// Eeprom.h
//
//   EEPROM declarations.
//
// Version date: 14 April 2013

//-------------------------------------------
// Revision history
//
// 20130417  Configurable HID report offset and scaling
// 20130414  Original code

// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2013, Dan Newman <dan.newman@mtbaldy.us>
// All rights reserved.
//
// Contributor: Ed Nisley
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of 
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list 
//   of conditions and the following disclaimer in the documentation and/or other materials 
//   provided with the distribution.
//
//   Neither the names of the authors nor contributors may  be used to endorse or promote
//   products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

#if !defined(_EEPROM_H__)

#define _EEPROM_H__

#include "Thermistor.h"

void eeprom_init(void);
void eeprom_set_defaults(void);
uint16_t eeprom_reporting_period_get(void);
void eeprom_reporting_period_set(uint16_t period);
uint16_t eeprom_sensors_get(void);
void eeprom_sensors_set(uint16_t sensors);
void eeprom_thermistor_get(uint8_t id, thermistor_data_t *params);
void eeprom_thermistor_set(uint8_t id, const thermistor_data_t *params);
void eeprom_reporttransform_set(float offset, float scale);
void eeprom_reporttransform_get(float *offset, float *scale);

#endif
