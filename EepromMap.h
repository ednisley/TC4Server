// EepromMap.h
//
//   EEPROM offsets and default values.  Private declarations
//   used by Eeprom.cpp.
//
//   Note that the ATmega 32u4 (aka, Arduino Leonardo) only has 1K of EEPROM
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

#if !defined(_EEPROMMAP_H__)

#define _EEPROMMAP_H__

#include <stdint.h>
#include "Configuration.h"

// uint16_t
// Magic cookie stored in EEPROM to give us an indication
// that the EEPROM has been initialized or not.

const static uint16_t EEPROM_COOKIE = 0x0001;
#define EEPROM_MAGIC_COOKIE_0 0xA5
#define EEPROM_MAGIC_COOKIE_1 0x5B


// uint16_t
// Bit mask of which sensors are installed
// Only need 8 bits, buts lets use 16 for now

const static uint16_t EEPROM_INSTALLED_SENSORS = 0x0004;
#define EEPROM_INSTALLED_SENSORS_DEFAULT 0xFFFF

// uint16_t
// HID report period in milliseconds
// Upper limit, 0xFFFF is 65.535 seconds so a minute just squeezes in

const static uint16_t EEPROM_REPORTING_PERIOD = 0x0006;
#define EEPROM_REPORTING_PERIOD_DEFAULT 5000

// 2 x float
// HID report offset & scaling
// HID report = ( temp + offset ) * scale

const static uint16_t EEPROM_TEMP_OFFSET = 0x0008;
const static uint16_t EEPROM_TEMP_SCALE  = 0x0012;
#define EEPROM_TEMP_OFFSET_DEFAULT  ( TEMP_OFFSET_DEFAULT )
#define EEPROM_TEMP_SCALE_DEFAULT   ( TEMP_SCALE_DEFAULT )

// ------- EVERYTHING PAST THIS POINT IS THERMISTOR DATA -------

// NCHAN_TC occurrences of thermistor_data_t

const static uint16_t EEPROM_THERMISTOR = 0x0100;

#endif
