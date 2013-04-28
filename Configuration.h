// Configuration.h

//-------------------------------------------
// Revision history
//
// 20130417  Configurable HID report offset and scaling
// 20130315  Updated for HID support for Leonardo
// 20130119  Original code

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

#if !defined(_CONFIGURATION_H__)

#define _CONFIGURATION_H__

#include <avr/io.h>

#if defined(USBCON) && defined(__AVR_ATmega32U4__)
#define USE_HID
#endif

#define VERSION_MAJOR "0"
#define VERSION_MINOR "7"

// Comms speed
#define DEFAULT_BAUD 57600

// Pin assignments

#define NCHAN_TC       4 // number of thermocouple input channels
// Thermocouple type / library
#define TC_TYPE typeK
//#define TC_TYPE typeJ
//#define TC_TYPE typeT

// Note that Analog in 4 & 5 (ATmega SDA & SCL) are not available
//   as the TC4 uses them for serial comms with the external a/d,
//   temp sensor, and EEPROM chips.

// Pin numbers are as per Arduino's analog pin numbering system

#define NCHAN_TH       4 // Number of thermistors
                         //   TC4 label; Arduino pin [ ATmega 168, 328 pin ]
#define PIN_THERM_1    0 // AIN-0 / AN0; Analog in 0 [ PC0 / ADC0 / PCINT8 ]
#define PIN_THERM_2    1 // AIN-1 / AN1; Analog in 1 [ PC1 / ADC1 / PCINT9 ]
#define PIN_THERM_3    2 // AIN-2;       Analog in 2 [ PC2 / ADC2 / PCINT10 ]
#define PIN_THERM_4    3 // AIN-3;       Analog in 3 [ PC3 / ADC3 / PCINT11 ]

// Thermistor defaults
#define THERM_VCC       5.0     // Supply voltage, in Volts
#define THERM_VADC      5.0     // A/D reference voltage, in Volts
#define THERM_ADC_MAX   1023    // A/D max value
#define THERM_T0        25      // Temperature in degrees C
#define THERM_R0        100000  // Resistance in Ohms of thermistor @ Temp THERM_T0
#define THERM_BETA      4190    // Beta(25/85) or Beta(25/100)
#define THERM_R1        0       // Resistance of R1 in Ohms
#define THERM_R2        4700    // Resistance of R2 in Ohms

#define NCHAN_SSR      2 // Two SSR outputs
                         //                TC4 label; Arduino pin        [ ATmega 168, 328 pin ]
#define PIN_SSR_1      9 // OT1- controlled by  DIO9; Digital I/O pin  9 [ PB1 / OC1A / PCINT1 ]
#define PIN_SSR_2     10 // OT2- controlled by DIO10; Digital I/O pin 10 [ PB2 / SS / OC1B / PCINT2 ]

#define NCHAN_PWM      1 // One PWM output
#define PIN_PWM_1      3 // IO3+ controlled by DIO3; Digital I/O pin 3 [ PD3 / PCINT19 / INT1 / OC2B ]
#define PWM_RES      255 // In general, we just get 490 Hz and 0 - 255 as our choices

// Note that IO2+ which is controlled by DIO2; Digital I/O pin 2 [ PD2 / PCINT18 / INT0 ]
// is presently not controlled by the server

// Default values for systems without calibration values stored in EEPROM
// Best to simply leaves these alone.  Unlike other TC4 programs, this code
// does not store calibrations in EEPROM.  Instead, that is left to the client
// program to deal with.

#define CAL_GAIN      1.00f
#define UV_OFFSET        0
#define AMB_OFFSET     0.0f

// When converting temperatures from a floating point degrees Celsius to a
// a 16 bit integer for a HID report, we apply and offset and scaling factor
//
// HID report value = ( temperature + offset ) * scale

#define TEMP_OFFSET_DEFAULT 273.2
#define TEMP_SCALE_DEFAULT   10.0

// Undefine to enable SRAM checking
#define STACK_PAINT

#endif
