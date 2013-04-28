// Thermistor.cpp

//-------------------------------------------
// Revision history
//
// 20130414  Add EEPROM support
// 20130407  Address PROGMEM warnings
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

#include <stdint.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Configuration.h"
#include "Thermistor.h"
#include "Utils.h"

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#endif
#define PROGMEM __attribute__((section(".progmem.data")))

Thermistor::Thermistor(uint32_t adc_max, int8_t t0, uint32_t r0, uint16_t beta, uint32_t r1,
		       uint32_t r2)
{
    set(adc_max, t0, r0, beta, r1, r2);
}

Thermistor::Thermistor()
{
    set((uint32_t)(THERM_ADC_MAX), (int)(THERM_T0), (uint32_t)(THERM_R0),
	(uint16_t)(THERM_BETA), (uint32_t)(THERM_R1), (uint32_t)(THERM_R2));
}

void Thermistor::set(const thermistor_data_t *params)
{
    if ( !params )
	return;
    set(params->adc_max, params->t0, params->r0, params->beta, params->r1, params->r2);
}

void Thermistor::set(uint32_t adc_max, int8_t t0, uint32_t r0, uint16_t beta, uint32_t r1,
		     uint32_t r2)
{
    this->params.adc_max = adc_max;
    this->params.t0      = t0;
    this->params.r0      = r0;
    this->params.beta    = beta;
    this->params.r1      = r1;
    this->params.r2      = r2;
    this->k              = (double)r0 * exp( -(double)beta / ( (double)t0 + (double)273.15 ) );

    if ( r1 > 0 )
    {
	// Effective bias voltage
	this->vs = (double)r1 * (double)(THERM_VCC) / (double)( r1 + r2 );

        // Effective bias impedance
	this->rs = (double)( r1 * r2 ) / (double)( r1 + r2 );
    }
    else
    {
	this->vs = (double)(THERM_VCC);
	this->rs = (double)r2;
    }
}

void Thermistor::show(uint8_t port)
{
    static PROGMEM prog_char str1[] = "+OK THERMISTOR ";
    static PROGMEM prog_char str2[] = " nc ";

    write_pgmspace(str1);
    Serial.print(port);
    Serial.write(' ');
    Serial.print(params.adc_max);
    Serial.write(' ');
    Serial.print(params.t0);
    Serial.write(' ');
    Serial.print(params.r0);
    Serial.write(' ');
    Serial.print(params.beta);
    if ( params.r1 ) {
	Serial.write(' ');
	Serial.print(params.r1);
	Serial.write(' ');
    }
    else
	write_pgmspace(str2);
    Serial.print(params.r2);
    write_pgmspace(crlf_text);
}

float Thermistor::tempC(uint32_t adc)
{
    // Convert ADC value to voltage
    double v = (double)adc * (double)(THERM_VADC) / (double)( params.adc_max + 1 );

    // Resistance of the thermistor
    double r = rs * v / ( vs - v );

    // Temperature in degrees Celsius
    return (float)(( (double)params.beta / log( r / k ) ) - 273.15f);
}

