// Eeprom.cpp
//
//   EEPROM storage of TC4Server configuration information.
//
//   Note that these routines use the AVR's own built in EEPROM.  They does
//   not use the external EEPROM chip on the TC4 shield.  This is because
//   it's possible to use the TC4Server without a TC4 shield.  For example,
//   if only thermistors are used for temperature sensing, then the TC4 shield
//   is superfluous.

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

#include <avr/eeprom.h>
#include "Configuration.h"
#include "Eeprom.h"
#include "EepromMap.h"
#include "Thermistor.h"
#include "CriticalSection.h"

#if 1024 <= (EEPROM_THERMISTOR + NCHAN_TH * THERMISTOR_DATA_SIZE)
#error Thermistor storage exceeds available EEPROM
#endif

void eeprom_init(void)
{
    // See if the EEPROM has been initialized
    uint8_t version[2];
    eeprom_read_block(version, (uint8_t *)EEPROM_COOKIE, 2);
    if ( version[0] == EEPROM_MAGIC_COOKIE_0 && version[1] == EEPROM_MAGIC_COOKIE_1 )
	return;

    // No, the EEPROM has not been initialized
    eeprom_set_defaults();
}


void eeprom_set_defaults(void)
{
    // Write our magic version number
    uint8_t version[2] = { EEPROM_MAGIC_COOKIE_0, EEPROM_MAGIC_COOKIE_1 };
    eeprom_write_block(version, (void *)EEPROM_COOKIE, 2);

    // And write defaults
    eeprom_reporting_period_set((uint16_t)EEPROM_REPORTING_PERIOD_DEFAULT);
    eeprom_sensors_set((uint16_t)EEPROM_INSTALLED_SENSORS_DEFAULT);
    eeprom_reporttransform_set((float)EEPROM_TEMP_OFFSET_DEFAULT,
			       (float)EEPROM_TEMP_SCALE_DEFAULT);

    // And write the thermistor defaults
    Thermistor th;
    const thermistor_data_t *params = th.get();
    for (uint8_t i = 0; i < NCHAN_TH; i++)
	eeprom_thermistor_set(i, params);
}


void eeprom_reporttransform_set(float offset, float scale)
{
    eeprom_write_block(&offset, (void *)EEPROM_TEMP_OFFSET, sizeof(float));
    eeprom_write_block(&scale,  (void *)EEPROM_TEMP_SCALE,  sizeof(float));
}


void eeprom_reporttransform_get(float *offset, float *scale)
{
    if ( offset ) eeprom_read_block(offset, (const void *)EEPROM_TEMP_OFFSET, sizeof(float));
    if ( scale  ) eeprom_read_block(scale,  (const void *)EEPROM_TEMP_SCALE,  sizeof(float));
}


uint16_t eeprom_reporting_period_get(void)
{
    return eeprom_read_word((uint16_t *)EEPROM_REPORTING_PERIOD);
}


void eeprom_reporting_period_set(uint16_t period)
{
    eeprom_write_word((uint16_t *)EEPROM_REPORTING_PERIOD, period);
}


uint16_t eeprom_sensors_get(void)
{
    return eeprom_read_word((uint16_t *)EEPROM_INSTALLED_SENSORS);
}


void eeprom_sensors_set(uint16_t sensors)
{
    eeprom_write_word((uint16_t *)EEPROM_INSTALLED_SENSORS, sensors);
}


void eeprom_thermistor_set(uint8_t id, const thermistor_data_t *params)
{
    if ( !params )
	return;

    CRITICAL_SECTION_START;
    eeprom_write_block(params,
		       (void *)(EEPROM_THERMISTOR + id * sizeof(thermistor_data_t)),
		       sizeof(thermistor_data_t));
    CRITICAL_SECTION_END;
}


void eeprom_thermistor_get(uint8_t id, thermistor_data_t *params)
{
    if ( !params )
	return;

    CRITICAL_SECTION_START;
    eeprom_read_block(params, 
		      (const void *)(EEPROM_THERMISTOR + id * sizeof(thermistor_data_t)),
		      sizeof(thermistor_data_t));
    CRITICAL_SECTION_END;
}
