// HidTempSensor.cpp

//-------------------------------------------
// Revision history
//
// 20130417  Configurable HID report offset and scaling
// 20130414  Added EEPROM support
// 20130315  Original code

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

#include "Configuration.h"
#ifdef USE_HID

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include "Configuration.h"
#include "HidTempSensor.h"
#include "HID_TC4.h"
#include "Server.h"
#include "thermocouple.h"
#include "Eeprom.h"

#if defined(__AVR_ATmega32U4__)
#include <avr/pgmspace.h>
#include "pins_arduino.h"
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

static float read_temp(uint8_t id);
static uint16_t read_start(uint8_t id);
static uint16_t read_adc(void);
static void read_adc_start(uint8_t pin);

#define DEBUG_LED

#ifdef DEBUG_LED
static void debug_led(uint64_t clock);
#define DBG_LED(x) debug_led(x)
#else
#define DBG_LED(x)
#endif

// Period of HID reports in microseconds
static uint64_t hid_reporting_period = 5L * 1000L * 1000L;  // default is 5 seconds

// Sensor ports to probe
static uint16_t sensor_mask = 0xFFFF;

// Offset and scaling for converting the temp to a HID report value
static float temp_offset;
static float temp_scale;
static float temp_max;

// Temperature data
static float    data[1 + NCHAN_TC + NCHAN_TH];
static uint16_t report_data[1 + NCHAN_TC + NCHAN_TH];

//================================================================================
//================================================================================
//      HidTempSensor


static void init_temps(void)
{
    for (uint8_t i = 0; i < 1 + NCHAN_TC + NCHAN_TH; i++)
    {
	data[i]        = temp_max;
	report_data[i] = 0xFFFF;
    }
}

void hidtemp_init(void)
{
#ifdef DEBUG_LED
    pinMode(13, OUTPUT);
#endif

    // Get the minimum periodicity of our HID reports
    // We don't allow a period shorter than 500 ms.
    // Also, we don't send a report until after we have new data.   As such,
    // this is really a minimum period.

    hidtemp_set_reporting_period(eeprom_reporting_period_get());

    // Note which sensors we should probe

    hidtemp_set_sensor_mask(eeprom_sensors_get());

    // And our transform from floating point temps in degrees Celsius to
    // integer HID report values in the range 0 - 0xFFFF
    eeprom_reporttransform_get(&temp_offset, &temp_scale);

    // And initialize our stored temperatures
    temp_max = (65535.0 / temp_scale) - temp_offset; 
    init_temps();
}

float hidtemp_get_temp(uint8_t id)
{
    if ( id < 0 || id >= (1 + NCHAN_TC + NCHAN_TH) )
	return 0.0;
    return data[id];
}

void hidtemp_set_temp(uint8_t id, float temp)
{
    // We need to check for an uninstalled sensor since
    // we may have started a read and while the read was
    // waiting to finish, the user then used a PORTS command
    // to indicate that the port was not in use

    if ( id < 0 || id >= (1 + NCHAN_TC + NCHAN_TH) ||
	 ((id != 0) && !(sensor_mask & (1 << (id-1)))) )
	return;

    // For storing the measured, floating point value
    // we stick to a 1:1 mapping of sensor port to array index
    data[id] = temp;

    // Convert the floating point temperature to an integer in the
    // range [0, 65535].  We first multiply the temperature by 100
    // so that we can retain two decimal digits to the right of the
    // decimal point.

    uint16_t itemp;

    temp = (temp + temp_offset) * temp_scale;

    if ( temp < 0.0 ) itemp = 0;
    else if ( temp >= 65535.0 ) itemp = 0xFFFF;
    else itemp = (uint16_t)temp;

    // Now map sensor ports to HID Joystick entities
    //
    //   Port 0      = Ambient             = Wheel
    //   Ports 1 - 4 = Thermocouples 1 - 4 = X, Y, Z, Rudder
    //   Ports 5 - 6 = Thermistors   5 - 6 = RX, RY, RZ, Throttle
    //
    // We achieve this desired ordering via how the fields are
    // declared in the HID descriptor itself.

    report_data[id] = itemp;
}

static uint8_t adc_finished = 0;

#define STATE_NEXT_DEVICE   0
#define STATE_WAITING_CLOCK 1
#define STATE_WAITING_ADC   2
#define STATE_READ_DEVICE   3

void hidtemp_update(uint64_t clock)
{
    // Absolute clock time until an ongoing temperature read completes
    // Units of microseconds, uS
    static uint64_t wait_until = 0;

    // Absolute clock time until we send the next HID report
    // Units of uS
    static uint64_t next_report = 0;

    // Sensor to read next
    static uint8_t sensor = 1 + NCHAN_TC + NCHAN_TH;

    // Current state machine state
    static uint8_t state = STATE_NEXT_DEVICE;

    DBG_LED(clock);

    bool check_report_clock = false;

    if ( state == STATE_WAITING_CLOCK )
    {
	if ( clock < wait_until )
	    return;

	state = STATE_READ_DEVICE;

	// fall through to read
    }
    else if ( state == STATE_WAITING_ADC )
    {
	if ( !adc_finished )
	    return;

	state = STATE_READ_DEVICE;

	// fall through to read
    }

    if ( state == STATE_READ_DEVICE )
    {
	float temp = read_temp(sensor);
	hidtemp_set_temp(sensor, temp);

	// Flag that there's new data so check to see if it's time
	// to send a new HID report.  Rather than check right now,
	// we note this and get the next temperature conversion started
	// and then we go back and send the HID report if it's time to.

	check_report_clock = true;

	state = STATE_NEXT_DEVICE;

	// fall through
    }

    if ( state == STATE_NEXT_DEVICE )
    {
	// Identify the next sensor to probe

	while ( ++sensor <= NCHAN_TC + NCHAN_TH )
	    if ( sensor_mask & (1 << (sensor - 1)) )
		break;

	if ( sensor >= (1 + NCHAN_TC + NCHAN_TH) )
	    // Sensor 0 -- ambient -- is always okay
	    sensor = 0;

	uint16_t delay = read_start(sensor);
	if ( delay )
	{
	    wait_until = clock + (uint64_t)delay * 1000L;
	    state = STATE_WAITING_CLOCK;
	}
	else
	    state = STATE_WAITING_ADC;
    }

    if ( check_report_clock )
    {
	if ( clock >= next_report )
	{
	    // Next report in 5 seconds
	    next_report = clock + hid_reporting_period;
	    HID_SendReport(1, (const void *)&report_data, 18);
	}
    }
}

static uint16_t read_start(uint8_t id)
{
    uint16_t delay;

    if ( id == 0 )
    {
	// Ambient temp

	srv.amb.nextConversion();
	delay = srv.amb.getConvTime();
    }
    else if ( id <= NCHAN_TC )
    {
	// Start a conversion for a thermocouple

	srv.adc.nextConversion(id - 1);
	srv.amb.nextConversion();
	delay = max(srv.adc.getConvTime(), srv.amb.getConvTime());
    }
    else
    {
	// Start an onboard A/D conversion to read a thermistor

	read_adc_start(pins_th[id - (NCHAN_TC + 1)]);
	return (uint16_t)0;
    }

    if ( delay ) return delay;
    else return (uint16_t)250;
}

static float read_temp(uint8_t id)
{
    if ( id == 0 )
    {
	// Ambient temp
	return srv.amb.getAmbC();
    }
    else if ( id <= NCHAN_TC )
    {
	TC_TYPE tc;
	srv.amb.readSensor();
	return tc.Temp_C(0.001 * srv.adc.readuV(), srv.amb.getAmbC());
    }
    else
        return srv.thermistors[id - (NCHAN_TC + 1)].tempC((int32_t)read_adc());
}

ISR(ADC_vect)
{
    adc_finished = 1;
}

// Code lifted from Arduino's wiring_analog.c
// Written by David A. Mellis

// Modified to use the ADC Interrupt vector so as to avoid blocking while
//  waiting for the A/D conversion to finish

static void read_adc_start(uint8_t pin)
{
    static uint8_t analog_reference = DEFAULT;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
    if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)
    if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
    if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif
	
#if defined(__AVR_ATmega32U4__)
    pin = analogPinToChannel(pin);
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#elif defined(ADCSRB) && defined(MUX5)
    // the MUX5 bit of ADCSRB selects whether we're reading from channels
    // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
 
    // set the analog reference (high two bits of ADMUX) and select the
    // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
    // to 0 (the default).
#if defined(ADMUX)
    ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif

#if defined(ADCSRA) && defined(ADCL)
    // start the conversion
    adc_finished = 0;
    sbi(ADCSRA, ADIE);
    sbi(ADCSRA, ADSC);
#endif
}

static uint16_t read_adc(void)
{
#if defined(ADCSRA) && defined(ADCL)
    uint8_t low  = ADCL;
    uint8_t high = ADCH;
    return ((high << 8) | low);
#else
    return (0);
#endif
}

#ifdef DEBUG_LED
void debug_led(uint64_t clock)
{
    static uint64_t fooBar = 0;
    static bool foo = false;

    if ( clock > fooBar )
    {
	if ( foo )
	{
	    digitalWrite(13, LOW);
	    foo = false;
	    fooBar = clock + 1000L * 1500L;  // 1.5 second
	}
	else
	{
	    digitalWrite(13, HIGH);
	    foo = true;
	    fooBar = clock + 1000L * 3000L;  // 3.0 second
	}
    }
}
#endif

void hidtemp_set_reporting_period(uint16_t period)
{
    if ( period < 500 ) period = 500;
    hid_reporting_period = (uint64_t)period * 1000L;
}

void hidtemp_set_sensor_mask(uint16_t mask)
{
    sensor_mask = mask ? mask : 0xFFFF;
    init_temps();
}

void hidtemp_set_transform(float offset, float scale)
{
    temp_offset = offset;
    temp_scale  = scale;
}

#endif // USE_HID
