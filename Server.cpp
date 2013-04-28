// Server.cpp
//
//   This is the work horse for the TC4Server package.	This is the code
//   which reads commands and parses commands from the Serial device and
//   then executes them, writing the result back to the Serial device.
//
// Version date: 19 January 2013

//-------------------------------------------
// Revision history
//
// 20130417  Extended REPORT command to allow specification of offset and scale
// 20130414  Added EEPROM support
// 20130315  Updated with HID support for Leonardo
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

// Acknowledgement is given to Jim Gallt, author of the Bourbon libraries which supply
//   the cADC.cpp and thermocouple.cpp code used herein.

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "Configuration.h"
#include "Server.h"
#include "thermocouple.h"
#include "Thermistor.h"
#include "Utils.h"
#include "Eeprom.h"

#ifdef USE_HID
#include "HidTempSensor.h"
#endif

#if !defined(ARDUINO)
#include "Serial.h"
#endif

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#endif
#define PROGMEM __attribute__((section(".progmem.data")))

uint8_t pins_th[NCHAN_TH]   = { PIN_THERM_1, PIN_THERM_2, PIN_THERM_3, PIN_THERM_4 };
uint8_t pins_ssr[NCHAN_SSR] = { PIN_SSR_1, PIN_SSR_2 };
uint8_t pins_pwm[NCHAN_PWM] = { PIN_PWM_1 };

static const PROGMEM prog_char space_text[] = " ";
static const PROGMEM prog_char okay_text[] = "+OK";

static const PROGMEM prog_char bad_line[] = "-ERR Bad command\r\n";
static const PROGMEM prog_char okay_line[] = "+OK\r\n";
static const PROGMEM prog_char long_line[] = "-ERR Command too long\r\n";

typedef void action_proc_t(Server *server, char **tokens, uint8_t count);

typedef struct {
    const char	  *cmd;
    action_proc_t *action;
} command_table_t;

static action_proc_t get_action;
static action_proc_t help_action;
static action_proc_t pwm_action;
static action_proc_t reset_action;
static action_proc_t ssr_action;
static action_proc_t thermistor_action;
static action_proc_t version_action;

#ifdef USE_HID
static action_proc_t ports_action;
static action_proc_t report_action;
#endif

#ifdef STACK_PAINT
static action_proc_t sram_action;
#endif

// The commands are listed in the order of expected usage frequency.
// That because a simple linear search is conducted looking for a match.

static const command_table_t command_table[] = {
    { "GET",	    get_action },
    { "PWM",	    pwm_action },
    { "SSR",	    ssr_action },
    { "THERMISTOR", thermistor_action },
    { "RESET",	    reset_action },
    { "VERSION",    version_action },
    { "?",	    help_action },
    { "HELP",	    help_action }
#ifdef USE_HID
    , { "REPORT",   report_action },
    { "PORTS",	    ports_action }
#endif
#ifdef STACK_PAINT
    , { "SRAM",	    sram_action }
#endif
};

#define COMMAND_COUNT (sizeof(command_table) / sizeof(command_table_t))


static void version_action(Server *server, char **tokens, uint8_t count)
{
    static PROGMEM prog_char text[] = "+OK TC4Server " VERSION_MAJOR "." VERSION_MINOR
	" built " __DATE__ " at " __TIME__ " for "
#if defined(__AVR_ATmega168__)
	"ATmega 168"
#elif defined(__AVR_ATmega328P__)
	"ATmega 328P"
#elif defined(__AVR_ATmega1280__)
	"ATmega 1280"
#elif defined(__AVR_ATmega2560__)
	"ATmega 2560"
#elif defined(__AVR_ATmega32U4__)
	"ATmega 32U4"
#elif defined(__SAM3X8E__)
	"AT91SAM3X8E"
#endif
	"\r\n";
    write_pgmspace(text);
}

#ifdef STACK_PAINT

static void sram_action(Server *server, char **tokens, uint8_t count)
{
    static PROGMEM prog_char text[] = "+OK Free SRAM is ";

    write_pgmspace(text);
    Serial.print(StackCount());
    write_pgmspace(crlf_text);
}

#endif

static void help_action(Server *server, char **tokens, uint8_t count)
{
    static PROGMEM prog_char text[] =
	"?, HELP\r\n"
	"  This message.\r\n"
	"\r\n"
	"GET [sensor-number [...]]\r\n"
	"  Get the current temperature readings for the specified sensors in degrees\r\n"
	"  Celsius.  The format of the returned data is \"sensor-number temperature\".\r\n"
	"  For example,\r\n"
	"\r\n"
	"    Client: GET 3 3 1\\r\\n\r\n"
	"    Server: +OK 3 245 3 245 1 97\\r\\n\r\n"
	"\r\n"
	"  Specify sensor 0 to read the TC4's onboard temperature sensor.  When no\r\n"
	"  parameters are specified, the temperatures for all sensors is returned.\r\n"
	"\r\n"
	"  The valid sensors are 0 for onboard, 1 - 4 for thermocouples TC-1 - TC-4,\r\n"
	"  and 5 - 8 for thermistors on analog inputs A0 - A3.\r\n"
	"\r\n"
#ifdef USE_HID
	"PORTS [port-number [...]]\r\n"
	"  Specify which ports, 1 - 8, have connected sensors.  By default, all ports\r\n"
	"  are assumed to have sensors connected.  Issuing this command without any\r\n"
	"  parameters displays the list of connected ports.  Note that port 0 is always\r\n"
	"  connected and cannot be specified with this command.\r\n"
	"\r\n"
	"  Settings made with this command are saved in EEPROM.\r\n"
	"\r\n"
#endif
	"PWM [port-number duty-cycle [...]]\r\n"
	"  Set the PWM duty cycle for the specified port.  The permitted duty-cyles\r\n"
	"  are floating point numbers ranging from 0 (off) to 100 (on 100%).  For\r\n"
	"  example, to set port 1 to a 33.333% duty cycle, use the command\r\n"
	"\r\n"
	"     Client: PWM 1 33.33\\r\\n\r\n"
	"     Server: +OK\\r\\n\r\n"
	"\r\n"
	"  The only valid port number is 1.  Issuing the command with no parameters\r\n"
	"  sets the duty cycle to 0 on all PWM ports.\r\n"
	"\r\n"
#ifdef USE_HID
	"REPORT [period offset scale]\r\n"
	"  Specify the minimum period in milliseconds between USB HID reports as well\r\n"
	"  as an offset and scale for transforming a floating point temperature in\r\n"
	"  degrees Celsius to a signed 16 bit integer HID report value in the range\r\n"
	"  0 to 65535,\r\n"
	"\r\n"
	"    report value = ( temperature + offset ) * scale\r\n"
	"\r\n"
	"  \"period\" and \"scale\" must be integers; \"offset\" need not be.\r\n"
	"\r\n"
	"  Issuing this command with no parameter displays the current settings.\r\n"
	"  Settings made with this command are saved in EEPROM.\r\n"
	"\r\n"
#endif
	"RESET [\"FACTORY\"]\r\n"
	"  Perform an immediate software reset.  When the optional parameter \"FACTORY\"\r\n"
	"  is specified, the EEPROM is reset to factory default settings.\r\n"
	"\r\n"
#ifdef STACK_PAINT
	"SRAM\r\n"
	"  Report available SRAM.\r\n"
	"\r\n"
#endif
	"SSR [port-number 0|1 [...]]\r\n"
	"  Enable (1) or disable (0) the specified SSR output.  The valid port numbers\r\n"
	"  are 1 - 2.\r\n"
	"\r\n"
	"  When no parameters are specified, all SSR outputs are disabled.\r\n"
	"\r\n"
	"THERMISTOR port-number [adc-max t0 r0 beta r1 r2]\r\n"
	"  Specify the characteristics of the thermistor and A/D circuit tied to the\r\n"
	"  specified port.  The valid port numbers are 5, 6, 7, and 8.  The maximum A/D\r\n"
	"  value is given by adc-max (e.g., 1023 for 10 bit resolution).  Also specified\r\n"
	"  is the thermistor's resistance r0 in Ohms at the Celsius temperature t0 and\r\n"
	"  its beta value in degrees Kelvin.  The resistance in Ohms of two resistors r1\r\n"
	"  and r2 in a voltage divider circuit should also be supplied,\r\n"
	"\r\n"
	"    Vref --- r2 ----+----- r1 -----+\r\n"
	"                    |              |\r\n"
	"                    +- thermistor -+\r\n"
	"                    |              |\r\n"
	"                  Vout <- A/D -> Gnd\r\n"
	"\r\n"
	"  Specify \"nc\" when r1 is not included in the circuit.\r\n"
	"\r\n"
	"  When only a port-number is supplied, the settings for that thermistor are\r\n"
	"  displayed.  Settings made with this command are saved in EEPROM.\r\n"
	"\r\n"
	"VERSION\r\n"
	"  Report version and build information.\r\n";
    (void)server;

    write_pgmspace(text);
}

#ifdef USE_HID

static void ports_action(Server *server, char **tokens, uint8_t count)
{
    if ( !tokens || !count )
    {
	uint16_t mask = 0x0001;
	uint16_t ports = eeprom_sensors_get();

	write_pgmspace(okay_text);
	for (uint8_t i = 1; i <= NCHAN_TC + NCHAN_TH; i++)
	{
	    if ( ports & mask )
	    {
		write_pgmspace(space_text);
		Serial.print(i);
	    }
	    mask <<= 1;
	}
	Serial.println();
    }
    else
    {
	int ports[1 + NCHAN_TC + NCHAN_TH];

	if ( parse_tokens(ports, tokens, count, 1, NCHAN_TC + NCHAN_TH) )
	    // Error response generated by parse_tokens()
	    return;

	// Light the bits corresponding to the installed sensors
	// Sensor n = bit n-1

	uint16_t mask = 0x0000;
	for (uint8_t i = 0; i < count; i++)
	    mask |= 1 << ( ports[i] - 1);

	// And store the bits in EEPROM
	hidtemp_set_sensor_mask(mask);
	eeprom_sensors_set(mask);

	write_pgmspace(okay_line);
    }
}

static void report_action(Server *server, char **tokens, uint8_t count)
{
    if ( !tokens || !count )
    {
	uint16_t period = eeprom_reporting_period_get();
	float offset, scale;
	eeprom_reporttransform_get(&offset, &scale);
	write_pgmspace(okay_text);
	write_pgmspace(space_text);
	Serial.print(period);
	write_pgmspace(space_text);
	Serial.print(offset);
	write_pgmspace(space_text);
	Serial.println((int32_t)scale);
    }
    else if ( count == 3 )
    {
	static int32_t mins[2] = {     0, -1000};
	static int32_t maxs[2] = {0xFFFF,  1000};
	char hack[] = "0";
	int32_t vals[4];

	// HACK TO MAKE AN EVEN TOKEN COUNT
	tokens[count] = hack;

	if ( parse_2ftokens(vals, 100.0f, tokens, count + 1, mins, maxs) )
	    // Error response was generated by parse_tokens()
	    return;

	// And store the bits in EEPROM

	hidtemp_set_reporting_period((uint16_t)(0xFFFF & vals[0]));
	eeprom_reporting_period_set((uint16_t)(0xFFFF & vals[0]));

	float offset = (float)vals[1] / 100.0;
	float scale  = (float)vals[2];
	hidtemp_set_transform(offset, scale);
	eeprom_reporttransform_set(offset, scale);

	write_pgmspace(okay_line);
    }
    else
	write_pgmspace(bad_line);
}

#endif

static void thermistor_action(Server *server, char **tokens, uint8_t count)
{
    static PROGMEM prog_char err1[] = "-ERR Missing command arguments\r\n";
    static PROGMEM prog_char err2[] = "-ERR Thermistor port number is out of range\r\n";
    static PROGMEM prog_char err3[] = "-ERR t0 is out of range; -128C <= t0 <= 127C\r\n";
    static PROGMEM prog_char err4[] = "-ERR beta is out of ranage; 0 <= beta <= 65535\r\n";

    if ( !tokens || !count || ( count != 1 && count != 7 ) )
    {
	write_pgmspace(err1);
	return;
    }

    // Change R1 'nc' to '0'
    if ( count == 7 &&
	 ( tokens[5][0] == 'N' || tokens[5][0] == 'n' ) &&
	 ( tokens[5][1] == 'C' || tokens[5][1] == 'c' ) &&
	 ( tokens[5][2] == '\0') )
    {
	tokens[5][0] = '0';
	tokens[5][1] = '\0';
    }

    int32_t vals[7];
    if ( parse_tokens32(vals, tokens, count, 0, 0x7fffffff) )
	// Error response was generated by parse_tokens()
	return;

    vals[0] -= ( NCHAN_TC + 1 );
    if ( vals[0] < 0 || vals[0] >= NCHAN_TH)
    {
	write_pgmspace(err2);
	return;
    }

    if ( count == 1)
	server->thermistors[vals[0]].show(vals[0] + (NCHAN_TC + 1));
    else
    {
	if ( vals[2] > 0x7f )
	{
	    write_pgmspace(err3);
	    return;
	}
	else if ( vals[4] > 0xffff )
	{
	    write_pgmspace(err4);
	    return;
	}

	server->thermistors[vals[0]].set((uint32_t)vals[1],  /* adc_max */
					 (int8_t)vals[2],    /* t0	*/
					 (uint32_t)vals[3],  /* r0	*/
					 (uint32_t)vals[4],  /* beta	*/
					 (uint32_t)vals[5],  /* r1	*/
					 (uint32_t)vals[6]); /* r2	*/

	eeprom_thermistor_set((uint8_t)vals[0], server->thermistors[vals[0]].get());

	write_pgmspace(okay_line);
    }
}

static void pwm_action(Server *server, char **tokens, uint8_t count)
{
    (void)server;

    if ( !tokens || !count )
    {
	// Turn all PWMs off
	for (uint8_t i = 0; i < NCHAN_PWM; i++)
	    analogWrite(pins_pwm[i], 0);
    }
    else
    {
	static int32_t mins[2] = {1,   0};
	static int32_t maxs[2] = {1, 100};
	int32_t vals[MAXTOKENS];

	if ( parse_2ftokens(vals, 1000.0f, tokens, count, mins, maxs) )
	    // Error response was generated by parse_2tokens()
	    return;

	for (uint8_t i = 0; i < count; )
	{
	    analogWrite(pins_pwm[vals[i]-1], (int)(0.5f + (float)vals[i+1] * 255.0f / (100.0f * 1000.0f)));
	    i += 2;
	}
    }

    write_pgmspace(okay_line);
}

static void reset_action(Server *server, char **tokens, uint8_t count)
{
    (void)server;

    if ( count == 1 && 0 == strcasecmp(tokens[0], "FACTORY") )
	eeprom_set_defaults();

#if defined(ARDUINO)
    asm volatile ("  jmp 0");
#endif
}


static void ssr_action(Server *server, char **tokens, uint8_t count)
{
    (void)server;

    if ( !tokens || !count )
    {
	// Turn all SSRs off
	for (uint8_t i = 0; i < NCHAN_SSR; i++)
	    digitalWrite(pins_ssr[i], LOW);
    }
    else
    {
	static int mins[2] = {1, 0};
	static int maxs[2] = {NCHAN_SSR, 1};
	int vals[MAXTOKENS];

	if ( parse_2tokens(vals, tokens, count, mins, maxs) )
	    // Error response was generated by parse_2tokens()
	    return;

	for (uint8_t i = 0; i < count; )
	{
	    digitalWrite(pins_ssr[vals[i]-1], vals[i+1] ? HIGH : LOW);
	    i += 2;
	}
    }

    write_pgmspace(okay_line);
}

#ifndef USE_HID

static float get_temp(Server *server, int sensor_id)
{
    TC_TYPE tc;

    if ( !server || sensor_id < 0 || sensor_id >= (1 + NCHAN_TC + NCHAN_TH) )
	return 0.0;

    if ( sensor_id == 0 )
    {
	// Ambient temp
	server->amb.nextConversion();
	delay(server->amb.getConvTime());
	server->amb.readSensor();
	return server->amb.getAmbC();
    }
    else if ( sensor_id <= NCHAN_TC )
    {
	server->adc.nextConversion(sensor_id - 1);
	server->amb.nextConversion();
	delay(max(server->adc.getConvTime(), server->amb.getConvTime()));
	server->amb.readSensor();
	return tc.Temp_C(0.001 * server->adc.readuV(), server->amb.getAmbC());
    }
    else
	return server->thermistors[sensor_id - (NCHAN_TC + 1)].tempC(
	    (int32_t)analogRead(pins_th[sensor_id - (NCHAN_TC + 1)]));
}

#endif

static void get_action(Server *server, char **tokens, uint8_t count)
{
    int sensors[1 + NCHAN_TC + NCHAN_TH];

    if ( !tokens || !count )
    {
	for (uint8_t i = 0; i <= NCHAN_TC + NCHAN_TH; i++)
	    sensors[i] = i;
	count = 1 + NCHAN_TC + NCHAN_TH;
    }
    else if ( parse_tokens(sensors, tokens, count, 0, NCHAN_TC + NCHAN_TH) )
	// Error response was generated by parse_tokens()
	return;

    write_pgmspace(okay_text);

    for (uint8_t i = 0; i < count; i++)
    {
	write_pgmspace(space_text);
	Serial.print(sensors[i]);
	write_pgmspace(space_text);
#ifdef USE_HID
	float temp = hidtemp_get_temp((uint8_t)(0xff & sensors[i]));
#else
	float temp = get_temp(server, sensors[i]);
#endif
	Serial.print(temp);
    }
    write_pgmspace(crlf_text);
}

//  Command line reading states
#define STATE_READ    0
#define STATE_READ_LF 1
#define STATE_SKIP    2

Server::Server(long speed) :
    adc(A_ADC),
    amb(A_AMB),
    read_state(STATE_READ),
    buflen(0)
{
}

void Server::init(long speed)
{
    uint8_t i;

    read_state = STATE_READ;
    buflen = 0;

    // Start timeout interrupt for TWI I/O functions
    init_timers();

    // Set pin directions
    for (i = 0; i < NCHAN_SSR; i++)
	pinMode(pins_ssr[i], OUTPUT);
    for (i = 0; i < NCHAN_PWM; i++)
	pinMode(pins_pwm[i], OUTPUT);

    // Turn SSRs and PWM off
    for (i = 0; i < NCHAN_SSR; i++)
	digitalWrite(pins_ssr[i], HIGH);
    for (i = 0; i < NCHAN_PWM; i++)
	analogWrite(pins_pwm[i], 0);

    // Needed for communications with the thermocouple's amplifier chip
    //	 By rights, this call should be in an initialization routine within
    //	 the cADC library, but the authors chose not to put it there.  Rather
    //	 than change the library too much, we'll put it here for the time
    //	 being.
    Wire.begin();

    // Initialize our serial device
    Serial.begin(speed ? speed : DEFAULT_BAUD);

    // Initialize the ambient temperature filtering
    amb.init(0);

    // Offsets
    // We don't store calibration settings in EEPROM.  If we need linear or
    // other corrections, we can perform them on the client device.
    adc.setCal(CAL_GAIN, UV_OFFSET);
    amb.setOffset(AMB_OFFSET);

    // Thermistors
    // Pull thermistor configs from EEPROM and effect the settings
    thermistor_data_t params;
    for (i = 0; i < NCHAN_TH; i++)
    {
	eeprom_thermistor_get(i, &params);
	thermistors[i].set(&params);
    }
}

void Server::runSlice(void)
{
    while (Serial.available())
    {
	int b = Serial.read();
	if (b == -1)
	    // Read error
	    break;
	else if (b <= 0 || b > 0x7f)
	    // Garbage
	    read_state = STATE_SKIP;

	// Convert to ASCII
	char c = (char)(0xff & b);

	// Convert to upper case
	if ((c >= 'a') && (c <= 'z'))
	    c -= 'a' - 'A';

	switch(read_state)
	{
	case STATE_READ_LF :
	    // Saw a CR terminating the last command.  If this is a LF, then eat it
	    //	 and assume we're seeing CRLF line termination.
	    read_state = STATE_READ;
	    if (c == '\n')
		break;

	    // Fall through

	case STATE_READ :
	    // Read until we see a line terminator or we read too much data
	    if (c != '\n' && c != '\r')
	    {
		read_buf[buflen++] = c;
		// Must allow an extra spare byte in read_buf so we compare
		//   to MAXBUFLEN and not sizeof(buf) == MAXBUFLEN+1
		if (buflen >= MAXBUFLEN)
		    // Not a valid command or too much white space
		    read_state = STATE_SKIP;
		break;
	    }

	    // State for next time
	    read_state = (c != '\r') ? STATE_READ : STATE_READ_LF;

	    // Break the line into tokens
	    // Note that read_buf will be poked with NUL terminators and must
	    //	 have space for one additional NUL at read_buf[buflen]
	    {
		char *tokens[MAXTOKENS];
		uint8_t count = tokenize(read_buf, buflen, tokens, MAXTOKENS);

		// For when we parse the next line
		buflen = 0;

		// Handle the case of an empty line
		if (!count)
		{
		    // Read an empty line
		    write_pgmspace(okay_line);
		    break;
		}

		// Determine which command by inspecting the first token
		uint8_t i;
		for (i = 0; i < COMMAND_COUNT; i++)
		{
		    if ((command_table[i].cmd[0] == tokens[0][0]) &&
			!strcmp(command_table[i].cmd, tokens[0]))
		    {
			(*command_table[i].action)(this, tokens + 1, count - 1);
			break;
		    }
		}
		if (i == COMMAND_COUNT)
		    write_pgmspace(bad_line);
	    }
	    break;

	case STATE_SKIP:
	    // Read until we see a line terminator, '\n' or '\r'
	    if (c != '\n' && c != '\r')
		break;
	    write_pgmspace(long_line);
	    read_state = (c != '\r') ? STATE_READ : STATE_READ_LF;
	    buflen = 0;
	    break;
	}
    }
}
