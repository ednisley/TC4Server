// Utils.cpp

//-------------------------------------------
// Revision history
//
// 20130407  Address PROGMEM warnings
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

#include <stdint.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <Wire.h>

#include "Configuration.h"
#include "Utils.h"
#include "CriticalSection.h"

#ifdef USE_HID
#include "HidTempSensor.h"
#endif

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#endif
#define PROGMEM __attribute__((section(".progmem.data")))

extern "C" {
extern void twi_releaseBus();
};

PROGMEM prog_char crlf_text[] = "\r\n";

// Forward declarations
static void error_report(int typ, const char *token, int min, int max);

static void error_report(int typ, const char *token, int min, int max)
{
    static PROGMEM prog_char str1[]  = "-ERR The string \"";
    static PROGMEM prog_char str2[]  = "\" is not a valid decimal integer\r\n";
    static PROGMEM prog_char str3[]  = "-ERR The id \"";
    static PROGMEM prog_char str4[]  = "\" is out of range [";
    static PROGMEM prog_char str5[]  = ",";
    static PROGMEM prog_char str6[]  = "]\r\n";
    static PROGMEM prog_char str7[]  = "-ERR The value \"";
    static PROGMEM prog_char str8[]  = "-ERR Syntax error\r\n";
    static PROGMEM prog_char str9[]  = "\" is not a valid floating point number\r\n";
    static PROGMEM prog_char str10[] = "-ERR There must be an even number of parameters, "
	"\"id value [...]\"\r\n";

    if ( !token )
	token = "";

    if ( typ == 1 )
    {
	write_pgmspace(str1);
	Serial.write(token);
	write_pgmspace(str2);
    }
    else if ( typ == 2 )
    {
	write_pgmspace(str3);
	Serial.write(token);
	write_pgmspace(str4);
	Serial.print(min);
	write_pgmspace(str5);
	Serial.print(max);
	write_pgmspace(str6);
    }
    else if ( typ == 3 )
    {
	write_pgmspace(str7);
	Serial.write(token);
	write_pgmspace(str4);
	Serial.print(min);
	write_pgmspace(str5);
	Serial.print(max);
	write_pgmspace(str6);
    }
    else if ( typ == 4 )
    {
	write_pgmspace(str7);
	Serial.write(token);
	write_pgmspace(str9);
    }
    else if ( typ == 5 )
    {
	write_pgmspace(str10);
    }
    else
    {
	write_pgmspace(str8);
    }
}


int parse_tokens(int *vals, char **tokens, uint8_t ntokens, int min, int max)
{
    uint8_t i = 0;

    if ( !vals || !tokens)
	// Error
	goto syntax_error;

    else if ( !ntokens )
	// Not much to do
	return(0);

    // We assume that all leading and trailing whitespace has already been eliminated
    for (i = 0; i < ntokens; i++)
    {
	int8_t sign = 0;
	int v = 0;
	char c, *p = tokens[i];

	while ( (c = *p++) )
	{
	    if ( (c >= '0') && (c <= '9') )
	    {
		v *= 10;
		v += c - '0';
	    }
	    else if ( c == '+' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = 1;
	    }
	    else if ( c == '-' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = -1;
	    }
	    else
		goto format_error;
	}
	if ( sign < 0 )
	    v = -v;

	if ( (v < min) || (v > max) )
	    goto id_error;

	vals[i] = v;
    }

    // Success
    return(0);

syntax_error:
    error_report(-1, 0, 0, 0);
    return(-1);

format_error:
    error_report(1, tokens[i], 0, 0);
    return(-1);

id_error:
    error_report(2, tokens[i], min, max);
    return(-1);
}


int parse_tokens32(int32_t *vals, char **tokens, uint8_t ntokens, int32_t min, int32_t max)
{
    uint8_t i = 0;

    if ( !vals || !tokens)
	// Error
	goto syntax_error;

    else if ( !ntokens )
	// Not much to do
	return(0);

    // We assume that all leading and trailing whitespace has already been eliminated
    for (i = 0; i < ntokens; i++)
    {
	int8_t sign = 0;
	int32_t v = 0;
	char c, *p = tokens[i];

	while ( (c = *p++) )
	{
	    if ( (c >= '0') && (c <= '9') )
	    {
		v *= 10;
		v += c - '0';
	    }
	    else if ( c == '+' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = 1;
	    }
	    else if ( c == '-' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = -1;
	    }
	    else
		goto format_error;
	}
	if ( sign < 0 )
	    v = -v;

	if ( (v < min) || (v > max) )
	    goto id_error;

	vals[i] = v;
    }

    // Success
    return(0);

syntax_error:
    error_report(-1, 0, 0, 0);
    return(-1);

format_error:
    error_report(1, tokens[i], 0, 0);
    return(-1);

id_error:
    error_report(2, tokens[i], min, max);
    return(-1);
}

int parse_2tokens(int *vals, char **tokens, uint8_t ntokens, int *mins, int *maxs)
{
    uint8_t i = 0;

    if ( !vals || !tokens || !mins || !maxs)
	// Error
	goto syntax_error;

    else if ( !ntokens )
	// Not much to do
	return(0);

    else if ( 1 & ntokens )
	// Must be an even number of parameters
	goto syntax2_error;

    // We assume that all leading and trailing whitespace has already been eliminated
    for (i = 0; i < ntokens; i++)
    {
	int v = 0;
	int8_t sign = 0;
	char c, *p = tokens[i];

	while ( (c = *p++) )
	{
	    if ( (c >= '0') && (c <= '9') )
	    {
		v *= 10;
		v += c - '0';
	    }
	    else if ( c == '+' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = 1;
	    }
	    else if ( c == '-' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = -1;
	    }
	    else
		goto format_error;
	}
	if ( sign < 0 )
	    v = -v;

	if ( !(i & 1) )
	{
	    if ( (v < mins[0]) || (v > maxs[0]) )
		goto id_error;
	}
	else
	{
	    if ( (v < mins[1]) || (v > maxs[1]) )
		goto value_error;
	}

	vals[i] = v;
    }

    // Success
    return(0);

syntax_error:
    error_report(-1, 0, 0, 0);
    return(-1);

syntax2_error:
    error_report(5, 0, 0, 0);
    return(-1);

format_error:
    error_report(1, tokens[i], 0, 0);
    return(-1);

id_error:
    error_report(2, tokens[i], mins[0], maxs[0]);
    return(-1);

value_error:
    error_report(3, tokens[i], mins[1], maxs[1]);
    return(-1);
}


int parse_2ftokens(int32_t *vals, float scale, char **tokens, uint8_t ntokens,
		   int32_t *mins, int32_t *maxs)
{
    uint8_t i = 0;
    int32_t max1, min1;

    if ( !vals || !tokens || !mins || !maxs)
	// Error
	goto syntax_error;

    else if ( !ntokens )
	// Not much to do
	return(0);

    else if ( 1 & ntokens )
	// Must be an even number of parameters
	goto syntax2_error;

    min1 = (uint32_t)(0.5 + (float)mins[1] * scale);
    max1 = (uint32_t)(0.5 + (float)maxs[1] * scale);

    // We assume that all leading and trailing whitespace has already been eliminated
    for (i = 0; i < ntokens; i++)
    {
	int32_t v = 0;
	float d = 1.0f, f = 0.0f;
	int8_t sign = 0, decimal = 0;
	char c, *p = tokens[i];

	while ( (c = *p++) )
	{
	    if ( (c >= '0') && (c <= '9') )
	    {
		if ( decimal )
		{
		    d /= 10.0f;
		    f += (c - '0') * d;
		}
		else
		{
		    v *= 10;
		    v += c - '0';
		}
	    }
	    else if ( c == '.' )
	    {
		if ( !(1 & i) )
		    goto format_error;
		else if ( decimal )
		    goto formatf_error;
		decimal = 1;
	    }
	    else if ( c == '+' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = 1;
	    }
	    else if ( c == '-' )
	    {
		if ( sign != 0 )
		    goto format_error;
		sign = -1;
	    }
	    else
		goto format_error;
	}

	if ( !(i & 1) )
	{
	    if ( sign < 0 )
		v = -v;
	    if ( (v < mins[0]) || (v > maxs[0]) )
		goto id_error;

	    vals[i] = v;
	}
	else
	{
	    f += (float)v;
	    if ( sign < 0 )
		f = -f;

	    int32_t temp = (int32_t)(0.5 + f * scale);

	    if ( (temp < min1) || (temp > max1) )
		goto value_error;

	    vals[i] = temp;
	}
    }

    // Success
    return(0);

syntax_error:
    error_report(-1, 0, 0, 0);
    return(-1);

syntax2_error:
    error_report(5, 0, 0, 0);
    return(-1);

format_error:
    error_report(1, tokens[i], 0, 0);
    return(-1);

formatf_error:
    error_report(4, tokens[i], 0, 0);
    return(-1);

id_error:
    error_report(2, tokens[i], mins[0], maxs[0]);
    return(-1);

value_error:
    error_report(3, tokens[i], mins[1], maxs[1]);
    return(-1);
}


uint8_t tokenize(char *line, uint8_t len, char **tokens, uint8_t maxtok)
{
    // Sanity checks
    if (!tokens || !maxtok || !line || !len)
	return(0);

    // Terminate the end of the line as it is the end of the last token
    line[len] = '\0';

    uint8_t count = 0; // Count of tokens identified
    uint8_t next  = 1; // The next non whitespace char is the start of a token

    while (len > 0)
    {
	if ((*line == '\n') || (*line == '\r') || (*line == ' ') || (*line == '\t'))
	{
	    *line = '\0';
	    next = 1;
	}
	else if (next)
	{
	    tokens[count++] = line;
	    if (count >= maxtok)
		break;
	    next = 0;
	}

	++line;
	--len;
    }

    return(count);
}


void write_pgmspace(const prog_char *str)
{
    if (!str)
	return;

    unsigned char buf[32+1];
    size_t l = strlen_P(str);
    while (l)
    {
	size_t l2 = (l >= sizeof(buf)) ? sizeof(buf) - 1 : l;

	memcpy_P(buf, str, l2);
	buf[l2] = '\0';
	str += l2;
	l   -= l2;
	Serial.write((char *)buf);
    }
}

// We're going to put a 16bit timer into CTC mode for purposes
// of tracking elapsed time.  The elapsed time will then be used
// for handling read/write timeouts to the I2C (TWI) bus as well
// as waiting for external A/D conversions to complete. (For internal
// A/D conversion, we use the ADC interrupt vector.)

// We'll divide F_CPU by 64 (CLOCK_PRESCALE) and then have an interrupt
// fire when a counter reaches 25 (TOP_COUNT).  For a 16 MHz processor,
// this gives us an interrupt at 10 KHz and 5 KHz for an 8 MHz processor.

#define TOP_COUNT               25L
#define CLOCK_PRESCALE          64L

// We'll accumulate in a 64 bit counter, the number of elapsed microseconds.
// MICROS_INTERVAL is the number of approximate microseconds between each
// call to our interrupt descibed by TOP_COUNT and CLOCK_PRESCALE.

#define MICROSECONDS_PER_SECOND 1000000L
#define MICROS_INTERVAL ( ( MICROSECONDS_PER_SECOND * TOP_COUNT * CLOCK_PRESCALE) / F_CPU )

// ONE_SECOND_COUNTDOWN is how many times our interrupt must fire for
// approximately a second of time to pass.

#define ONE_SECOND_COUNTDOWN  ( F_CPU / (TOP_COUNT * CLOCK_PRESCALE) )

// A countdown timer of sorts for timing out read/writes with the Arduino Wire
// library

static volatile uint16_t wire_read_timeout_countdown = 0;

#ifdef USE_HID

// Safely get the count of elapsed microseconds since the processor
// last reset itself.

static uint64_t microseconds = 0;

uint64_t get_clock(void)
{
    uint64_t clock;

    CRITICAL_SECTION_START;
    clock = microseconds;
    CRITICAL_SECTION_END;

    return clock;
}

#endif

ISR(TIMER1_COMPA_vect)
{
#ifdef USE_HID
    // Track the count of elapsed microseconds since the processor was
    // last reset
    microseconds += MICROS_INTERVAL;
#endif

    // Handle read timeouts
    if ( 0 == wire_read_timeout_countdown ) return;
    if ( 0 == --wire_read_timeout_countdown ) twi_releaseBus();
}

void init_timers(void)
{
    // Arduino's use these timers
    //   0 -- Delay functions
    //   1 -- Servo library
    //   2 -- Sound library [e.g., tone()]
    //   3,4,5 only on Megas

    // We go ahead and use Timer 1 and we leave it running
    //   all the time with the interrupt firing at 2 Hz which
    //   is fine for setting up a 1 second timeout.
    wire_read_timeout_countdown = 0;

#if CLOCK_PRESCALE != 64
#error init_timers() coded for a clock prescaling of /64
#endif

    TCCR1A = 0; // Do not set COM1A1:0; otherwise, the SSR pins (OC1A & OC1B) will toggle
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = 0xffff & TOP_COUNT; // 10 KHz (16 MHz / 64 / TOP_COUNT )

    TCCR1B |= ( 1 << CS11 ) | ( 1 << CS10 ) | ( 1 << WGM12 ); // 64 prescaler, CTC
    TIMSK1 |= ( 1 << OCIE1A ); // Timer/Counter 1 output compare A match interrupt enable
}

uint8_t wire_read(uint8_t address, uint8_t *data, uint8_t length)
{
    if (!data || !length)
	return 0;

    // Start a 1 second timeout

    // Our read countdown timer is wider than 8 bits and so
    // we cannot be assured that an assignment to it is atomic
    {
	CRITICAL_SECTION_START;
	wire_read_timeout_countdown = ONE_SECOND_COUNTDOWN;
	CRITICAL_SECTION_END;
    }

    uint8_t nread = Wire.requestFrom(address, length);

    // Cancel the timeout
    {
	CRITICAL_SECTION_START;
	wire_read_timeout_countdown = 0;
	CRITICAL_SECTION_END;
    }

    // If nothing was read, assume a timeout
    if ( !nread ) return 0;

    // Copy the read data into the receiving buffer
    uint8_t ncopy = ( nread <= length ) ? nread : length;
    for ( uint8_t i = 0; i < ncopy; i++ ) data[i] = Wire.read();

    return nread;
}

#ifdef STACK_PAINT

//Stack checking
//http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=52249
extern uint8_t _end;
extern uint8_t __stack;

#define STACK_CANARY  0xc5

void StackPaint(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));
void StackPaint(void)
{
#if 0
    uint8_t *p = &_end;

    while(p <= &__stack)
    {
        *p = STACK_CANARY;
        p++;
    }
#else
    __asm volatile ("    ldi r30,lo8(_end)\n"
                    "    ldi r31,hi8(_end)\n"
                    "    ldi r24,lo8(0xc5)\n" /* STACK_CANARY = 0xc5 */
                    "    ldi r25,hi8(__stack)\n"
                    "    rjmp .cmp\n"
                    ".loop:\n"
                    "    st Z+,r24\n"
                    ".cmp:\n"
                    "    cpi r30,lo8(__stack)\n"
                    "    cpc r31,r25\n"
                    "    brlo .loop\n"
                    "    breq .loop"::);
#endif
} 

uint16_t StackCount(void)
{
    const uint8_t *p = &_end;
    uint16_t       c = 0;

    while(*p == STACK_CANARY && p <= &__stack)
    {
        p++;
        c++;
    }

    return c;
}

#endif
