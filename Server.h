// Server.h

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

// Acknowledgement is given to Jim Gallt, author of the Bourbon libraries which supply
//   the cADC.cpp and thermocouple.cpp code used herein.

#if !defined(_SERVER_H__)

#define _SERVER_H__

#include <stdint.h>
#include "Configuration.h"
#include "cADC.h"
#include "Thermistor.h"

// Longest commands:
//
//  PWM 0 65535
//  TEMP 1 2 3 4 5 6 7 8
//  THERMISTOR 4 262143 25 100000 4190 10000 10000
//  12345678901234567890123456789012345678901234567890

// Maximum length of a command line in characters, excluding CRLF
#define MAXBUFLEN 60

// Maximum number of tokens in a command line (command + parameters)
#define MAXTOKENS 9

#if MAXTOKENS < (1 + NCHAN_TC + NCHAN_TH)
#warning "MAXTOKENS should be at least as large as 1 + NCHAN_TC + NCHAN_TH"
#endif

class Server
{
public:
	// Managed devices
	cADC       adc;  // MCP3424 device
	ambSensor  amb;  // MCP9800 used for measuring the board temp ("ambient" temp)
	Thermistor thermistors[NCHAN_TH]; // Thermistors
	Server(long speed=DEFAULT_BAUD);
	void init(long speed=DEFAULT_BAUD);
	void runSlice(void);

private:
	// Internal reading state as data is read over the serial line
	uint8_t   read_state;

	// Number of bytes read so far into the buffer, read_buf[]
	uint8_t   buflen;

	// We need one extra byte which is used for the NUL terminator
	// on the last token in the command line
	char      read_buf[MAXBUFLEN+2];
};

// Exported from our .pde file
extern Server srv;

// Some of the other utilities need this info exported from Server.cpp
extern uint8_t pins_th[NCHAN_TH];
extern uint8_t pins_ssr[NCHAN_SSR];
extern uint8_t pins_pwm[NCHAN_PWM];

#endif
