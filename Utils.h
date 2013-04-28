// Utils.h

//-------------------------------------------
// Revision history
//
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

#if !defined(_UTILS_H__)

#define _UTILS_H__

#include <stdint.h>
#include <avr/pgmspace.h>

uint16_t StackCount(void);

int parse_tokens(int *vals, char **tokens, uint8_t ntokens, int min, int max);
int parse_tokens32(int32_t *vals, char **tokens, uint8_t ntokens, int32_t min, int32_t max);
int parse_2tokens(int *vals, char **tokens, uint8_t ntokens, int *mins, int *maxs);
int parse_2ftokens(int32_t *vals, float scale, char **tokens, uint8_t ntokens, int32_t *mins,
		   int32_t *maxs);
uint8_t tokenize(char *line, uint8_t len, char **tokens, uint8_t maxtok);
void write_pgmspace(const prog_char *str);

uint8_t wire_read(uint8_t address, uint8_t *data, uint8_t length);
void init_timers(void);

extern PROGMEM prog_char crlf_text[];

#ifdef USE_HID
uint64_t get_clock(void);
#endif

#endif
