// Thermistor.h

//-------------------------------------------
// Revision history
//
// 20130414  Added EEPROM support (changed data structures around)
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

#if !defined(_THERMISTOR_H__)

typedef struct __attribute__ ((__packed__)) {
    uint32_t adc_max;
    int8_t   t0;
    uint32_t r0;
    uint16_t beta;
    uint32_t r1;
    uint32_t r2;
} thermistor_data_t;

// We cannot use sizeof() for the following define constant
// It's used in an "#if" test to ensure we didn't run out of EEPROM

#define THERMISTOR_DATA_SIZE 19

#define _THERMISTOR_H__

class Thermistor {

public:
    Thermistor(void);
    Thermistor(uint32_t adc_max, int8_t t0, uint32_t r0, uint16_t beta, uint32_t r1, uint32_t r2);
    const thermistor_data_t *get(void) { return &params; }
    void set(const thermistor_data_t *params);
    void set(uint32_t adc_max, int8_t t0, uint32_t r0, uint16_t beta, uint32_t r1, uint32_t r2);
    void show(uint8_t port);
    float tempC(uint32_t adc);

private:
    thermistor_data_t params;
    double            vs;
    double            rs;
    double            k;
};

#endif
