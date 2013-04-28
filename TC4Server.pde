// TC4Server.pde
//
//   A simple command / response server for manipulating a
//   TC4 Digital Thermometer and Termperture Controller Shield
//
//   This version of the software is targetted to the TC4 Shield Version 5.30,
//
//       http://code.google.com/p/tc4-shield/
//
//-------------------------------------------
// Revision history
//
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

#include <Wire.h>
#include "Configuration.h"
#include "Server.h"
#include "Eeprom.h"

#ifdef USE_HID
#include "HidTempSensor.h"
#include "Utils.h"
#endif

Server srv;

void setup()
{
    // Other TC4 apps take a 100 ms delay
    delay(100);
    eeprom_init();
#ifdef USE_HID
    hidtemp_init();
#endif
    srv.init();
}

#if defined(ARDUINO)
void loop()
#else
int main()
#endif
{
#ifdef USE_HID
    hidtemp_update(get_clock());
#endif
    srv.runSlice();
}

