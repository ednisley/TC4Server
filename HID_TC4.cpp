// HID_TC4.cpp
//
//   Arduino HID routines which override the routines supplied by the Arduino
//   HID.cpp core library.  The overrides are not C++ class hierarchy overrides.
//   They are good, old fashioned "the linker found me first" overrides.
//
//   This file must have a name other than HID.cpp.  Otherwise, the Arduino build
//   environment will overwrite the resulting object file with the Arduino core HID.o
//   file.
//
// This code follows that of Peter Barrett's Arduino HID.cpp and Copyright (c) 2011/
//
// Version date: 14 April 2013

//-------------------------------------------
// Revision history
//
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

#include "Platform.h"
#include "USBAPI.h"
#include "USBDesc.h"

#if defined(USBCON) && defined(HID_ENABLED)

#include "HID_TC4.h"
#include "HidTempSensor.h"

//================================================================================
//================================================================================

// HID report descriptor

extern const u8 _hidReportDescriptor[] PROGMEM;

// Report the data as a Joystick: most platforms support it and
// it has at least nine scalar fields with acceptable value ranges.
//
// For Joystick drivers on the USB master end, use the following to
// convert to units of degrees Celsius,
//
//   flat   = 0
//   fuzz   = 0
//   offset = -2732
//   scale  = 10

const u8 _hidReportDescriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x01,                    //     REPORT_ID (1) [superfluous]
// --
// -- The ordering of these usages is intended to map sensor N
// -- to the (N+1)th usage.  E.g., sensor 0 to the first usage,
// -- sensor 1 to the second usage, etc.
// --
    0x09, 0x38,                    //     USAGE (Wheel -- Ambient temp)
    0x09, 0x30,                    //     USAGE (X; TC 1)
    0x09, 0x31,                    //     USAGE (Y; TC 2)
    0x09, 0x32,                    //     USAGE (Z; TC 3)
    0x09, 0x37,                    //     USAGE (Rudder; TC 4)
    0x09, 0x33,                    //     USAGE (RX; TH 5)
    0x09, 0x34,                    //     USAGE (RY; TH 6)
    0x09, 0x35,                    //     USAGE (RZ; TH 7)
    0x09, 0x36,                    //     USAGE (Throttle; TH 8)
// --
// -- End of the key usages
// --
    0x16, 0x00, 0x00,              //     Logical Minimum (-32768)
    0x26, 0xff, 0xff,              //     Logical Maximum (32767)
    0x75, 16,                      //     REPORT_SIZE (16)
    0x95, 9,                       //     REPORT_COUNT (9)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};


extern const HIDDescriptor _hidInterface PROGMEM;
const HIDDescriptor _hidInterface = {
    D_INTERFACE(HID_INTERFACE, 1, 3, 0, 0),
    D_HIDREPORT(sizeof(_hidReportDescriptor)),
    D_ENDPOINT(USB_ENDPOINT_IN(HID_ENDPOINT_INT), USB_ENDPOINT_TYPE_INTERRUPT, 0x40, 0x01)
};

//================================================================================
//================================================================================
//      Driver

u8 _hid_protocol = 1;
u8 _hid_idle = 1;

#define WEAK __attribute__ ((weak))
//#define WEAK

int WEAK HID_GetInterface(u8 *interfaceNum)
{
    interfaceNum[0] += 1;   // uses 1
    return USB_SendControl(TRANSFER_PGM, &_hidInterface, sizeof(_hidInterface));
}

int WEAK HID_GetDescriptor(int i)
{
    return USB_SendControl(TRANSFER_PGM, _hidReportDescriptor, sizeof(_hidReportDescriptor));
}

void WEAK HID_SendReport(u8 id, const void *data, int len)
{
    USB_Send(HID_TX, &id, 1);
    USB_Send(HID_TX | TRANSFER_RELEASE, data, len);
}

bool WEAK HID_Setup(Setup& setup)
{
    u8 r = setup.bRequest;
    u8 requestType = setup.bmRequestType;
    if ( REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType )
    {
	if ( HID_GET_REPORT == r )
	{
	    //HID_GetReport();
	    return true;
	}
	if ( HID_GET_PROTOCOL == r )
	{
	    //Send8(_hid_protocol); // TODO
	    return true;
	}
    }

    if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
    {
	if ( HID_SET_PROTOCOL == r )
	{
	    _hid_protocol = setup.wValueL;
	    return true;
	}

	if ( HID_SET_IDLE == r )
	{
	    _hid_idle = setup.wValueL;
	    return true;
	}
    }
    return false;
}

#endif /* if defined(HID_ENABLED) && defined(USBCON) */
