16 April 2013
Revision 3

Contents

0. Introduction
1. Building the software
2. USB HID interface
3. Using a TC4 Shield with a Leonardo
4. Commands 
5. Unused TC4 features
6. Credits


0. Introduction
--------------
This Arduino sketch is a simple command / response server for use with the
TC4 Shield Version 5.30,

   http://code.google.com/p/tc4-shield/

and possibly later.  It has been developed for and tested on the Arduino
Duemilanove (AVR ATmega 328p) and Arduino Leonardo (AVR ATmega 32u4), but
should work with any Arduino or Arduino clone compatible with the TC4 Shield.

The command / response protocol is spoken over the Arduino's Serial interface
which is, in the case of Duemilanove and Leonardo's, their USB interface.  The
communications speed is presently set to 57600 Baud but can be changed by
changing the file Configuration.h.

Additionally, USB HID reports are generated on Arduinos which present
themselves as a USB HID device.  See the section USB HID section for details.

The interface allows up to four thermocouples and four thermistors to be
queried for their current temperature in degrees Celcius via a "GET" command.
The recognized sensor "ports" are

 Port  Device
    0  TC4's onboard ambient temperature sensor
1 - 4  TC4's thermocouple ports 1 - 4 (A/D has 18 bit precision)
5 - 8  Thermistors on the Arduino analog ports 0 - 3 with Port 5=A0, 6=A1, ...
         (A/D has 10 bit precision)

Additionally,

1. The "PWM" command sets a PWM duty cycle for the TC4's IO3 port which is tied
   to the Arduino's digital pin 3 (ATmega 168 & 328's PD3 / PCINT19 / INT1;
   ATmega 32U4's PD0 / OC0B / SCL / INT0).  Duty cycles set are any floating
   point value between 0 and 100 corresponding to 0% - 100%.  Presently, the
   value is converted to an integer between 0 and 255 as per the requirements
   of the Arduino libraries.

2. The "SSR" command controls the TC4's OT1 and OT2 outputs which are suitable
   for driving a SSR.  These outputs are tied to the Arduino's digital pins 9
   and 10  (ATmega 168 & 328's PB1/OC1A/PCINT1 and PB2/SS/OC1B/PCINT2;
   ATmega 32U4's PB5 / PCINT5 / OC1A / !OC4B / ADC12 and PB6 / PCINT6 / OC1B /
   OC4B / ADC13).

3. The "HELP" or "?" command displays help text.

On startup, the server sets all PWM duty cycles to 0% and SSR ports to 0 (off).
Additionally, configuration settings for the thermistors and USB HID settings
are loaded from EEPROM and used in initialization of the TC4Server software.


1. Building the software
------------------------
1. Load the sketch, TC4Server.pde, into the Arduino application.
2. Build and load onto the Arduino target device.
3. Connect to the Arduino device through its USB connection.
4. Send CR, LF, or CRLF terminated commands to the device.
5. Read back the CRLF terminated response.


2. USB HID interface
--------------------
On Arduinos configured to present themselves as a USB HID device (e.g.,
Leonardo), the TC4Server can also generate USB HID reports which present, in
scaled units of degrees Kelvin, the temperature readings from each sensor port,
ports 0 - 8.

So as to not require a custom USB HID driver, the TC4Server's USB HID report is
that of a Joystick with the following sensor port to Joystick position mapping,

  Port 0 (ambient)        -- Wheel
  Port 1 (thermocouple 1) -- X
  Port 2 (thermocouple 2) -- Y
  Port 3 (thermocouple 3) -- Z
  Port 4 (thermocouple 4) -- Rudder
  Port 5 (thermistor 1)   -- RX
  Port 6 (thermistor 2)   -- RY
  Port 7 (thermistor 3)   -- RZ
  Port 8 (thermistor 4)   -- Throttle

To map a USB HID report position back to a temperature in, use either of the
formulae,

  temperature Kelvin  = position / 100
  temperature Celsius = ( position - 27315 ) / 100

The range of Joystick positions is 0 to 65535 which corresponds to a
temperature range of -273.15 C to 382.20 C.

When operating on a USB HID device, each temperature sensor is periodically
read in the sequnce 0, 1, 2, ..., 8.  After a sensor read has completed as
USB HID report is sent if it has been at least T milliseconds since the last
report.  By default, T is 5000 ms (5 seconds).  The REPORT command changes
the reporting period to any value from 500 ms to 65535 ms in units of
milliseconds.

Further, the PORTS command specifies which ports to read.  Port 0 is always
read and thus may not be controlled via the PORTS command.  The ports are
always read in ascending numerical order regardless of the order they might
be specified with the PORTS command.


3. Using a TC4 Shield with a Leonardo
-------------------------------------
The Duemilanove's A4 and A5 pins double as the I2C SDA and SCL pins which
the TC4 Shield uses for serial communications with its external chips.
However, on the Leonardo, the SDA and SCL pins are distinct from the A4 and
A5 pins.  To use a TC4 Shield with a Leonardo, bend up or otherwise do not
use the TC4's SDA and SCL male pins when connecting the TC4 Shield to the
Leonardo.  Run jumper wires from the Leonardo's SDA and SCL pins to the
TC4 Shields A4/SDA and A5/SCL inputs.  These pins are on opposite corners
of the respective boards with the Leonardo's SDA and SCL pins near its
RESET button.


4. Commands
-----------
The following is lifted straight from the server's HELP response.  Note that
commands names may actually be sent in upper, lower, or mixed case.

?, HELP
  This message.

GET [sensor-number [...]]
  Get the current temperature readings for the specified sensors in degrees
  Celsius.  The format of the returned data is "sensor-number temperature".
  For example,

    Client: GET 3 3 1\r\n
    Server: +OK 3 245 3 245 1 97\r\n

  Specify sensor 0 to read the ambient temperature of the TC4 board itself.
  When no parameters are specified, the temperatures for all sensors are
  returned.

  The valid sensors are 0 for ambient, 1 - 4 for thermocouples TC-1 - TC-4,
  and 5 - 8 for thermistors on analog inputs A0 - A3.

PORTS [port-number [...]]   ** USB HID capable devices only **
  Specify which ports, 1 - 8, have connected sensors.	By default, all
  ports are assumed to have sensors connected.	 Issuing this command with no
  parameters will display the list of connected ports.	 Note that port 0,
  the onboard "ambient" temperature sensor, is always connected and cannot
  be specified with this command.

PWM [port-number duty-cycle [...]]
  Set the PWM duty cycle for the specified port.  The permitted duty-cyles
  are floating point numbers ranging from 0 (off) to 100 (on 100%).  For
  example, set port 1 to a 33.333% duty cycle, use the command

     Client: PWM 1 33.33\r\n
     Server: +OK\r\n

  The only valid port number is 1.  Issuing the command with no parameters
  sets the duty cycle to 0 on all PWM ports.

REPORT [period]  ** USB HID capable devices only **
  Specify the minimum period in milliseconds between USB HID reports.	Issuing
  this command with no parameter will display the current setting.

RESET ["FACTORY"]
  Perform an immediate software reset.	 When the optional parameter "FACTORY"
  is specified, the EEPROM is reset to factory default settings.

SRAM
  Report available SRAM.

SSR [port-number 0|1 [...]]
  Enable (1) or disable (0) the specified SSR output.	The valid port numbers
  are 1 - 2.

  When no parameters are specified, all SSR outputs are disabled.

THERMISTOR port-number [adc-max t0 r0 beta r1 r2]
  Specify the characteristics of the thermistor and A/D circuit tied to the
  specified port.  The valid port numbers are 5, 6, 7, and 8.	The maximum A/D
  value is given by adc-max (e.g., 1023 for 10 bit resolution).  Also specified
  is the thermistor's resistance r0 in Ohms at the Celsius temperature t0 and
  its beta value in degrees Kelvin.  The resistance in Ohms of two resistors r1
  and r2 in a voltage divider circuit should also be supplied,

	Vref --- r2 ----+----- r1 -----+
			|	       |
			+- thermistor -+
			|	       |
		       Vout <- A/D -> Gnd

  Specify "nc" when r1 is not included in the circuit.

  When only a port-number is supplied, the settings for that thermistor are
  displayed.

VERSION
  Report version and build information.


5. Unused TC4 features
----------------------
1. The TC4 shields EEPROM chip is not used.  This EEPROM chip should not be
   confused with the Arduino's (AVR's) onboard EEPROM.  This is an external
   EEPROM chip which can be communicated with using the SPI interface.  It
   is useful to some TC4 applications as a means of storing calibration data
   on the TC4 shield itself.  That way, the calibration data stays with the
   shield as it is moved from Arduino to Arduino.

   As the TC4Server here is intended to be used with a more powerful computer,
   calibration on the shield itself is ignored.  The intent is to provide the
   raw data to the client computer which can then implement whatever form of
   calibration is deemed necessary (e.g., linear two-point calibration with
   thermocouples, possibly the more typical log-based three-point fit used for
   thermistors).

2. Sample averaging (filtering) of the cold junction temperature.  As the data
   samples will be of low frequency, averaging or otherwise filtering the cold
   junction temperature is not attempted.  The cold junction temperature is
   taken to be the temperature read from the onboard temperature sensor.


6. Credits
----------
A number of the source files included here are taken from the source file
collection at

   http://code.google.com/p/tc4-shield/

And have only been modified such that they work with the Arduino 1.0 and later
code environment.  Namely, changing Wire.send() and receive() calls to
Wire.write() and read() calls.

Where appropriate, the revision history in the impacted source files has been
updated to indicate the changes.
