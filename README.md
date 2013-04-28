24 April 2013  
Revision 4

Contents

1. Introduction
2. Building the software
3. Basic operation
4. USB HID interface
5. Using a TC4 Shield with a Leonardo
6. Commands 
7. Unused TC4 features
8. Credits


1. Introduction
--------------
The TC4Server Arduino sketch presents an eight channel temperature monitoring
device which interfaces to a computer as both a USB HID interface (Arduino
Leonardo) and a serial command / response server (Leonardo, Duemilanove, etc.).

To use the sketch, a TC4 Shield Version 5.30,

	http://code.google.com/p/tc4-shield/

is required.  Later versions of the TC4 Shield may also work.  This sketch has
been written for and tested on the Arduino Duemilanove (AVR ATmega 328p) and
Arduino Leonardo (AVR ATmega 32u4), but should work with any Arduino or Arduino
clone compatible with the TC4 Shield and around 28K of program space for USB HID
support and only 20K without.

The command / response protocol is spoken over the Arduino's Serial interface
which is, in the case of Duemilanove and Leonardo's, their USB interface.

The TC4Server allows up to four thermocouples and four thermistors to be
queried for their current temperature in degrees Celcius via a "GET" command.
The recognized sensor "ports" are

|  Port  |  Device
| -----: | :-----
|     0  | TC4's onboard ambient temperature sensor
| 1 - 4  | TC4's thermocouple ports 1 - 4 (18 bit A/D)
| 5 - 8  | Thermistors on the Arduino ports A0 - A3 with Port 5=A0, 6=A1, ... (10 bit A/D)

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

On startup, the TC4Server sets all PWM duty cycles to 0% and SSR ports to 0 (off).
Additionally, configuration settings for the thermistors and USB HID settings
are loaded from EEPROM and used in initialization of the TC4Server.


2. Building the software
------------------------
1. Arduino 1.0 or later is required.
2. Load the sketch, TC4Server.pde, into the Arduino application.
3. Build and load onto the Arduino target device.

Using the Arduino application's Serial Monitor, you should be able to connect to
the Arduino over its serial connection (USB interface).  Try that from the Arduino
application and send the VERSION command by entering the text string "VERSION" and
sending it.  The TC4Server should respond with version information.  You can also
send the HELP command to receive detailed help on the available commands.

3. Basic Operation
-----------------
Temperature information may be obtained in one of two ways from the TC4Server:
by connecting to it over its serial interface and sending commands and
reading the responses, or by passively receiving USB HID reports from a
TC4Server built with USB HID support and running on an Arduino which supports
the USB HID interface.  Both methods may be used concurrently.

When sending commands to the TC4Server over the serial interface, the 
basic command and response sequence is one in which the client -- a host
computer -- sends a US-ASCII command terminated with a carriage return (CR,
0x0D), line feed (LF, 0x0A), or a carriage return line feed pair (CRLF).
With the exception of the HELP command, the TC4Server will always respond
with a single line response which

1. begins with a `+` or `-` to indicate success (`+`) or failure (`-`), and
2. ends with a CRLF pair.

The supported commands are described in Section 6, Commands.

When the TC4Server is not built with USB HID support, a temperature sensor is
only sampled in response to an explicit request via a GET command.  Sampling
a sensor may take upwards of 0.3 seconds for ports 0 - 4.  A little less time
for ports 5 - 8.  It takes close to 2.5 seconds for all nine ports to be sampled.

When the TC4Server is built with USB HID support, each temperature sensor is
periodically read in the sequence 0, 1, 2, ..., 8.  After a sensor read has
completed, a USB HID report is sent if it has been at least T milliseconds
since the last report.  By default, T is 5000 ms (5 seconds).  The REPORT
command changes the reporting period to any value from 500 ms to 65535 ms
in units of milliseconds.  In this case, the GET command returns the last
sampled value for the requested port.

The PORTS command specifies which ports to sample.  Port 0 is always sampled
and thus may not be controlled via the PORTS command.  The ports are always
sampled in ascending numerical order regardless of the order they might
be specified with the PORTS command.

For usages in which the TC4Server will be used primarily as a USB HID
device, you should configure the TC4Server by connect to it via the serial
interface and issuing PORTS, REPORT, and THERMISTOR commands to set any
configuration settings.  Those settings will automatically be saved in the
AVR's EEPROM and used to configure the device each time it is powered up or
restarted.  Once the TC4Server is configured, you can then ignore the serial
command interface an work solely with the temperature measurements reported
over the USB HID interface.


4. USB HID interface
--------------------
On Arduinos configured to present themselves as a USB HID device (e.g., Leonardo),
the TC4Server can also generate USB HID reports which present, in scaled units
of degrees Kelvin, the temperature readings from each sensor port, ports 0 - 8.

So as to not require a custom USB HID driver, the TC4Server's USB HID report is
that of a Joystick with the following sensor port to Joystick position mapping,

  Port (usage)            | Joystick Control
  :---------------------- | :---------------
  Port 0 (ambient)        | Wheel
  Port 1 (thermocouple 1) | X
  Port 2 (thermocouple 2) | Y
  Port 3 (thermocouple 3) | Z
  Port 4 (thermocouple 4) | Rudder
  Port 5 (thermistor 1)   | RX
  Port 6 (thermistor 2)   | RY
  Port 7 (thermistor 3)   | RZ
  Port 8 (thermistor 4)   | Throttle

To map a USB HID report position back to a temperature, use either of the
formulae,

	temperature Kelvin  = position / 10
	temperature Celsius = position / 10 - 273.2

The range of Joystick positions is 0 to 65535 which corresponds to a
temperature range of -273.2 C to 6280.3 C.  By default an offset of 273.2 and
scale factor of 10 is used.  These values may be changed via the REPORT command
described in Section 5.


5. Using a TC4 Shield with a Leonardo
-------------------------------------
The Duemilanove's A4 and A5 pins double as the I2C SDA and SCL pins which
the TC4 Shield uses for serial communications with its external chips.
However, on the Leonardo the SDA and SCL pins are distinct from the A4 and
A5 pins.  To use a TC4 Shield with a Leonardo, bend up or otherwise do not
use the TC4's SDA and SCL male pins when connecting the TC4 Shield to the
Leonardo.  Run jumper wires from the Leonardo's SDA and SCL pins to the
TC4 Shield's A4/SDA and A5/SCL inputs.  These pins are on opposite corners
of the respective boards with the Leonardo's SDA and SCL pins near its
RESET button.


6. Commands
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

PORTS [port-number [...]]   _USB HID capable devices only_   
  Specify which ports, 1 - 8, have connected sensors.  By default, all
  ports are assumed to have sensors connected.  Issuing this command with no
  parameters will display the list of connected ports.  Note that port 0 is
  always connected and cannot be specified with this command.
  
  Settings made with this command are saved in EEPROM.

PWM [port-number duty-cycle [...]]  
  Set the PWM duty cycle for the specified port.  The permitted duty-cyles
  are floating point numbers ranging from 0 (off) to 100 (on 100%).  For
  example, set port 1 to a 33.333% duty cycle, use the command

     Client: PWM 1 33.33\r\n
     Server: +OK\r\n

  The only valid port number is 1.  Issuing the command with no parameters
  sets the duty cycle to 0 on all PWM ports.

REPORT [period offset scale]  _USB HID capable devices only_   
  Specify the minimum period in milliseconds between USB HID reports as well  as an offset and scale for transforming a floating point temperature in  degrees Celsius to a signed 16 bit integer HID report value in the range  0 to 65535,	report value = ( temperature + offset ) * scale
  "period" and "scale" must be integers; "offset" need not be.
  Issuing this command with no parameter displays the current settings.  Settings made with this command are saved in EEPROM.

RESET ["FACTORY"]  
  Perform an immediate software reset.  When the optional parameter "FACTORY"
  is specified, the EEPROM is reset to factory default settings.

SRAM  
  Report available SRAM.

SSR [port-number 0|1 [...]]  
  Enable (1) or disable (0) the specified SSR output.  The valid port numbers
  are 1 - 2.

  When no parameters are specified, all SSR outputs are disabled.

THERMISTOR port-number [adc-max t0 r0 beta r1 r2]  
  Specify the characteristics of the thermistor and A/D circuit tied to the
  specified port.  The valid port numbers are 5, 6, 7, and 8. The maximum A/D
  value is given by adc-max (e.g., 1023 for 10 bit resolution).  Also specified
  is the thermistor's resistance r0 in Ohms at the Celsius temperature t0 and
  its beta value in degrees Kelvin.  The resistance in Ohms of two resistors r1
  and r2 in a voltage divider circuit should also be supplied,

	Vref --- r2 ----+----- r1 -----+
	                |              |
	                +- thermistor -+
	                |              |
	               Vout <- A/D -> Gnd

  Specify "nc" when r1 is not included in the circuit.

  When only a port-number is supplied, the settings for that thermistor are
  displayed.  Settings made with this command are saved in EEPROM.

VERSION  
  Report version and build information.


7. Unused TC4 features
----------------------
1. The TC4 shield's EEPROM chip is not used.  This EEPROM chip should not be
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


8. Credits
---------
Jim Gallt's source codes for handling thermocouple, MCP9800, and MCP3424 devices
are taken from the source file collection at

	http://code.google.com/p/tc4-shield/

Where appropriate, the revision history in the impacted source files has been
updated to indicate any changes.
