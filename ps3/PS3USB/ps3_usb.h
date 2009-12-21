/* ps3_usb.h - PS3 Game Controller on Arduino USB Host Library

Copyright (c) 2009 Richard Ibbotson richard.ibbotson@btinternet.com

All right reserved. This library is free software; you can redistribute it
and/or modify it under the terms of the GNU Lesser General Public License 
as published by the Free Software Foundation; either version 2.1 of the License,
or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License along 
with this library; if not, write to the Free Software Foundation, Inc.,
 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA */ 

#ifndef _ps3_usb_h_
#define _ps3_usb_h_
#include "Usb.h"
#include <avr/pgmspace.h>

#define byteswap(x) ((x >> 8) | (x << 8))


/* PS3 data taken from descriptors */
#define PS3_ADDR        1
#define PS3_VID         0x054c  // Sony VID
#define PS3_PID         0x0268  // Batch Device
#define PS3_CONFIGURATION 1
#define PS3_IF          0
#define PS3_NUM_EP      3
#define EP_MAXPKTSIZE   64
#define EP_INTERRUPT    0x03 
#define EP_POLL         0x01
#define CONTROL_PIPE      0
#define OUTPUT_PIPE       1
#define INPUT_PIPE        2


/* Defines for bits in ps3_status */
#define statusDeviceConnected 0x01
#define statusUSBConfigured 	0x02
#define statusPS3Connected  0x04
#define statusReportReceived 0x08


/* Defines of various parameters for PS3 Game controller reports */
#define PS3_F4_REPORT_LEN 4
#define PS3_F5_REPORT_LEN 8
#define PS3_01_REPORT_LEN 48
#define HID_REPORT_FEATURE 3 
#define HID_REPORT_OUTPUT  2
#define PS3_F4_REPORT_ID  0xF4
#define PS3_01_REPORT_ID  0x01
#define PS3_F5_REPORT_ID 0xF5


/* Defines for the PS3 Buttons 
*/


#define buSelect    0
#define buLAnalog   1
#define buRAnalog   2
#define buStart     3
#define buUp        4
#define buRight     5
#define buDown      6
#define buLeft      7
#define buL2        8
#define buR2        9
#define buL1        10
#define buR1        11
#define buTriangle  12
#define buCircle    13
#define buCross     14
#define buSquare    15
#define buPS        16


/* Defines for the PS3 Joysticks 
*/

#define leftJoystickX 0
#define leftJoystickY 1
#define rightJoystickX 2
#define rightJoystickY 3


/* Defines for the PS3 Accelerometers and Gyro 
*/

#define AccelerometerX 0
#define AccelerometerY 1
#define AccelerometerZ 2
#define GyrometerZ 3

/* Defines for the PS3 LED and Rumble 
*/
#define psLED1 0x01
#define psLED2 0x02
#define psLED3 0x04
#define psLED4 0x08
#define psRumbleHigh 0x10
#define psRumbleLow 0x20

//Structure which describes the type 01 input report
typedef struct {        
    unsigned char ReportType;     //Report Type 01
    unsigned char Reserved1;      // Unknown
    unsigned int  ButtonState;    // Main buttons
    unsigned char PSButtonState;  // PS button
    unsigned char Reserved2;      // Unknown
    unsigned char LeftStickX;     // left Joystick X axis 0 - 255, 128 is mid
    unsigned char LeftStickY;     // left Joystick Y axis 0 - 255, 128 is mid
    unsigned char RightStickX;    // right Joystick X axis 0 - 255, 128 is mid
    unsigned char RightStickY;    // right Joystick Y axis 0 - 255, 128 is mid
    unsigned char Reserved3[4];   // Unknown
    unsigned char PressureUp;     // digital Pad Up button Pressure 0 - 255
    unsigned char PressureRight;  // digital Pad Right button Pressure 0 - 255
    unsigned char PressureDown;   // digital Pad Down button Pressure 0 - 255
    unsigned char PressureLeft;   // digital Pad Left button Pressure 0 - 255
    unsigned char PressureL2;     // digital Pad L2 button Pressure 0 - 255
    unsigned char PressureR2;     // digital Pad R2 button Pressure 0 - 255
    unsigned char PressureL1;     // digital Pad L1 button Pressure 0 - 255
    unsigned char PressureR1;     // digital Pad R1 button Pressure 0 - 255
    unsigned char PressureTriangle;   // digital Pad Triangle button Pressure 0 - 255
    unsigned char PressureCircle;     // digital Pad Circle button Pressure 0 - 255
    unsigned char PressureCross;      // digital Pad Cross button Pressure 0 - 255
    unsigned char PressureSquare;     // digital Pad Square button Pressure 0 - 255
    unsigned char Reserved4[3];   // Unknown
    unsigned char Charge;         // charging status ? 02 = charge, 03 = normal
    unsigned char Power;          // Battery status ?
    unsigned char Connection;     // Connection Type ?
    unsigned char Reserved5[9];   // Unknown
    unsigned int AccelX;          // X axis accelerometer Big Endian 0 - 1023
    unsigned int AccelY;          // Y axis accelerometer Big Endian 0 - 1023
    unsigned int AccelZ;          // Z axis accelerometer Big Endian 0 - 1023
    unsigned int GyroZ;           // Z axis Gyro Big Endian 0 - 1023
    
} TYPE_01_REPORT;


class PS3_USB {


    public:
		PS3_USB( void );
		void init(void);
		void task( void );
		unsigned char getStatus(void);
		bool statConnected(void);
		bool statReportReceived(void);
        bool buttonChanged(void);
		bool buttonPressed(unsigned char);
		void LEDRumble(unsigned char);
		unsigned int getMotion(unsigned char);
		unsigned char getJoystick(unsigned char);
		unsigned char getPressure(unsigned char);
		void setBDADDR(unsigned char *);
		void getBDADDR(unsigned char *);
		TYPE_01_REPORT report;
        
    private:
        void PS3_init(void);
		void PS3_poll(void);
};

#endif //_ps3_usb_h_