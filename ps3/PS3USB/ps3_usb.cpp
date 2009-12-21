/* ps3_usb.cpp - PS3 Game Controller on Arduino USB Host Library

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

#include "ps3_usb.h"

static unsigned char ps3_status;
static unsigned int oldbuttonstate;
static unsigned char oldpsbuttonstate;

prog_char output_01_report[] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x02, 0xff, 0x27, 0x10, 0x00, 0x32, 0xff, 
                                     0x27, 0x10, 0x00, 0x32, 0xff, 0x27, 0x10, 0x00, 
                                     0x32, 0xff, 0x27, 0x10, 0x00, 0x32, 0x00, 0x00, 
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

prog_char feature_F4_report[] PROGMEM = {0x42, 0x0c, 0x00, 0x00};

EP_RECORD ep_record[ PS3_NUM_EP ];  //endpoint record structure for the PS3 controller
TYPE_01_REPORT report;

MAX3421E Max;
USB Usb;

/* constructor */

PS3_USB::PS3_USB() {

}

void PS3_USB::init(void){
	Max.powerOn();
	delay(200);
}

void PS3_USB::task(void){
	Max.Task();
    Usb.Task();
	
	if( Usb.getUsbTaskState() == USB_DETACHED_SUBSTATE_INITIALIZE) {  //reinitialised
		ps3_status = 0;
    
    }


	if( Usb.getUsbTaskState() == USB_STATE_CONFIGURING ) {  //wait for addressing state
		
        PS3_init();
    if( ps3_status & statusPS3Connected)  Usb.setUsbTaskState( USB_STATE_RUNNING );
    }
    if( Usb.getUsbTaskState() == USB_STATE_RUNNING ) {  //poll the PS3 Controller 
        PS3_poll();
    }
	

}

/* Initialize PS3 Controller */
void PS3_USB::PS3_init( void )
{
 byte rcode = 0;  //return code
 char buf[ 64 ] = { 0 };      //General purpose buffer for usb data
 byte i;
 USB_DEVICE_DESCRIPTOR* device_descriptor;
 
 
	ps3_status = statusDeviceConnected;;

 /* Initialize data structures for endpoints of device */
    ep_record[ CONTROL_PIPE ] = *( Usb.getDevTableEntry( 0,0 ));  //copy endpoint 0 parameters
    ep_record[ OUTPUT_PIPE ].epAddr = 0x02;    // PS3 output endpoint
    ep_record[ OUTPUT_PIPE ].Attr  = EP_INTERRUPT;
    ep_record[ OUTPUT_PIPE ].MaxPktSize = EP_MAXPKTSIZE;
    ep_record[ OUTPUT_PIPE ].Interval  = EP_POLL;
    ep_record[ OUTPUT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record[ OUTPUT_PIPE ].rcvToggle = bmRCVTOG0;
    ep_record[ INPUT_PIPE ].epAddr = 0x01;    // PS3 report endpoint
    ep_record[ INPUT_PIPE ].Attr  = EP_INTERRUPT;
    ep_record[ INPUT_PIPE ].MaxPktSize = EP_MAXPKTSIZE;
    ep_record[ INPUT_PIPE ].Interval  = EP_POLL;
    ep_record[ INPUT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record[ INPUT_PIPE ].rcvToggle = bmRCVTOG0;
    
    Usb.setDevTableEntry( PS3_ADDR, ep_record );              //plug kbd.endpoint parameters to devtable
    delay(200); // give time for address change
    /* read the device descriptor and check VID and PID*/
    rcode = Usb.getDevDescr( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, DEV_DESCR_LEN , buf );
    if( rcode ) return;
    device_descriptor = (USB_DEVICE_DESCRIPTOR *) &buf;
    if(
    (device_descriptor->idVendor != PS3_VID) ||(device_descriptor->idProduct != PS3_PID)  ) return;
    
    /* Configure device */
    rcode = Usb.setConf( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_CONFIGURATION );                    
    if( rcode ) return;
    ps3_status |= statusUSBConfigured;
 
    /* Set the PS3 controller to send reports */
    for (i=0; i < PS3_F4_REPORT_LEN; i++) buf[i] = pgm_read_byte_near( feature_F4_report + i); 
    rcode = Usb.setReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_F4_REPORT_LEN,  PS3_IF, HID_REPORT_FEATURE, PS3_F4_REPORT_ID , buf );
    if( rcode ) return;
    
    /* Set the PS3 controller LED 1 On */
    for (i=0; i < PS3_01_REPORT_LEN; i++) buf[i] = pgm_read_byte_near( output_01_report + i); 
    rcode = Usb.setReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_01_REPORT_LEN,  PS3_IF, HID_REPORT_OUTPUT, PS3_01_REPORT_ID , buf );
    if( rcode ) return;
    
    ps3_status |= statusPS3Connected;

    
}



/* Poll PS3 interrupt pipe and process result if any */

void PS3_USB::PS3_poll( void )
{
 
 byte rcode = 0;     //return code
    /* poll PS3 */
    rcode = Usb.inTransfer(PS3_ADDR, ep_record[ INPUT_PIPE ].epAddr, PS3_01_REPORT_LEN, (char *) &report );
    if( rcode )  return;
    ps3_status |= statusReportReceived;
    return;
}

unsigned char PS3_USB::getStatus( void ){
    return( ps3_status);
}


bool PS3_USB::statConnected( void){
	return(ps3_status & statusPS3Connected);
}

bool PS3_USB::statReportReceived( void){
	bool rvalue = ps3_status & statusReportReceived;
	ps3_status &= ~statusReportReceived; // clear the report received flag
	return(rvalue);
}

unsigned char PS3_USB::getJoystick(unsigned char joy){
	if (joy > 3) return(128); // not valid joystick
    return( *(&report.LeftStickX + joy));
}

unsigned int PS3_USB::getMotion(unsigned char accel ){
	if(accel > 3) return(512);
    return( byteswap(*(&report.AccelX + accel)));
}

unsigned char PS3_USB::getPressure(unsigned char button ){
	if ((button < 4) || (button > 15)) return(0);
    return(*(&report.PressureUp + button - 4));
}

bool PS3_USB::buttonChanged(void){
	if ((report.ButtonState != oldbuttonstate) || (report.PSButtonState != oldpsbuttonstate)){
	oldbuttonstate = report.ButtonState;
	oldpsbuttonstate = report.PSButtonState;
	return(true);
	}
	return(false);
}

bool PS3_USB::buttonPressed(unsigned char button){
	if(button == 16) return( report.PSButtonState & 0x01);
	return(report.ButtonState & (1 << button));
	
}

void PS3_USB::LEDRumble(unsigned char ledrum){
unsigned char i;
char buf[ 64 ] = { 0 };      //General purpose buffer for usb data
/* default buffer values */
      for (i=0; i < PS3_01_REPORT_LEN; i++) buf[i] = pgm_read_byte_near( output_01_report + i); 
      /* LED setings */
      buf[9] = (ledrum & 0x0f) << 1;
      /* rumble settings */
      if (ledrum & 0x30){
        buf[1] = buf[3] = 0xfe;
        if (ledrum & 0x10) buf[4] = 0xff;
        else buf[2] = 0xff;
      }
	       
      Usb.setReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_01_REPORT_LEN,  PS3_IF, HID_REPORT_OUTPUT, PS3_01_REPORT_ID , buf );

	return;
}

void PS3_USB::setBDADDR(unsigned char * bdaddr){
	char buf[ PS3_F5_REPORT_LEN ];
    buf[0] = 0x01;
    buf[1] = 0x00;
    for (int i=0; i < 6; i++){
        buf[i+2] = bdaddr[i];
    }
    Usb.setReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_F5_REPORT_LEN,  PS3_IF, HID_REPORT_FEATURE, PS3_F5_REPORT_ID , buf ); 
	return;
}

void PS3_USB::getBDADDR(unsigned char * bdaddr){
	char buf[ PS3_F5_REPORT_LEN ];
	Usb.getReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_F5_REPORT_LEN,  PS3_IF, HID_REPORT_FEATURE, PS3_F5_REPORT_ID , buf );
    for( int i=0; i < 6; i++){
        bdaddr[i] = buf[i + 2];
    }
	return;
}