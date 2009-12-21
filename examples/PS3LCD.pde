

/* MAX3421E USB Host controller LCD/PS3 demonstration */
#include <Spi.h>
#include <Max3421e.h>
#include <Usb.h>
#include <Max_LCD.h>
#include <MemoryFree.h>
#include <avr/pgmspace.h>

#define byteswap(x) ((x >> 8) | (x << 8))

/*The application will work in reduced host mode, so we can save program and data
memory space. After verifying the PID and VID we will use known values for the 
configuration values for device, interface, endpoints and HID */

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

/* various parameters for PS3 Game controller reports */
#define PS3_F4_REPORT_LEN 4
#define PS3_F5_REPORT_LEN 8
#define PS3_01_REPORT_LEN 48
#define HID_REPORT_FEATURE 3 
#define HID_REPORT_OUTPUT  2
#define PS3_F4_REPORT_ID  0xF4
#define PS3_01_REPORT_ID  0x01
#define PS3_F5_REPORT_ID 0xF5

/* Defines for the PS3 Buttons in the HID Report
*/

#define buttonchange (PS3Report->ButtonState != oldbuttons ) // true if any button changed
#define buSelect    (PS3Report->ButtonState & 0x0001)
#define buLAnalog   (PS3Report->ButtonState & 0x0002)
#define buRAnalog   (PS3Report->ButtonState & 0x0004)
#define buStart     (PS3Report->ButtonState & 0x0008)
#define buUp        (PS3Report->ButtonState & 0x0010)
#define buRight     (PS3Report->ButtonState & 0x0020)
#define buDown      (PS3Report->ButtonState & 0x0040)
#define buLeft      (PS3Report->ButtonState & 0x0080)
#define buL2        (PS3Report->ButtonState & 0x0100)
#define buR2        (PS3Report->ButtonState & 0x0200)
#define buL1        (PS3Report->ButtonState & 0x0400)
#define buR1        (PS3Report->ButtonState & 0x0800)
#define buTriangle  (PS3Report->ButtonState & 0x1000)
#define buCircle    (PS3Report->ButtonState & 0x2000)
#define buCross     (PS3Report->ButtonState & 0x4000)
#define buSquare    (PS3Report->ButtonState & 0x8000)
#define buPS        (PS3Report->PSButtonState & 0x01)


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
    unsigned int AccelerometerX;          // X axis accelerometer Big Endian 0 - 1023
    unsigned int AccelerometerY;          // Y axis accelerometer Big Endian 0 - 1023
    unsigned int AccelerometerZ;          // Z axis accelerometer Big Endian 0 - 1023
    unsigned int GyrometerX;          // Z axis Gyro Big Endian 0 - 1023
    
} TYPE_01_REPORT;


/*Menu screens
Assign values for different menu screns
*/

#define Menu_Root    0
#define Menu_Basic   1
#define Menu_Buttons 2
#define Menu_Joystick 3
#define Menu_Pressure 4
#define Menu_Accelerometer 5
#define Menu_LED 6
#define Menu_Bdaddr 7
#define Menu_Freememory 8


/* Menu Text
Located in Flash to save data RAM
*/

prog_char menutext_0[] PROGMEM = "Select Test";
prog_char menutext_1[] PROGMEM = "Basic Tests";  
prog_char menutext_2[] PROGMEM = "Buttons Test";
prog_char menutext_3[] PROGMEM = "Joystick Test";
prog_char menutext_4[] PROGMEM = "Pressure Test";
prog_char menutext_5[] PROGMEM = "Motion Test";
prog_char menutext_6[] PROGMEM = "LED/Rumble Test";
prog_char menutext_7[] PROGMEM = "Bluetooth Addr";
prog_char menutext_8[] PROGMEM = "Free Memory";

// Pointers to flash text strings
PROGMEM const char *menu_table[] = 	  
{   
  menutext_0,
  menutext_1,
  menutext_2,
  menutext_3,
  menutext_4,
  menutext_5,
  menutext_6,
  menutext_7,
  menutext_8 };
  

prog_char output_01_report[] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x02, 0xff, 0x27, 0x10, 0x00, 0x32, 0xff, 
                                     0x27, 0x10, 0x00, 0x32, 0xff, 0x27, 0x10, 0x00, 
                                     0x32, 0xff, 0x27, 0x10, 0x00, 0x32, 0x00, 0x00, 
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

prog_char feature_F4_report[] PROGMEM = {0x42, 0x0c, 0x00, 0x00};

EP_RECORD ep_record[ PS3_NUM_EP ];  //endpoint record structure for the PS3 controller

char buf[ 64 ] = { 0 };      //General purpose buffer for usb data
unsigned int oldbuttons;
char screen, selscreen;
char lcdbuffer[17];
unsigned char bdaddr[6];
char bdcursor;
char ledrum;
char lrcursor;


void setup();
void loop();

MAX3421E Max;
USB Usb;
Max_LCD LCD;

void setup() {
  // set up the LCD's number of rows and columns: 
  LCD.begin(16, 2);
  LCD.home();
  LCD.print("PS3 Controller");
  LCD.setCursor(0,1);
  LCD.print("Wait for connect");
  Serial.begin( 9600 );
  Serial.println("PS3 Controller Start");
  Serial.print("freeMemory() reports ");
  Serial.println( freeMemory() );
  Max.powerOn();
  delay(200);
}

void loop() {
  

    Max.Task();
    Usb.Task();
    if( Usb.getUsbTaskState() == USB_STATE_CONFIGURING ) {  //wait for addressing state
        PS3_init();
        process_report();
        Usb.setUsbTaskState( USB_STATE_RUNNING );
    }
    if( Usb.getUsbTaskState() == USB_STATE_RUNNING ) {  //poll the PS3 Controller 
        PS3_poll();
    }
    delay(100); // Test to see how often we need to poll
}
/* Initialize PS3 Controller */
void PS3_init( void )
{
 byte rcode = 0;  //return code
 byte i;
 USB_DEVICE_DESCRIPTOR* device_descriptor;

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
    
    /* read the device descriptor and check VID and PID*/
    rcode = Usb.getDevDescr( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, DEV_DESCR_LEN , buf );
    if( rcode ) {
        Serial.print("Error attempting read device descriptor. Return code :");
        Serial.println( rcode, HEX );
        while(1);  //stop
    }
    device_descriptor = (USB_DEVICE_DESCRIPTOR *) &buf;
    if(
    (device_descriptor->idVendor != PS3_VID) ||(device_descriptor->idProduct != PS3_PID)  ) {
        Serial.println("Unsupported USB Device");
          while(1);  //stop   
    }
    
    /* Configure device */
    rcode = Usb.setConf( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_CONFIGURATION );                    
    if( rcode ) {
        Serial.print("Error attempting to configure PS3 controller. Return code :");
        Serial.println( rcode, HEX );
        while(1);  //stop
    }
    
 
    /* Set the PS3 controller to send reports */
    for (i=0; i < PS3_F4_REPORT_LEN; i++) buf[i] = pgm_read_byte_near( feature_F4_report + i); 
    rcode = Usb.setReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_F4_REPORT_LEN,  PS3_IF, HID_REPORT_FEATURE, PS3_F4_REPORT_ID , buf );
    if( rcode ) {
        Serial.print("Set report error: ");
        Serial.println( rcode, HEX );
        while(1);  //stop
       
    }
    
    /* Set the PS3 controller LED 1 On */
    for (i=0; i < PS3_01_REPORT_LEN; i++) buf[i] = pgm_read_byte_near( output_01_report + i); 
    rcode = Usb.setReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_01_REPORT_LEN,  PS3_IF, HID_REPORT_OUTPUT, PS3_01_REPORT_ID , buf );
    if( rcode ) {
        Serial.print("Set report error: ");
        Serial.println( rcode, HEX );
        while(1);  //stop
       
    }
    
    LCD.print("PS3 initialized");
    Serial.println("PS3 initialized");
    delay(200);
    screen = Menu_Root;
    selscreen = Menu_Basic;
    LCD.clear();
    LCD.home();
    LCD.print("Main Menu");
    LCD.setCursor(0,1);
    strcpy_P(lcdbuffer, (char*)pgm_read_word(&(menu_table[selscreen]))); 
    LCD.print(lcdbuffer);

    
}

/* Poll PS3 interrupt pipe and process result if any */

void PS3_poll( void )
{
 
 byte rcode = 0;     //return code
    /* poll PS3 */
    rcode = Usb.inTransfer(PS3_ADDR, ep_record[ INPUT_PIPE ].epAddr, PS3_01_REPORT_LEN, buf );
    if( rcode != 0 ) {
       return;
    }
    process_report();
    return;
}

void process_report(void)
{
  byte i, j;
  unsigned int  mask;
  TYPE_01_REPORT *  PS3Report = (TYPE_01_REPORT *) &buf;
  if(buPS){
    screen = Menu_Root;
    selscreen = Menu_Basic;
    LCD.clear();
    LCD.home();
    LCD.print("Main Menu");
    LCD.setCursor(0,1);
    LCD.noCursor();
    strcpy_P(lcdbuffer, (char*)pgm_read_word(&(menu_table[selscreen]))); 
    LCD.print(lcdbuffer);
    oldbuttons = PS3Report->ButtonState;
    

  }
  
  switch (screen){
    
    case Menu_Root:     
      if(buttonchange){
        if(buDown) selscreen--;
        else if(buUp | buSelect) selscreen++;
        else if(buStart) {
          screen = selscreen;
          LCD.clear();
          oldbuttons = PS3Report->ButtonState;
          break;
        }
        else {
          oldbuttons = PS3Report->ButtonState;
          break;
          
        }
        if (selscreen == 0) selscreen = 1;
        if (selscreen > 8) selscreen = 1;
        LCD.clear();
        LCD.home();
        LCD.print("Main Menu:");
        LCD.setCursor(0,1);
        strcpy_P(lcdbuffer, (char*)pgm_read_word(&(menu_table[selscreen]))); 
        LCD.print(lcdbuffer);
        oldbuttons = PS3Report->ButtonState;
      }
      break;
    
    case Menu_Basic:
      if(buttonchange){
        LCD.home();
        if (buL1) LCD.print('X');
        else LCD.print(' ');
        LCD.print("  Test L/R");
        LCD.setCursor(0,1);
        if (buL2) LCD.print('X');
        else LCD.print(' ');
        LCD.print("  Buttons");
        LCD.setCursor(15,0);
        if (buR1) LCD.print('X');
        else LCD.print(' ');
        
        LCD.setCursor(15,1);
        if (buR2) LCD.print('X');
        else LCD.print(' ');
      }
      
      break;
    
    case Menu_Buttons:  
      if(buttonchange){
        LCD.home();
        LCD.print("0123456789ABCDEF");
        LCD.setCursor(0,1);
        mask = 1;
        for( i = 0; i < 16; i++){
          if (PS3Report->ButtonState & mask) lcdbuffer[i] = '^';
          else lcdbuffer[i] = ' ';
          mask <<= 1;
        }      
        LCD.print(lcdbuffer);
        oldbuttons = PS3Report->ButtonState;
 
      }
      
      break;
    
    case Menu_Joystick:
      LCD.home();
      LCD.print('^');
      LCD.print(PS3Report->LeftStickY, DEC);
      LCD.print("  ");
      LCD.setCursor(8,0);
      LCD.print('^');
      LCD.print(PS3Report->RightStickY, DEC);
      LCD.print("  ");
      LCD.setCursor(0,1);
      LCD.print('>');
      LCD.print(PS3Report->LeftStickX, DEC);
      LCD.print("  ");
      LCD.setCursor(8,1);
      LCD.print('>');
      LCD.print(PS3Report->RightStickX, DEC);
      LCD.print("  ");
      break;
      
    case Menu_Pressure:
      LCD.home();
      LCD.print(PS3Report->PressureUp, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureDown, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureLeft, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureRight, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureL1, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureR1, DEC);
      LCD.print("      ");
 
      LCD.setCursor(0,1);
      LCD.print(PS3Report->PressureCircle, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureTriangle, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureSquare, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureCross, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureL2, DEC);
      LCD.print(" ");
      LCD.print(PS3Report->PressureR2, DEC);
      LCD.print("      ");
     
      break;
      
    case Menu_Accelerometer:
      LCD.home();
      LCD.print('X');
      LCD.print(byteswap(PS3Report->AccelerometerX), DEC);
      LCD.print("  ");
      LCD.setCursor(8,0);
      LCD.print('Y');
      LCD.print(byteswap(PS3Report->AccelerometerY), DEC);
      LCD.print("  ");
      LCD.setCursor(0,1);
      LCD.print('Z');
      LCD.print(byteswap(PS3Report->AccelerometerZ), DEC);
      LCD.print("  ");
      LCD.setCursor(8,1);
      LCD.print('G');
      LCD.print(byteswap(PS3Report->GyrometerX), DEC);
      LCD.print("  ");
      break;
      
    case Menu_LED:
      if(buttonchange){
        oldbuttons = PS3Report->ButtonState;
        if (buRight) lrcursor++;
        else if (buLeft) lrcursor--;
        else if (buUp) ledrum |= 1 << lrcursor;
        else if (buDown) ledrum &= ~(1 << lrcursor);
        if (lrcursor > 7) lrcursor = 0;
        if (lrcursor < 0) lrcursor = 7;
      }
      LCD.home();
      LCD.print("1 2 3 4 S W ");
      LCD.setCursor(0,1);
      j = 0;
      for (i=0; i < 6; i++){
        if(ledrum & 1 << i) lcdbuffer[j] = 'X';
        else lcdbuffer[j] = ' ';
        j++; ;
        lcdbuffer[j] = ' ';
        j++;
      }
      lcdbuffer[j] = 0;
      LCD.print(lcdbuffer);
      LCD.setCursor((lrcursor * 2),1);
      LCD.cursor();
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

      
      break;
      
    case Menu_Bdaddr:
      if(buttonchange){
        oldbuttons = PS3Report->ButtonState;
 
        if (buRight) bdcursor++;
        else if (buLeft) bdcursor--;
        if (bdcursor > 11) bdcursor = 0;
        if (bdcursor < 0) bdcursor = 11;
        if(buUp){
          if(bdcursor % 2){
            if ((bdaddr[bdcursor /2] & 0x0f) == 0x0f) bdaddr[bdcursor /2] &= 0xf0;
            bdaddr[bdcursor / 2] += 0x1;           
          }
          else{
            if ((bdaddr[bdcursor /2] & 0xf0) == 0xf0) bdaddr[bdcursor /2] &= 0x0f;
            bdaddr[bdcursor / 2] += 0x10;       
          }
          
        }
        else if (buDown){
          if(bdcursor % 2){
            if ((bdaddr[bdcursor /2] & 0x0f) == 0x0) bdaddr[bdcursor /2] |= 0x0f;
            bdaddr[bdcursor / 2] -= 0x1;           
          }
          else{
            if ((bdaddr[bdcursor /2] & 0xf0) == 0x0) bdaddr[bdcursor /2] |= 0xf0;
            bdaddr[bdcursor / 2] -= 0x10;       
          }
         
         }
         if( buCross){
           buf[0] = 0x01;
           buf[1] = 0x00;
           for (i=0; i < 6; i++){
             buf[i+2] = bdaddr[i];
           }
          Serial.println( "bdaddr"); 
          Usb.setReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_F5_REPORT_LEN,  PS3_IF, HID_REPORT_FEATURE, PS3_F5_REPORT_ID , buf ); 
        }
      }
      LCD.home();
      LCD.print("R: ");
      Usb.getReport( PS3_ADDR, ep_record[ CONTROL_PIPE ].epAddr, PS3_F5_REPORT_LEN,  PS3_IF, HID_REPORT_FEATURE, PS3_F5_REPORT_ID , buf );
      for( i=0; i < 6; i++){
        if ((unsigned char)buf[i+2] < 16) LCD.print ('0');
        LCD.print((unsigned char)buf[i + 2], HEX);
      }
      
      LCD.setCursor(0,1);
      LCD.print("W: ");
      for( i=0; i < 6; i++){
        if (bdaddr[i] < 16) LCD.print ('0');
        LCD.print(bdaddr[i], HEX);
      }
      LCD.setCursor(3 + bdcursor ,1);
      LCD.cursor();
      
      break;
      
    case Menu_Freememory:
      LCD.home();
      LCD.print("Free Memory ");
      LCD.print( freeMemory(), DEC );
      LCD.setCursor(0,1);
      break;
    
    default:
      break;
      
  }
  
  return;
}




