/* MAX3421E USB Host controller Bluetooth demonstration */
#include <Spi.h>
#include <Max3421e.h>
#include <Usb.h>
#include <Max_LCD.h>
#include <MemoryFree.h>
#include <avr/pgmspace.h>

/*The application will work in reduced host mode, so we can save program and data
memory space. After verifying the PID and VID we will use known values for the 
configuration values for device, interface, endpoints and HID */

/* CSR Bluetooth data taken from descriptors */
#define BT_ADDR        1
#define CSR_VID_LO      0x12  // CSR VID
#define CSR_VID_HI      0x0a
#define CSR_PID_LO      0x01  // Bluetooth HCI Device
#define CSR_PID_HI      0x00
#define BT_CONFIGURATION 1
#define BT_INTERFACE    0 // Only use interface 0
#define BT_NUM_EP      4
#define INT_MAXPKTSIZE   16
#define BULK_MAXPKTSIZE  64
#define EP_INTERRUPT    0x03 // endpoint types
#define EP_BULK         0x02
#define EP_POLL         0x01 // interrupt poll interval

#define CONTROL_PIPE      0 // names we give to the 4 pipes
#define EVENT_PIPE        1
#define DATAIN_PIPE       2
#define DATAOUT_PIPE      3

/* Size of data buffer: keep small to save RAM */

#define MAX_BUFFER_SIZE   64 // size of general purpose data buffer

/* Bluetooth HCI states for hci_task() */
#define HCI_INIT_STATE    0
#define HCI_RESET_STATE   1
#define HCI_CONNECT_OUT_STATE 2
#define HCI_CONNECTED_STATE    3



/* HCI event flags*/
#define HCI_FLAG_CMD_COMPLETE 0x01
#define HCI_FLAG_CMD_STATUS 0x02
#define HCI_FLAG_CONN_COMPLETE 0x04
#define HCI_FLAG_CONNECT_OK 0x08
#define HCI_FLAG_DISCONN_COMPLETE 0x10



/*Macros for event flag tests */
#define hci_cmd_complete (hci_event_flag & HCI_FLAG_CMD_COMPLETE)
#define hci_cmd_status (hci_event_flag & HCI_FLAG_CMD_STATUS)
#define hci_connect_complete (hci_event_flag & HCI_FLAG_CONN_COMPLETE)
#define hci_connect_ok (hci_event_flag & HCI_FLAG_CONNECT_OK)
#define hci_disconnect_complete (hci_event_flag & HCI_FLAG_DISCONN_COMPLETE)


/* HCI Events managed */
#define EV_COMMAND_COMPLETE  0x0e
#define EV_COMMAND_STATUS    0x0f
#define EV_CONNECT_COMPLETE  0x03
#define EV_DISCONNECT_COMPLETE 0x05
#define EV_NUM_COMPLETE_PKT  0x13
#define EV_QOS_SETUP_COMPLETE 0x0d

// used in control endpoint header for HCI Commands
#define bmREQ_HCI_OUT  USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_DEVICE
#define HCI_COMMAND_REQ    0 

// HCI Macro
#define hci_timeout  hci_counter-- == 0


/* Bluetooth L2CAP states for l2cap_task() */
#define L2CAP_DOWN_STATE 0
#define L2CAP_INIT_STATE  1
#define L2CAP_CONTROL_CONNECTING_STATE 2
#define L2CAP_CONTROL_CONFIGURING_STATE 3
#define L2CAP_INTERRUPT_CONNECTING_STATE 4
#define L2CAP_INTERRUPT_CONFIGURING_STATE 5
#define L2CAP_CONNECTED_STATE 6
#define L2CAP_LED_STATE 7
#define L2CAP_READY_STATE 8
#define L2CAP_DISCONNECT_STATE 9



/* L2CAP event flags  and tests */
#define L2CAP_COMMAND_CONNECTED 0x01
#define L2CAP_COMMAND_CONFIGURED 0x02
#define L2CAP_COMMAND_CONFIG_REQ 0x04
#define L2CAP_INTERRUPT_CONNECTED 0x08
#define L2CAP_INTERRUPT_CONFIGURED 0x10
#define L2CAP_INTERRUPT_CONFIG_REQ 0x20
#define L2CAP_COMMAND_DISCONNECT_REQ 0x40
#define L2CAP_INTERRUPT_DISCONNECT_REQ 0x80

#define l2cap_command_connected (l2cap_event_status & L2CAP_COMMAND_CONNECTED)
#define l2cap_command_configured (l2cap_event_status & L2CAP_COMMAND_CONFIGURED)
#define l2cap_command_config_req (l2cap_event_status & L2CAP_COMMAND_CONFIG_REQ)
#define l2cap_interrupt_connected (l2cap_event_status & L2CAP_INTERRUPT_CONNECTED)
#define l2cap_interrupt_configured (l2cap_event_status & L2CAP_INTERRUPT_CONFIGURED)
#define l2cap_interrupt_config_req (l2cap_event_status & L2CAP_INTERRUPT_CONFIG_REQ)
#define l2cap_command_disconnected (l2cap_event_status & L2CAP_COMMAND_DISCONNECT_REQ)
#define l2cap_interrupt_disconnected (l2cap_event_status & L2CAP_INTERRUPT_DISCONNECT_REQ)




#define acl_handle_ok ((buf[0] | (buf[1] << 8)) == (hci_handle | 0x2000))
#define l2cap_control ((buf[6] | (buf[7] << 8)) == 0x01)
#define l2cap_interrupt  ((buf[6] | (buf[7] << 8)) ==  interrupt_scid)
#define l2cap_command  ((buf[6] | (buf[7] << 8)) ==  command_scid)
#define l2cap_connection_response (buf[8] ==  0x03) 
#define l2cap_configuration_response (buf[8] ==  0x05) 
#define l2cap_configuration_request (buf[8]  ==  0x04) 
#define l2cap_connection_success ((buf[16] | (buf[17] << 8)) ==  0x00) 
#define l2cap_disconnect_request (buf[8] ==  0x06) 

/* HID Flags and tests */
#define HID_FLAG_STATUS_REPORTED 0x01
#define HID_FLAG_BUTTONS_CHANGED 0x02
#define HID_FLAG_EXTENSION 0x04
#define HID_FLAG_COMMAND_SUCCESS 0x08

#define hid_status_reported (hid_flags & HID_FLAG_STATUS_REPORTED)
#define hid_buttons_changed (hid_flags & HID_FLAG_BUTTONS_CHANGED)
#define hid_extension (hid_flags & HID_FLAG_EXTENSION)
#define hid_command_success (hid_flags & HID_FLAG_COMMAND_SUCCESS)
#define hid_handshake_success (buf[8] == 0)

/* Defines for the Wiimote LEDs 
*/

#define LED1	    0x10
#define LED2	    0x20
#define LED3	    0x40
#define LED4	    0x80

/* Defines for the Wiimote Buttons 
*/

#define buttonLeft_pressed      (hid_buttons & 1 << 1)
#define buttonRight_pressed     (hid_buttons & 1 << 2)
#define buttonDown_pressed      (hid_buttons & 1 << 3)
#define buttonUp_pressed        (hid_buttons & 1 << 4)
#define buttonPlus_pressed	(hid_buttons & 1 << 5)

#define button2_pressed	   (hid_buttons & 1 << 8)
#define button1_pressed	   (hid_buttons & 1 << 9)
#define buttonB_pressed	   (hid_buttons & 1 << 10)
#define buttonA_pressed	   (hid_buttons & 1 << 11)
#define buttonMinus_pressed     (hid_buttons & 1 << 12)
#define buttonHome_pressed      (hid_buttons & 1 << 15)



/* define for Bluetooth HID 
*/
#define DATA_INPUT 0x0a1

EP_RECORD ep_record[ BT_NUM_EP ];  //endpoint record structure for the Bluetooth controller


/* Print strings in Program Memory */
const char Free_Memory_str[] PROGMEM = "\r\nfreeMemory() reports:\t"; 
const char Dev_Error_str[] PROGMEM ="\r\nDevice Descriptor Error:\t";
const char Wrong_Device_str[] PROGMEM ="\r\nWrong USB Device ID";
const char Config_Error_str[] PROGMEM ="\r\nError Setting Configuration:\t";
const char Int_Error_str[] PROGMEM ="\r\nError Setting Interface:\t";
const char CSR_Init_str[] PROGMEM ="\r\nCSR Initialized";
const char HCI_Reset_str[] PROGMEM ="\r\nHCI Reset complete";
const char Reset_Error_str[] PROGMEM ="\r\nNo response to HCI Reset ";
const char Device_Connected_str[] PROGMEM = "\r\nConnected to Wiimote";
const char HCI_Command_Failed_str[] PROGMEM ="\r\nHCI_Command_Failed: ";
const char Unmanaged_Event_str[] PROGMEM ="\r\nUnmanaged Event: ";
const char Device_Disconnected_str[] PROGMEM = "\r\nWiimote Disconnect";
const char HID_Buttons_str[] PROGMEM = "\r\nButtons Pressed ";


unsigned char buf[ MAX_BUFFER_SIZE ] = { 0 };      //General purpose buffer for usb data
/* variables used by high level HCI task */
unsigned char hci_state;  //current state of bluetooth hci connection
unsigned int  hci_counter; // counter used for bluetooth hci loops


/*variables filled from HCI event management */
unsigned int  hci_event_flag;  // flags of received bluetooth events
int  hci_handle;

/* Variables used by L2CAP*/
char l2cap_state;  // current state of l2cap connections
int  l2cap_counter;
char l2cap_event_status;
char l2cap_txid; // packet id increments for each packet sent
unsigned int command_scid = 0x40;
unsigned int interrupt_scid = 0x41;
unsigned int command_dcid, interrupt_dcid;

/* variable used by Bluetooth HID */
unsigned char hid_flags;
unsigned int hid_buttons;
unsigned int old_hid_buttons;
unsigned int hid_AccelX, hid_AccelY, hid_AccelZ;

/* Bluetooth Address of Wiimote */
unsigned char Wiimote_bdaddr[6] = { 0x00, 0x22, 0xaa, 0x8a, 0x06, 0xa3};


void setup();

void loop();

MAX3421E Max;
USB Usb;


void setup() {
  Serial.begin( 9600 );
  printProgStr(Free_Memory_str);
  Serial.print( freeMemory() );
  Max.powerOn();
  delay(200);
}

void loop() {
  

    Max.Task();
    Usb.Task();
    delay(1);
    if( Usb.getUsbTaskState() == USB_STATE_CONFIGURING ) {  //wait for addressing state
        CSR_init();
        Usb.setUsbTaskState( USB_STATE_RUNNING );
    }
    if( Usb.getUsbTaskState() == USB_STATE_RUNNING ){
      HCI_task(); //poll the HCI event pipe
      L2CAP_task(); // start polling the ACL input pipe too, though discard data until connected
    }  
    if(l2cap_state == L2CAP_READY_STATE) myapp();
  
}
/* Initialize CSR Bluetooth Controller */
void CSR_init( void )
{
 byte rcode = 0;  //return code

 
/**/

    /* Initialize data structures for endpoints of device 1*/
    ep_record[ CONTROL_PIPE ] = *( Usb.getDevTableEntry( 0,0 ));  //copy endpoint 0 parameters
    ep_record[ EVENT_PIPE ].epAddr = 0x01;    // Bluetooth event endpoint
    ep_record[ EVENT_PIPE ].Attr  = EP_INTERRUPT;
    ep_record[ EVENT_PIPE ].MaxPktSize = INT_MAXPKTSIZE;
    ep_record[ EVENT_PIPE ].Interval  = EP_POLL;
    ep_record[ EVENT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record[ EVENT_PIPE ].rcvToggle = bmRCVTOG0;
    ep_record[ DATAIN_PIPE ].epAddr = 0x02;    // Bluetoth data endpoint
    ep_record[ DATAIN_PIPE ].Attr  = EP_BULK;
    ep_record[ DATAIN_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record[ DATAIN_PIPE ].Interval  = 0;
    ep_record[ DATAIN_PIPE ].sndToggle = bmSNDTOG0;
    ep_record[ DATAIN_PIPE ].rcvToggle = bmRCVTOG0;
    ep_record[ DATAOUT_PIPE ].epAddr = 0x02;    // Bluetooth data endpoint
    ep_record[ DATAOUT_PIPE ].Attr  = EP_BULK;
    ep_record[ DATAOUT_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record[ DATAOUT_PIPE ].Interval  = 0;
    ep_record[ DATAOUT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record[ DATAOUT_PIPE ].rcvToggle = bmRCVTOG0;
    Usb.setDevTableEntry( BT_ADDR, ep_record );              //plug kbd.endpoint parameters to devtable
    
    /* read the device descriptor and check VID and PID*/
    rcode = Usb.getDevDescr( BT_ADDR, ep_record[ CONTROL_PIPE ].epAddr, DEV_DESCR_LEN , (char *) buf );
    if( rcode ) {
        printProgStr(Dev_Error_str);
        Serial.print( rcode, HEX );
        while(1);  //stop
    }
    if((buf[ 8 ] != CSR_VID_LO) || (buf[ 9 ] != CSR_VID_HI) || (buf[ 10 ] != CSR_PID_LO) || (buf[ 11 ] != CSR_PID_HI) ) {
        printProgStr(Wrong_Device_str);
          while(1);  //stop   
    }
    
    /* Configure device */
    rcode = Usb.setConf( BT_ADDR, ep_record[ CONTROL_PIPE ].epAddr, BT_CONFIGURATION );                    
    if( rcode ) {
        printProgStr(Config_Error_str);
        Serial.print( rcode, HEX );
        while(1);  //stop
    }
  

    hci_state = HCI_INIT_STATE;
    l2cap_state = L2CAP_DOWN_STATE;
    hci_counter = 10;
  //  LCD.clear();
    
    printProgStr(CSR_Init_str);
    delay(200);
    
}


void HCI_task( void )

{
 
    HCI_event_task();
    
    switch (hci_state){
      case HCI_INIT_STATE:      
      if (hci_timeout){  // wait until we have looped 10 times to clear any old events
      hci_reset();
      hci_state = HCI_RESET_STATE;
      hci_counter = 1000;
      }
      break;
      
      case HCI_RESET_STATE:
      if (hci_cmd_complete){
       hci_connect(Wiimote_bdaddr); // connect to Wiimote 
       hci_state = HCI_CONNECT_OUT_STATE; 
       hci_counter = 10000;
      }
      if (hci_timeout) {
        printProgStr(Reset_Error_str);
        hci_state = HCI_INIT_STATE;
        hci_counter = 10;
      }
      break;
      
      
      
      case HCI_CONNECT_OUT_STATE:
      if(hci_connect_complete){
        if(hci_connect_ok){
          printProgStr(Device_Connected_str);
          hci_state = HCI_CONNECTED_STATE; 
          l2cap_state = L2CAP_INIT_STATE;       
        } 
        else{
          hci_connect(Wiimote_bdaddr); // try again to connect to Wiimote  
          hci_counter = 10000; 
        }
      }
      if (hci_timeout) {
        hci_connect(Wiimote_bdaddr); // try again to connect to Wiimote  
        hci_counter = 10000;
      }
      break;
     
                     
           
      case HCI_CONNECTED_STATE:
      if(hci_disconnect_complete){
        printProgStr(Device_Disconnected_str);
        hci_state = HCI_INIT_STATE;
        l2cap_state = L2CAP_DOWN_STATE;
        hci_counter = 10; 
      }
      break;
      
      default:
      break;
   
    }

    return;
}


void HCI_event_task( void )
{
byte rcode = 0;  //return code
char char_left;
char result_pointer;
char buf_offset;
  /* check the event pipe*/
  rcode = Usb.inTransfer(BT_ADDR, ep_record[ EVENT_PIPE ].epAddr, MAX_BUFFER_SIZE, (char *) buf, USB_NAK_NOWAIT); // input on endpoint 1
  if ( !rcode){
    switch (buf[0]){            //switch on event type
    
      case EV_COMMAND_COMPLETE:
      hci_event_flag |= HCI_FLAG_CMD_COMPLETE; // set command complete flag           
        
      break;
      
    case EV_COMMAND_STATUS:
    
      hci_event_flag |= HCI_FLAG_CMD_STATUS; //set status flag
      if(buf[2]){    // show status on serial if not OK  
        printProgStr(HCI_Command_Failed_str);
        Serial.print( buf[2], HEX );
        Serial.print(' ');
        Serial.print( buf[4], HEX );
        Serial.print(' ');
        Serial.print( buf[5], HEX );
        
      }
      break;
    
    case EV_CONNECT_COMPLETE:
    
      hci_event_flag |= HCI_FLAG_CONN_COMPLETE; // set connection complete flag
      if (!buf[2]){ // check if connected OK
      hci_handle = buf[3] | buf[4] << 8; //store the handle for the ACL connection
      hci_event_flag |= HCI_FLAG_CONNECT_OK; //set connection OK flag
      }
      break;
    
    case EV_NUM_COMPLETE_PKT:
    break;
    
    case EV_QOS_SETUP_COMPLETE:
    
    break;
    
    case EV_DISCONNECT_COMPLETE:
    hci_event_flag |= HCI_FLAG_DISCONN_COMPLETE;
    break;
    
    default:
      printProgStr(Unmanaged_Event_str);
      Serial.println( buf[0], HEX );
      break;
    
    }  // switch
  }
  return;
}
void L2CAP_task(void)
{
    l2cap_event_task();
    switch (l2cap_state){
      
      case L2CAP_DOWN_STATE:
      
      break;
 
      
      case L2CAP_INIT_STATE:
      l2cap_event_status = 0;
      l2cap_connect( command_scid, 0x11);
      l2cap_state = L2CAP_CONTROL_CONNECTING_STATE;
      break;
      
      case L2CAP_CONTROL_CONNECTING_STATE:
      if(l2cap_command_connected){
        l2cap_event_status &= ~L2CAP_COMMAND_CONFIGURED;
        l2cap_configure(command_dcid);
        l2cap_state = L2CAP_CONTROL_CONFIGURING_STATE;
      }
      break;
 
      
      case L2CAP_CONTROL_CONFIGURING_STATE:
      if(l2cap_command_configured){
        l2cap_event_status &= ~L2CAP_INTERRUPT_CONNECTED;
        l2cap_connect( interrupt_scid, 0x13);
        l2cap_state = L2CAP_INTERRUPT_CONNECTING_STATE;
      }
      break;
      
      case L2CAP_INTERRUPT_CONNECTING_STATE:
      if(l2cap_interrupt_connected){
        l2cap_event_status &= ~L2CAP_INTERRUPT_CONFIGURED;
        l2cap_configure(interrupt_dcid);
        l2cap_state = L2CAP_INTERRUPT_CONFIGURING_STATE;
      }
      break;
      
      case L2CAP_INTERRUPT_CONFIGURING_STATE:
      if(l2cap_interrupt_configured){
        l2cap_state = L2CAP_CONNECTED_STATE;
      }
      break;
      
      case L2CAP_CONNECTED_STATE:
      hid_flags = 0;
      hid_led(LED1);
      l2cap_state = L2CAP_LED_STATE;
      break;
      
      case L2CAP_LED_STATE:
      if(hid_command_success){
        hid_report(0x37);
        l2cap_state = L2CAP_READY_STATE;
      }
      break;
      
      case L2CAP_READY_STATE:
        if(hid_status_reported){ // a status report will require reporting to be restarted
          hid_report(0x37);  
          hid_flags &= ~HID_FLAG_STATUS_REPORTED;
        }
        if(l2cap_interrupt_disconnected || l2cap_command_disconnected){
          l2cap_state = L2CAP_DISCONNECT_STATE;
        }
      
      break;

      case L2CAP_DISCONNECT_STATE:
      
      break;
      
      
      default:
      
      break;
    }
  return;
}



void hci_reset(void)
{
   hci_event_flag = 0; // clear all the flags
   buf[0] = 0x03;
   buf[1] = 0x0c;
   buf[2] = 0x00;
   HCI_Command(3 , buf);
  return;
}

void hci_connect(unsigned char * bdaddr)
{
   hci_event_flag &= ~(HCI_FLAG_CONN_COMPLETE | HCI_FLAG_CONNECT_OK);
   buf[0] = 0x05; // HCI OCF = 5
   buf[1]= 0x04; // HCI OGF = 1
   buf[2] = 0x0d; // parameter length =13
   buf[3] = *(bdaddr +5); // 6 octet bluetooth address
   buf[4] = *(bdaddr +4);
   buf[5] = *(bdaddr +3);
   buf[6] = *(bdaddr +2);
   buf[7] = *(bdaddr +1);
   buf[8] = *bdaddr;
   buf[9] = 0x18; // DM1 or DH1 may be used
   buf[10] = 0xcc; // DM3, DH3, DM5, DH5 may be used
   buf[11] = 0x01; // page repetition mode R1
   buf[12] = 0x00; // always 0
   buf[13] = 0x00; // clock offset 
   buf[14] = 0x00;  // 
   buf[15] = 0x00; //  do not allow role switch
   HCI_Command(16 , buf);
 
  return;
}



//perform HCI Command
byte HCI_Command( unsigned int nbytes, unsigned char* dataptr ) {
    hci_event_flag &= ~HCI_FLAG_CMD_COMPLETE;
    return( Usb.ctrlReq( BT_ADDR, ep_record[ CONTROL_PIPE ].epAddr, bmREQ_HCI_OUT, HCI_COMMAND_REQ, 0x00, 0x00 ,0, nbytes, (char *) dataptr ));
}


 
void l2cap_event_task(void)
{
  byte rcode = 0;  //return code 
    /* check the event pipe*/
  rcode = Usb.inTransfer(BT_ADDR, ep_record[ DATAIN_PIPE ].epAddr, MAX_BUFFER_SIZE, (char *) buf, USB_NAK_NOWAIT); // input on endpoint 2
  if ( !rcode){
    if (acl_handle_ok){
      if(l2cap_control){
        if (l2cap_connection_response){
          if(l2cap_connection_success){
            
            if ((buf[14]  | (buf[15] <<8)) == command_scid){
              command_dcid =  buf[12] | (buf[13] << 8); 
              l2cap_event_status |= L2CAP_COMMAND_CONNECTED;
            }
            else if ((buf[14]  | (buf[15] <<8)) == interrupt_scid){
              interrupt_dcid =  buf[12] | ( buf[13] << 8); 
              l2cap_event_status |= L2CAP_INTERRUPT_CONNECTED;
            }
          }
          
        }
         else if (l2cap_configuration_response){         
            if ((buf[12]  | (buf[13] <<8)) == command_scid){
              l2cap_event_status |= L2CAP_COMMAND_CONFIGURED;
            }
            else if ((buf[12]  | (buf[13] <<8)) == interrupt_scid){
               l2cap_event_status |= L2CAP_INTERRUPT_CONFIGURED;
            }
         
        }

       
        else if (l2cap_configuration_request){ 
            if ((buf[12] | (buf[13] << 8)) == command_scid){             
              l2cap_event_status |= L2CAP_COMMAND_CONFIG_REQ;
              l2cap_conf_response(buf[9], command_dcid);
            }
            else if ((buf[12] | (buf[13] << 8)) == interrupt_scid){             
              l2cap_event_status |= L2CAP_INTERRUPT_CONFIG_REQ;
              l2cap_conf_response(buf[9], interrupt_dcid);
            }
         
        }
        else if (l2cap_disconnect_request){
            if ((buf[12] | (buf[13] << 8)) == command_scid){ 
              l2cap_event_status |= L2CAP_COMMAND_DISCONNECT_REQ;
              l2cap_disc_response(buf[9], command_scid, command_dcid); 
            }
            if ((buf[12] | (buf[13] << 8)) == interrupt_scid){ 
              l2cap_event_status |= L2CAP_INTERRUPT_DISCONNECT_REQ;
              l2cap_disc_response(buf[9], command_scid, command_dcid); 
            }
        }
      } // l2cap_control
      
      else if (l2cap_interrupt){
        process_hid_interrupt();
      } // l2cap_interrupt  
      else if (l2cap_command){
        if (hid_handshake_success){
          hid_flags |= HID_FLAG_COMMAND_SUCCESS;
        }
        
      } // l2cap_interrupt  
      
    } // acl_handle_ok
   
  } //!rcode
  return;
} //l2cap_event_task

  
// Create L2CAP Control Connection
byte l2cap_connect( unsigned int scid, unsigned int psm)
{
buf[0] = hci_handle & 0xff;
buf[1]= ((hci_handle >> 8) & 0x0f) | 0x20;
buf[2] = 0x0c;// length 4..15
buf[3] = 0x00; 
buf[4] = 0x08; // L2CAP length 8.. 15
buf[5] = 0x00;
buf[6] = 0x01; // CID = 1 Signalling packet
buf[7] = 0x00;
buf[8] = 0x02; // connection request
buf[9] = l2cap_txid++; // identifier
buf[10] = 0x04; // length
buf[11] = 0x00;
buf[12] = psm & 0xff; // PSM
buf[13] = psm >> 8;
buf[14] = scid & 0xff; // Source CID
buf[15] = scid >> 8;
//2B 20 0C 00 08 00 01 00 02 07 04 00 11 00 40 00 
    return(Usb.outTransfer(BT_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, 16, (char *) buf)); // output on endpoint 2

}

byte l2cap_configure( unsigned int dcid)
{
  
// Configure L2CAP Connection
buf[0] = hci_handle & 0xff;
buf[1]= ((hci_handle >> 8) & 0x0f) | 0x20;
buf[2] = 0x10;// length 
buf[3] = 0x00; 
buf[4] = 0x0c; // L2CAP length 8.. 15
buf[5] = 0x00;
buf[6] = 0x01; // CID = 1 Signalling packet
buf[7] = 0x00;
buf[8] = 0x04; // configure request
buf[9] = l2cap_txid++; // identifier
buf[10] = 0x08; // length
buf[11] = 0x00;
buf[12] = dcid & 0xff;
buf[13] = dcid >> 8;
buf[14] = 0x00;
buf[15] = 0x00;
buf[16] = 0x01;
buf[17] = 0x02;
buf[18] = 0xA0;
buf[19] = 0x02;

//2B 20 10 00 0C 00 01 00 04 0A 08 00 4B 00 00 00 01 02 A0 02


    return(Usb.outTransfer(BT_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, 20, (char *) buf)); // output on endpoint 2
 
}


byte l2cap_conf_response(char rxid, unsigned int dcid)
{
  
// Respond to L2CAP Configuration request
buf[0] = hci_handle & 0xff;
buf[1]= ((hci_handle >> 8) & 0x0f) | 0x20;
buf[2] = 0x0e;// length 
buf[3] = 0x00; 
buf[4] = 0x0a; // L2CAP length
buf[5] = 0x00;
buf[6] = 0x01; // CID = 1 Signalling packet
buf[7] = 0x00;
buf[8] = 0x05; // configure response
buf[9] = rxid; // identifier
buf[10] = 0x06; // length
buf[11] = 0x00;
buf[12] = dcid & 0xff;
buf[13] = dcid >> 8;
buf[14] = 0x00;
buf[15] = 0x00;
buf[16] = 0x00;
buf[17] = 0x00;

//2B 20 0E 00 0A 00 01 00 05 10 06 00 4A 00 00 00 00 00 


    return(Usb.outTransfer(BT_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, 18, (char *)buf)); // output on endpoint 2
 
}

byte l2cap_disc_response(char rxid, unsigned int scid, unsigned int dcid)
{
  
// Respond to L2CAP Disconnect request
buf[0] = hci_handle & 0xff;
buf[1]= ((hci_handle >> 8) & 0x0f) | 0x20;
buf[2] = 0x0c;// length 
buf[3] = 0x00; 
buf[4] = 0x08; // L2CAP length
buf[5] = 0x00;
buf[6] = 0x01; // CID = 1 Signalling packet
buf[7] = 0x00;
buf[8] = 0x07; // configure response
buf[9] = rxid; // identifier
buf[10] = 0x04; // length
buf[11] = 0x00;
buf[12] = dcid & 0xff;
buf[13] = dcid >> 8;
buf[14] = scid & 0xff;
buf[15] = scid >> 8;





    return(Usb.outTransfer(BT_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, 16, (char *) buf)); // output on endpoint 2
 
}



byte hid_led( unsigned char led)
{
  
// Set HID report led 1 on
buf[0] = hci_handle & 0xff;
buf[1]= ((hci_handle >> 8) & 0x0f) | 0x20;
buf[2] = 0x07;// length 
buf[3] = 0x00; 
buf[4] = 0x03; 
buf[5] = 0x00;
buf[6] = command_dcid & 0xff;
buf[7] = command_dcid >> 8;
buf[8] = 0x52; // hid set report
buf[9] = 0x11; // set LED 1 on
buf[10] = led; 


hid_flags &= ~HID_FLAG_COMMAND_SUCCESS;

    return(Usb.outTransfer(BT_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, 11, (char *) buf)); // output on endpoint 2
 
}

byte hid_report( unsigned char report)
{
  
// Set HID report number
buf[0] = hci_handle & 0xff;
buf[1]= ((hci_handle >> 8) & 0x0f) | 0x20;
buf[2] = 0x08;// length 
buf[3] = 0x00; 
buf[4] = 0x04; 
buf[5] = 0x00;
buf[6] = command_dcid & 0xff;
buf[7] = command_dcid >> 8;
buf[8] = 0x52; // hid set report
buf[9] = 0x12; // set report no.
buf[10] = 0x00; 
buf[11] = report; 

hid_flags &= ~HID_FLAG_COMMAND_SUCCESS;



    return(Usb.outTransfer(BT_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, 12, (char *) buf)); // output on endpoint 2
 
}

void process_hid_interrupt(void)
{
  if(buf[8] == DATA_INPUT){
    if(buf[9] == 0x37){ //buttons, accel, extension report
      hid_buttons = (buf[10] & 0x9f) | (buf[11] & 0x9f) << 8;
      if(hid_buttons != old_hid_buttons){
        hid_flags |= HID_FLAG_BUTTONS_CHANGED;
        old_hid_buttons = hid_buttons;
      }
      hid_AccelX = (buf[10] & 0x60) >> 5 | buf[12] << 2;
      hid_AccelY = (buf[11] & 0x20) >> 4 | buf[13] << 2;
      hid_AccelZ = (buf[11] & 0x40) >> 5 | buf[14] << 2;

    }
    if(buf[9] == 0x20){ //status report
      hid_flags |= HID_FLAG_STATUS_REPORTED;     
    }
  }
}

// Print a string from Program Memory directly to save RAM 
void printProgStr(const prog_char str[])
{
  char c;
  if(!str) return;
  while((c = pgm_read_byte(str++)))
    Serial.print(c,BYTE);
}

// My application

void myapp(void)
{
  if(hid_buttons_changed){
      hid_flags &= ~HID_FLAG_BUTTONS_CHANGED;
      printProgStr(HID_Buttons_str);
      Serial.print(hid_buttons, HEX);
      if(button1_pressed){
        Serial.print("\r\nAccelerometers ");
        Serial.print(hid_AccelX, HEX);
        Serial.print(" ");
        Serial.print(hid_AccelY, HEX);
        Serial.print(" ");
        Serial.print(hid_AccelZ, HEX); 
      }
 
  }
  
}
