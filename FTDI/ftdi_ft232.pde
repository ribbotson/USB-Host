/* MAX3421E USB Host controller FTDI demonstration */
/* Support for FTDI Serial Interface devices*/
#include <Spi.h>
#include <Max3421e.h>
#include <Usb.h>
#include <MemoryFree.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

/*The application will work in reduced host mode, so we can save program and data
memory space. After verifying the PID and VID we will use known values for the 
configuration values for device, interface, endpoints and HID */

/* FTDI data taken from descriptors */
#define FTDI_ADDR        1
#define FTDI_VID        0x0403  // FTDI VID
#define FTDI_PID        0x6001  // FTDI PID
#define FTDI_CONFIGURATION 1
#define FTDI_INTERFACE    0 // Only use interface 0
#define FTDI_NUM_EP      3
#define BULK_MAXPKTSIZE  64
#define EP_BULK         0x02
#define USB_NAK_NOWAIT       1

#define CONTROL_PIPE      0 // names we give to the 3 pipes
#define DATAIN_PIPE       1
#define DATAOUT_PIPE      2

/* Data Buffers: */

#define GP_BUFFER_SIZE   64 // size of general purpose data buffer
#define TX_BUFFER_SIZE   64 // size of transmit buffer
#define RX_BUFFER_SIZE   64 // size of recieve buffer


// used in control endpoint header for FTDI Commands
#define bmREQ_FTDI_OUT  0x40
#define bmREQ_FTDI_IN   0xc0


// Defines for FT232 Types
// Note: I have only tried this on BM and R devices
// I think 2232 and later may also have different PID
#define FT232AM 0x0200
#define FT232BM 0x0400
#define FT2232  0x0500
#define FT232R  0x0600

/* Commands */
#define FTDI_SIO_RESET 		0 /* Reset the port */
#define FTDI_SIO_MODEM_CTRL 	1 /* Set the modem control register */
#define FTDI_SIO_SET_FLOW_CTRL	2 /* Set flow control register */
#define FTDI_SIO_SET_BAUD_RATE	3 /* Set baud rate */
#define FTDI_SIO_SET_DATA	4 /* Set the data characteristics of the port */
#define FTDI_SIO_GET_MODEM_STATUS	5 /* Retrieve current value of modern status register */
#define FTDI_SIO_SET_EVENT_CHAR	6 /* Set the event character */
#define FTDI_SIO_SET_ERROR_CHAR	7 /* Set the error character */

#define FTDI_SIO_RESET_SIO 0
#define FTDI_SIO_RESET_PURGE_RX 1
#define FTDI_SIO_RESET_PURGE_TX 2

#define FTDI_SIO_SET_DATA_PARITY_NONE (0x0 << 8 )
#define FTDI_SIO_SET_DATA_PARITY_ODD (0x1 << 8 )
#define FTDI_SIO_SET_DATA_PARITY_EVEN (0x2 << 8 )
#define FTDI_SIO_SET_DATA_PARITY_MARK (0x3 << 8 )
#define FTDI_SIO_SET_DATA_PARITY_SPACE (0x4 << 8 )
#define FTDI_SIO_SET_DATA_STOP_BITS_1 (0x0 << 11 )
#define FTDI_SIO_SET_DATA_STOP_BITS_15 (0x1 << 11 )
#define FTDI_SIO_SET_DATA_STOP_BITS_2 (0x2 << 11 )
#define FTDI_SIO_SET_BREAK (0x1 << 14)

#define FTDI_SIO_SET_DTR_MASK 0x1
#define FTDI_SIO_SET_DTR_HIGH ( 1 | ( FTDI_SIO_SET_DTR_MASK  << 8))
#define FTDI_SIO_SET_DTR_LOW  ( 0 | ( FTDI_SIO_SET_DTR_MASK  << 8))
#define FTDI_SIO_SET_RTS_MASK 0x2
#define FTDI_SIO_SET_RTS_HIGH ( 2 | ( FTDI_SIO_SET_RTS_MASK << 8 ))
#define FTDI_SIO_SET_RTS_LOW ( 0 | ( FTDI_SIO_SET_RTS_MASK << 8 ))

#define FTDI_SIO_DISABLE_FLOW_CTRL 0x0 
#define FTDI_SIO_RTS_CTS_HS (0x1 << 8)
#define FTDI_SIO_DTR_DSR_HS (0x2 << 8)
#define FTDI_SIO_XON_XOFF_HS (0x4 << 8)


#define FTDI_SIO_CTS_MASK 0x10
#define FTDI_SIO_DSR_MASK 0x20
#define FTDI_SIO_RI_MASK  0x40
#define FTDI_SIO_RLSD_MASK 0x80


EP_RECORD ep_record[ FTDI_NUM_EP ];  //endpoint record structure for the Bluetooth controller

/* Print strings in Program Memory */
const char Free_Memory_str[] PROGMEM = "\r\nfreeMemory() reports:\t"; 
const char Dev_Error_str[] PROGMEM ="\r\nDevice Descriptor Error:\t";
const char Wrong_Device_str[] PROGMEM ="\r\nWrong USB Device ID";
const char Config_Error_str[] PROGMEM ="\r\nError Setting Configuration:\t";
const char Int_Error_str[] PROGMEM ="\r\nError Setting Interface:\t";
const char FTDI_Init_str[] PROGMEM ="\r\nFT232 Initialized";
const char FT232_Type_str[] PROGMEM ="\r\nFT232 Chip Type: ";
const char Divisor_str[] PROGMEM ="\r\nFT232 Divisors: ";


unsigned char gpbuf[ GP_BUFFER_SIZE ] = { 0 };      //General purpose buffer for usb data
unsigned int FT232_type;

unsigned char txbuf [TX_BUFFER_SIZE] = { 0 };
unsigned char tx_in_pointer;
unsigned char modem_status;
unsigned char line_status;
unsigned char rxbuf[ RX_BUFFER_SIZE] = { 0 };
unsigned char rx_in_pointer;
unsigned char rx_out_pointer;
boolean interrupt_critical = false;

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
  SetUSBInterruptMode();
}

void loop() {
  int r, t;

    while ( Usb.getUsbTaskState() != USB_STATE_CONFIGURING );  //wait for addressing state


    FTDI_init();
    FTDI_baud(115200);
    Usb.setUsbTaskState( USB_STATE_RUNNING );
    FTDI_set_control(FTDI_SIO_SET_DTR_HIGH);
    
    while(Usb.getUsbTaskState() == USB_STATE_RUNNING)
    {
      r = Serial.read();
      if(r > 0) FTDI_write(r);
      t = FTDI_read();
      if(t > 0) Serial.write(t);
      
    }
 
}
/* Initialize FTDI device */
void FTDI_init( void )
{
   byte rcode = 0;  //return code
   USB_DEVICE_DESCRIPTOR* device_descriptor;
 
/**/

    /* Initialize data structures for endpoints of device 1*/
    ep_record[ CONTROL_PIPE ] = *( Usb.getDevTableEntry( 0,0 ));  //copy endpoint 0 parameters
    ep_record[ DATAOUT_PIPE ].epAddr = 0x02;    // FTDI Output endpoint
    ep_record[ DATAOUT_PIPE ].Attr  = EP_BULK;
    ep_record[ DATAOUT_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record[ DATAOUT_PIPE ].Interval  = 0;
    ep_record[ DATAOUT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record[ DATAOUT_PIPE ].rcvToggle = bmRCVTOG0;
    ep_record[ DATAIN_PIPE ].epAddr = 0x01;    // FTDI Input endpoint
    ep_record[ DATAIN_PIPE ].Attr  = EP_BULK;
    ep_record[ DATAIN_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record[ DATAIN_PIPE ].Interval  = 0;
    ep_record[ DATAIN_PIPE ].sndToggle = bmSNDTOG0;
    ep_record[ DATAIN_PIPE ].rcvToggle = bmRCVTOG0;
    
    Usb.setDevTableEntry( FTDI_ADDR, ep_record );              //plug kbd.endpoint parameters to devtable
    
    /* read the device descriptor and check VID and PID*/
    interrupt_critical = true;
    rcode = Usb.getDevDescr( FTDI_ADDR, ep_record[ CONTROL_PIPE ].epAddr, DEV_DESCR_LEN , (char *) gpbuf );
    if( rcode ) {
        printProgStr(Dev_Error_str);
        Serial.print( rcode, HEX );
        while(1);  //stop
    }
    device_descriptor = (USB_DEVICE_DESCRIPTOR *) &gpbuf;
    if((device_descriptor->idVendor != FTDI_VID) || (device_descriptor->idProduct != FTDI_PID))
    {
        printProgStr(Wrong_Device_str);
        while(1);  //stop 
    }
    else {
      //determine FT232 device type
      // 0x0200 = A, 0x0400 = B. 0x0500 = 2232, 0x0600 = R
      FT232_type = device_descriptor-> bcdDevice;
      printProgStr(FT232_Type_str);
      Serial.print(FT232_type,HEX);  
    }
    
    /* Configure device */
    rcode = Usb.setConf( FTDI_ADDR, ep_record[ CONTROL_PIPE ].epAddr, FTDI_CONFIGURATION );                    
    if( rcode ) {
        printProgStr(Config_Error_str);
        Serial.print( rcode, HEX );
        while(1);  //stop
    }
   interrupt_critical = false;
   /* Setup the tx and rx buffers */
   tx_in_pointer = 0;
   rx_in_pointer = 2;
   rx_out_pointer = 2;
  

    
    printProgStr(FTDI_Init_str);
    delay(200);
    
}

void FTDI_baud(long baud)
{
  unsigned int baud_value, baud_index = 0;
  unsigned long divisor3;
  if (FT232_type == FT232AM)
  
  {
    divisor3 = 48000000 / 2 / baud; // divisor shifted 3 bits to the left
    if ((divisor3 & 0x7) == 7) divisor3 ++; // round x.7/8 up to x+1
	baud_value = divisor3 >> 3;
	divisor3 &= 0x7;
	if (divisor3 == 1) baud_value |= 0xc000; else // 0.125
	if (divisor3 >= 4) baud_value |= 0x4000; else // 0.5
	if (divisor3 != 0) baud_value |= 0x8000;      // 0.25
	if (baud_value == 1) baud_value = 0;	/* special case for maximum baud rate */
  }
  else
  {
    static const unsigned char divfrac[8] = { 0, 3, 2, 0, 1, 1, 2, 3 };
    static const unsigned char divindex[8] = {0, 0, 0, 1, 0, 1, 1, 1 };
	divisor3 = 48000000 / 2 / baud; // divisor shifted 3 bits to the left
	baud_value = divisor3 >> 3;
	baud_value |= divfrac[divisor3 & 0x7] << 14;
        baud_index = divindex[divisor3 & 0x7];
	/* Deal with special cases for highest baud rates. */
	if (baud_value == 1) baud_value = 0; else	// 1.0
	if (baud_value == 0x4001) baud_value = 1;	// 1.5
  }
  printProgStr(Divisor_str);

  Serial.print(baud_value, HEX);
  Serial.print(" ");
  Serial.print(baud_index, HEX);
  interrupt_critical = true;
  Usb.ctrlReq( FTDI_ADDR, ep_record[ CONTROL_PIPE ].epAddr, bmREQ_FTDI_OUT, FTDI_SIO_SET_BAUD_RATE, baud_value & 0xff, baud_value >> 8 ,baud_index, 0, NULL );
  interrupt_critical = false;
}

void FTDI_set_control(int signal)
{
  interrupt_critical = true;  
  Usb.ctrlReq( FTDI_ADDR, ep_record[ CONTROL_PIPE ].epAddr, bmREQ_FTDI_OUT, FTDI_SIO_MODEM_CTRL, signal & 0xff, signal >> 8 ,0, 0, NULL );
  interrupt_critical = false;
}

int FTDI_read(void)
{
  unsigned int rx_size = RX_BUFFER_SIZE;
  unsigned char rcode;
    // do we have any characters in local rx buffer
    // if not then in transfer to see if any in FTDI chip
  if (rx_in_pointer == rx_out_pointer)
  {
    interrupt_critical = true;
    rcode = Usb.inTransfern(FTDI_ADDR, ep_record[ DATAIN_PIPE ].epAddr, &rx_size, (char *) rxbuf, USB_NAK_NOWAIT);
 
    if (!rcode)
    {
      modem_status = rxbuf[0];
      line_status = rxbuf[1];
      if (rx_size > 2)
      {
        rx_in_pointer = rx_size;
        rx_out_pointer = 2;
        unsigned char c = rxbuf[rx_out_pointer++];
        interrupt_critical = false;
        return c; 
      }
      
    }
    interrupt_critical = false;
    return -1;
  } 
  else 
  {
    unsigned char c = rxbuf[rx_out_pointer++];
    return c;
  }
}

void FTDI_write(unsigned char c)
{
  txbuf[tx_in_pointer++] = c;
  if(tx_in_pointer == TX_BUFFER_SIZE)
  {
    interrupt_critical = true;
    Usb.outTransfer(FTDI_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, tx_in_pointer , (char *) txbuf); // output on endpoint 2
    tx_in_pointer = 0;
    interrupt_critical = false;
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

// Set Interrupt Mode

void SetUSBInterruptMode(void)
{
  //GPX pin is input
  pinMode(MAX_GPX, INPUT);  
  
  // Set the Max3421e to send SOF pulses on GPX pin
  Max.regWr(rPINCTL, (bmFDUPSPI + bmINTLEVEL + bmGPXB + bmGPXA));
  
  // set up for pin interrupt
  // Pin change interrupt control register - enables interrupt vectors
  // Bit 2 = enable PC vector 2 (PCINT23..16)
  // Bit 1 = enable PC vector 1 (PCINT14..8)
  // Bit 0 = enable PC vector 0 (PCINT7..0)
  PCICR |= (1 << PCIE2);
    
  // Pin change mask registers decide which pins are enabled as triggers
  PCMSK2 |= (1 << PCINT23);

  // enable interrupts
  interrupts();

  
  
}

void SetUSBPolledMode(void)
{
  // clear interrupt enable
  PCICR &= ~(1 << PCIE2);
  // clear mask
  PCMSK2 &= ~(1 << PCINT23);
}

// SOF Interrupt handler
ISR(PCINT2_vect){
  if (digitalRead(7) == HIGH) // ignore interrupt when pin change low to high
  {
    if (!interrupt_critical) // only do this when USB is not already busy or buffer data being processed
    {
       Max.Task();
       Usb.Task();
       if( tx_in_pointer > 0)
      {
        Usb.outTransfer(FTDI_ADDR, ep_record[ DATAOUT_PIPE ].epAddr, tx_in_pointer , (char *) txbuf); // output on endpoint 2
        tx_in_pointer = 0;
      }
    }
  }
}



