++/* MAX3421E USB Host controller get device descriptor */
#include <Spi.h>
#include <Max3421e.h>
#include <Usb.h>
#include <avr/pgmspace.h>




#define LOBYTE(x) ((char*)(&(x)))[0]
#define HIBYTE(x) ((char*)(&(x)))[1]
#define BUFSIZE 256    //buffer size


/* Print strings in Program Memory */
const char Gen_Error_str[] PROGMEM = "\r\nRequest error. Error code:\t"; 
const char Dev_Header_str[] PROGMEM ="Device descriptor: ";
const char Dev_Length_str[] PROGMEM ="\r\nDescriptor Length:\t";
const char Dev_Type_str[] PROGMEM ="\r\nDescriptor type:\t";
const char Dev_Version_str[] PROGMEM ="\r\nUSB version:\t";
const char Dev_Class_str[] PROGMEM ="\r\nDevice class:\t";
const char Dev_Subclass_str[] PROGMEM ="\r\nDevice Subclass:\t";
const char Dev_Protocol_str[] PROGMEM ="\r\nDevice Protocol:\t";
const char Dev_Pktsize_str[] PROGMEM ="\r\nMax.packet size:\t";
const char Dev_Vendor_str[] PROGMEM ="\r\nVendor  ID:\t";
const char Dev_Product_str[] PROGMEM ="\r\nProduct ID:\t";
const char Dev_Revision_str[] PROGMEM ="\r\nRevision ID:\t";
const char Dev_Mfg_str[] PROGMEM ="\r\nMfg.string index:\t";
const char Dev_Prod_str[] PROGMEM ="\r\nProd.string index:\t";
const char Dev_Serial_str[] PROGMEM ="\r\nSerial number index:\t";
const char Dev_Nconf_str[] PROGMEM ="\r\nNumber of conf.:\t";
const char Conf_Trunc_str[] PROGMEM ="Total length truncated to 256 bytes";
const char Conf_Header_str[] PROGMEM ="\r\nConfiguration descriptor:";
const char Conf_Totlen_str[] PROGMEM ="\r\nTotal length:\t";
const char Conf_Nint_str[] PROGMEM ="\r\nNum.intf:\t\t";
const char Conf_Value_str[] PROGMEM ="\r\nConf.value:\t";
const char Conf_String_str[] PROGMEM ="\r\nConf.string:\t";
const char Conf_Attr_str[] PROGMEM ="\r\nAttr.:\t\t";
const char Conf_Pwr_str[] PROGMEM ="\r\nMax.pwr:\t\t";
const char Int_Header_str[] PROGMEM ="\r\nInterface descriptor:";
const char Int_Number_str[] PROGMEM ="\r\nIntf.number:\t";
const char Int_Alt_str[] PROGMEM ="\r\nAlt.:\t\t";
const char Int_Endpoints_str[] PROGMEM ="\r\nEndpoints:\t\t";
const char Int_Class_str[] PROGMEM ="\r\nIntf. Class:\t\t";
const char Int_Subclass_str[] PROGMEM ="\r\nIntf. Subclass:\t";
const char Int_Protocol_str[] PROGMEM ="\r\nIntf. Protocol:\t";
const char Int_String_str[] PROGMEM ="\r\nIntf.string:\t";
const char End_Header_str[] PROGMEM ="\r\nEndpoint descriptor:";
const char End_Address_str[] PROGMEM ="\r\nEndpoint address:\t";
const char End_Attr_str[] PROGMEM ="\r\nAttr.:\t\t";
const char End_Pktsize_str[] PROGMEM ="\r\nMax.pkt size:\t";
const char End_Interval_str[] PROGMEM ="\r\nPolling interval:\t";
const char Unk_Header_str[] PROGMEM = "\r\nUnknown descriptor:";
const char Unk_Length_str[] PROGMEM ="\r\nLength:\t\t";
const char Unk_Type_str[] PROGMEM ="\r\nType:\t\t";
const char Unk_Contents_str[] PROGMEM ="\r\nContents:\t";


void setup();
void loop();
 
MAX3421E Max;
USB Usb;
 
void setup()
{
  byte tmpdata = 0;
  Serial.begin( 9600 );
  Serial.println("Start");
  Max.powerOn();
  delay( 200 );
}
 
void loop()
{
  byte rcode;
//  Max.Task();
//  Usb.Task();
  /*
  if( Usb.getUsbTaskState() >= 0x80 ) {  //state configuring or higher
    rcode = getdevdescr( 1 );                    //hardcoded device address
    if( rcode ) {
      printProgStr(Gen_Error_str);
      print_hex( rcode, 8 );
    }
    rcode = getconfdescr( 1, 0 );                 //get configuration descriptor
    if( rcode ) {
      printProgStr(Gen_Error_str);
      print_hex(rcode, 8);
    }
 
    while( 1 );                          //stop
  } */   
}
 
byte getdevdescr( byte addr )
{
  USB_DEVICE_DESCRIPTOR buf;
  byte rcode;
  rcode = Usb.getDevDescr( addr, 0, 0x12, ( char *)&buf );
  if( rcode ) {
    return( rcode );
  }
  printProgStr(Dev_Header_str);
  printProgStr(Dev_Length_str);
  print_hex( buf.bLength, 8 );
  printProgStr(Dev_Type_str);
  print_hex( buf.bDescriptorType, 8 );
  printProgStr(Dev_Version_str);
  print_hex( buf.bcdUSB, 16 );
  printProgStr(Dev_Class_str); 
  print_hex( buf.bDeviceClass, 8 );
  printProgStr(Dev_Subclass_str);
  print_hex( buf.bDeviceSubClass, 8 );
  printProgStr(Dev_Protocol_str);
  print_hex( buf.bDeviceProtocol, 8 );
  printProgStr(Dev_Pktsize_str);
  print_hex( buf.bMaxPacketSize0, 8 );
  printProgStr(Dev_Vendor_str);
  print_hex( buf.idVendor, 16 );
  printProgStr(Dev_Product_str);
  print_hex( buf.idProduct, 16 );
  printProgStr(Dev_Revision_str);
  print_hex( buf.bcdDevice, 16 );
  printProgStr(Dev_Mfg_str);
  print_hex( buf.iManufacturer, 8 );
  printProgStr(Dev_Prod_str);
  print_hex( buf.iProduct, 8 );
  printProgStr(Dev_Serial_str);
  print_hex( buf.iSerialNumber, 8 );
  printProgStr(Dev_Nconf_str);
  print_hex( buf.bNumConfigurations, 8 );
  return( 0 );
}


byte getconfdescr( byte addr, byte conf )
{
  char buf[ BUFSIZE ];
  char* buf_ptr = buf;
  byte rcode;
  byte descr_length;
  byte descr_type;
  unsigned int total_length;
  rcode = Usb.getConfDescr( addr, 0, 4, conf, buf );  //get total length
  LOBYTE( total_length ) = buf[ 2 ];
  HIBYTE( total_length ) = buf[ 3 ];
  if( total_length > 256 ) {    //check if total length is larger than buffer
    printProgStr(Conf_Trunc_str);
    total_length = 256;
  }
  rcode = Usb.getConfDescr( addr, 0, total_length, conf, buf ); //get the whole descriptor
  while( buf_ptr < buf + total_length ) {  //parsing descriptors
    descr_length = *( buf_ptr );
    descr_type = *( buf_ptr + 1 );
    switch( descr_type ) {
      case( USB_DESCRIPTOR_CONFIGURATION ):
        printconfdescr( buf_ptr );
        break;
      case( USB_DESCRIPTOR_INTERFACE ):
        printintfdescr( buf_ptr );
        break;
      case( USB_DESCRIPTOR_ENDPOINT ):
        printepdescr( buf_ptr );
        break;
      default:
        printunkdescr( buf_ptr );
        break;
        }//switch( descr_type  
    buf_ptr = ( buf_ptr + descr_length );    //advance buffer pointer
  }//while( buf_ptr <=...
  return( 0 );
}
/* prints hex numbers with leading zeroes */
// copyright, Peter H Anderson, Baltimore, MD, Nov, '07
// source: http://www.phanderson.com/arduino/arduino_display.html
void print_hex(int v, int num_places)
{
  int mask=0, n, num_nibbles, digit;
 
  for (n=1; n<=num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask; // truncate v to specified number of places
 
  num_nibbles = num_places / 4;
  if ((num_places % 4) != 0) {
    ++num_nibbles;
  }
  do {
    digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
    Serial.print(digit, HEX);
  } 
  while(--num_nibbles);
}
/* function to print configuration descriptor */
void printconfdescr( char* descr_ptr )
{
 USB_CONFIGURATION_DESCRIPTOR* conf_ptr = ( USB_CONFIGURATION_DESCRIPTOR* )descr_ptr;
  printProgStr(Conf_Header_str);
  printProgStr(Conf_Totlen_str);
  print_hex( conf_ptr->wTotalLength, 16 );
  printProgStr(Conf_Nint_str);
  print_hex( conf_ptr->bNumInterfaces, 8 );
  printProgStr(Conf_Value_str);
  print_hex( conf_ptr->bConfigurationValue, 8 );
  printProgStr(Conf_String_str);
  print_hex( conf_ptr->iConfiguration, 8 );
  printProgStr(Conf_Attr_str);
  print_hex( conf_ptr->bmAttributes, 8 );
  printProgStr(Conf_Pwr_str);
  print_hex( conf_ptr->bMaxPower, 8 );
  return;
}
/* function to print interface descriptor */
void printintfdescr( char* descr_ptr )
{
 USB_INTERFACE_DESCRIPTOR* intf_ptr = ( USB_INTERFACE_DESCRIPTOR* )descr_ptr;
  printProgStr(Int_Header_str);
  printProgStr(Int_Number_str);
  print_hex( intf_ptr->bInterfaceNumber, 8 );
  printProgStr(Int_Alt_str);
  print_hex( intf_ptr->bAlternateSetting, 8 );
  printProgStr(Int_Endpoints_str);
  print_hex( intf_ptr->bNumEndpoints, 8 );
  printProgStr(Int_Class_str);
  print_hex( intf_ptr->bInterfaceClass, 8 );
  printProgStr(Int_Subclass_str);
  print_hex( intf_ptr->bInterfaceSubClass, 8 );
  printProgStr(Int_Protocol_str);
  print_hex( intf_ptr->bInterfaceProtocol, 8 );
  printProgStr(Int_String_str);
  print_hex( intf_ptr->iInterface, 8 );
  return;
}
/* function to print endpoint descriptor */
void printepdescr( char* descr_ptr )
{
 USB_ENDPOINT_DESCRIPTOR* ep_ptr = ( USB_ENDPOINT_DESCRIPTOR* )descr_ptr;
  printProgStr(End_Header_str);
  printProgStr(End_Address_str);
  print_hex( ep_ptr->bEndpointAddress, 8 );
  printProgStr(End_Attr_str);
  print_hex( ep_ptr->bmAttributes, 8 );
  printProgStr(End_Pktsize_str);
  print_hex( ep_ptr->wMaxPacketSize, 16 );
  printProgStr(End_Interval_str);
  print_hex( ep_ptr->bInterval, 8 );
  
  return;
}
/*function to print unknown descriptor */
void printunkdescr( char* descr_ptr )
{
  byte length = *descr_ptr;
  byte i;
  printProgStr(Unk_Header_str);
  printProgStr(Unk_Length_str);
  print_hex( *descr_ptr, 8 );
  printProgStr(Unk_Type_str);
  print_hex( *(descr_ptr + 1 ), 8 );
  printProgStr(Unk_Contents_str);
  descr_ptr += 2;
  for( i = 0; i < length; i++ ) {
    print_hex( *descr_ptr, 8 );
    descr_ptr++;
  }
}
 

/* Print a string from Program Memory directly to save RAM */
void printProgStr(const prog_char str[])
{
  char c;
  if(!str) return;
  while((c = pgm_read_byte(str++)))
    Serial.print(c,BYTE);
}


