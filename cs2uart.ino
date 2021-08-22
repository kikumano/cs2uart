
//convert cyberstick data to serial
//for attiny2313 @8mhz

//depend on https://github.com/kikumano/PicoSerial

//recieve data from cyberstick
//send 8byte data to serial
//data format: digispark hid joystick
//X Y XROT YROT ZROT SLIDER btn_lobyte btn_hibyte

//10kohm pullup
//https://www.clarestudio.org/elec/misc/joystick.html


//#include <avr/pgmspace.h>

#define PICOSERIAL_USE_READBUFF
//#define PICOSERIAL_USE_WRITEBUFF
#define PICOSERIAL_BUFF_SIZE      4
#include "PicoSerial.h"

//d0 recv
//d1 send
//serial baudrate: 115200
//#define PICOSERIAL_BAUDRATE     250000
//#  define PICOSERIAL_BAUDRATE   115200
//#define PICOSERIAL_BAUDRATE     76800
//#  define PICOSERIAL_BAUDRATE   57600
//#define PICOSERIAL_BAUDRATE     38400
//#define PICOSERIAL_BAUDRATE     9600
#define PICOSERIAL_BAUDRATE   38400

//  d: disable polling
//  e: enable polling
//  s: send stickdata onetime
//  R: reset setting: not save setting.
//  S: save setting to eeprom
//  i,I: set polling interval ms: i - 20, I - 2000
//  b: send in binary
//  t: send in text
//  r: raw12 data format
//  h: hid8 data format
//  x: compensate f,g button in digital mode. for XE-1AP.
//  c: don't compensate f,g button in digital mode. for CZ-8NJ2.
//  0,1,2,3: deassert req at ack1,ack2,ack4,ack8
//    in binary:
//      setting ack: 01xx
//      hid8: 09xxxxxxxxxxxxxxxx[aa|dd]
//      raw12: 0dxxxxxxxxxxxxxxxxxxxxxxxxxx[aa|dd]
//        0xaa - analog mode, 0xdd - digital mode or disconnected
//    in text:
//      setting ack: bbbbb0bb\r\n
//      hid8: hhhhhhhhhhhhhhhh[-|*]\r\n
//      raw12: hhhhhhhhhhhhhhhhhhhhhhhh[-|*]\r\n
//        b: bit.
//        hh: hex. byte value.
//        * - analog mode, - - digital mode or disconnected

//
byte polling_interval = 2;

//  bit7: 0: disable, 1: enable
//  bit6: 0: oneshot, 1: repeat
//  bit5: 0: text,    1: binary
//  bit4: 0: hid8,    1: raw12
//  bit3: 0: asis,    1: compensate f,g button in digital mode. XE-1AP only.
//  bit2: 0: digital, 1: analog mode
//  bit0-1: 0: full, 1: half, 2: 3rd, 3: low speed
#define SEND_DATAMODE_ENABLE    7
#define SEND_DATAMODE_REPEAT    6
#define SEND_DATAMODE_BIN       5
#define SEND_DATAMODE_RAW       4
#define SEND_DATAMODE_XE_1AP    3
#define SEND_DATAMODE_ANALOG    2
#define SEND_DATAMODE_DATASPEED_1   1
#define SEND_DATAMODE_DATASPEED_0   0
#define SEND_DATAMODE_DATASPEED_MASK   ( bit( SEND_DATAMODE_DATASPEED_1 ) | bit( SEND_DATAMODE_DATASPEED_0 ) )
byte send_datamode = 0;
#define EEPROM_ADDR_SEND_DATAMODE   0

//data transfer speed
#define CS_DATASPEED_FULL   1
#define CS_DATASPEED_HALF   2
#define CS_DATASPEED_3RD    4
#define CS_DATASPEED_LOW    8
byte cs_dataspeed = CS_DATASPEED_FULL;

//data port
#define CS_DATAPORT       PORTB
#define CS_DATAPORT_IN    PINB
#define CS_BIT0_PIN       9
#define READ_DATAPORT()   ( ~CS_DATAPORT_IN )
#define READ_DATAPORT_BIT( n )    bitRead( PINB, n - CS_BIT0_PIN )

//heartbeat led
#define CS_LED_PIN    9
#define LED_OFF()     bitSet( CS_DATAPORT, CS_LED_PIN - CS_BIT0_PIN )
#define LED_ON()      bitClear( CS_DATAPORT, CS_LED_PIN - CS_BIT0_PIN )
#define LED_TURN()    ( CS_DATAPORT ^= bit( CS_LED_PIN - CS_BIT0_PIN ) )

#define LED_TIMER_OFF()     do{ bitClear( TIMSK, OCIE1A ); bitClear( TIMSK, OCIE1B ); } while( 0 )
#define LED_TIMER_ON()      do{ bitSet( TIMSK, OCIE1A ); bitSet( TIMSK, OCIE1B ); } while( 0 )

#define LED_VOLUME          OCR1B
#define LED_VOLUME_MIN      0
#define LED_VOLUME_MAX      0xff
#define LED_VOLUME_TOP      0x100
byte led_count = 0;

//analog mode
#define CS_REQ_PIN    10    // dsub 8 pin
//#define ASSERT_REQ()    digitalWrite( CS_REQ_PIN, LOW )
//#define DEASSERT_REQ()  digitalWrite( CS_REQ_PIN, HIGH )
#define ASSERT_REQ()    bitClear( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define DEASSERT_REQ()  bitSet( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define CS_ACK_PIN    11    // dsub 7 pin
#define READ_ACK_PIN()  bitRead( PINB, CS_ACK_PIN - CS_BIT0_PIN )
#define CS_CLK_PIN    12    // dsub 6 pin
#define READ_CLK_PIN()  bitRead( PINB, CS_CLK_PIN - CS_BIT0_PIN )
#define CS_D0_PIN     13    // dsub 1 pin
#define CS_D1_PIN     14    // dsub 2 pin
#define CS_D2_PIN     15    // dsub 3 pin
#define CS_D3_PIN     16    // dsub 4 pin
#define READ_DATA_NIBBLE()    ( ( READ_DATAPORT() >> 4 ) & 0x0f )

//nibble0:  A B C D
#define CS_NIBBLE_ABCD    0
#define CS_NIBBLE0_A        0b1000
#define CS_NIBBLE0_B        0b0100
#define CS_NIBBLE0_C        0b0010
#define CS_NIBBLE0_D        0b0001
//nibble1: E1E2 F G
#define CS_NIBBLE_E1E2FG  1
#define CS_NIBBLE1_E1       0b1000
#define CS_NIBBLE1_E2       0b0100
#define CS_NIBBLE1_F        0b0010
#define CS_NIBBLE1_G        0b0001
//nibble2: <HINIBBLE0>
#define CS_NIBBLE_Y0_HI   2
//nibble3: <HINIBBLE1>
#define CS_NIBBLE_X0_HI   3
//nibble4: <HINIBBLE2>
#define CS_NIBBLE_Y1_HI   4
//nibble5: <HINIBBLE3>
#define CS_NIBBLE_X1_HI   5
//nibble6: <LONIBBLE0>
#define CS_NIBBLE_Y0_LOW  6
//nibble7: <LONIBBLE1>
#define CS_NIBBLE_X0_LOW  7
//nibble8: <LONIBBLE2>
#define CS_NIBBLE_Y1_LOW  8
//nibble9: <LONIBBLE3>
#define CS_NIBBLE_X1_LOW  9
//nibble10: A B A'B'
#define CS_NIBBLE_AABB    10
#define CS_NIBBLE10_A       0b1000
#define CS_NIBBLE10_B       0b0100
#define CS_NIBBLE10_AA      0b0010
#define CS_NIBBLE10_BB      0b0001
//nibble11: <RESERVED>
#define CS_NIBBLE_RESERVED 11

//digital mode
//common high: r l d u a b
//common low:  d ctdtue1e2
#define CS_COMMON_PIN        CS_REQ_PIN
#define SET_COMMON_PIN_HIGH()   bitSet( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define SET_COMMON_PIN_LOW()    bitClear( CS_DATAPORT, CS_REQ_PIN - CS_BIT0_PIN )
#define CS_B_E2_PIN          CS_ACK_PIN
#define CS_A_E1_PIN          CS_CLK_PIN
#define CS_UP_TUP_PIN        CS_D0_PIN
#define CS_DOWN_TDOWN_PIN    CS_D1_PIN
#define CS_LEFT_C_PIN        CS_D2_PIN
#define CS_RIGHT_D_PIN       CS_D3_PIN
//in XE-1AP: 
//  CS_RIGHT_D_PIN & CS_LEFT_C_PIN? start: 0;
//  CS_DOWN_TDOWN_PIN & CS_UP_TUP_PIN? select: 0;

byte stickdata_raw[12];


//X Y XROT YROT ZROT SLIDER btn_lobyte btn_hibyte
#define USB_HID_X             0
#define USB_HID_Y             1
#define USB_HID_XROT          2
#define USB_HID_YROT          3
#define USB_HID_ZROT          4
#define USB_HID_SLIDER        5
#define USB_HID_BTN_LOBYTE    6
#define USB_HID_BTN_HIBYTE    7
byte stickdata[8];

#define READ_EEPROM( addr, data )       do{ while( bitRead( EECR, EEPE ) ){  } \
                                          EEAR = addr; \
                                          bitSet( EECR, EEPE ); \
                                          data = EEDR; } while( 0 )
#define WRITE_EEPROM( addr, data )      do{ while( bitRead( EECR, EEPE ) ){  } \
                                          EECR = 0; \
                                          EEAR = addr; EEDR = data; \
                                          bitSet( EECR, EEMPE ); bitSet( EECR, EEPE ); } while( 0 )


void setup() {
  // put your setup code here, to run once:
//disable Analog Comparator: save energy
  bitSet( ACSR, ACD );

//load setting:
//  while( bitRead( EECR, EEPE ) ){  }
//  EEAR = EEPROM_ADDR_SEND_DATAMODE;
//  bitSet( EECR, EERE );
//  send_datamode = EEDR;
  READ_EEPROM( EEPROM_ADDR_SEND_DATAMODE, send_datamode );
  if( send_datamode == 0xff ){ send_datamode = 0; }
//  cs_dataspeed = bit( send_datamode & SEND_DATAMODE_DATASPEED_MASK );
  cs_dataspeed = 1 << ( send_datamode & SEND_DATAMODE_DATASPEED_MASK );

//in:
//  pinMode( CS_ACK_PIN, INPUT_PULLUP );
//  pinMode( CS_CLK_PIN, INPUT_PULLUP );
//  pinMode( CS_D0_PIN, INPUT_PULLUP );
//  pinMode( CS_D1_PIN, INPUT_PULLUP );
//  pinMode( CS_D2_PIN, INPUT_PULLUP );
//  pinMode( CS_D3_PIN, INPUT_PULLUP );
  CS_DATAPORT = 0xff;

//out:
//  DEASSERT_REQ();
//  pinMode( CS_REQ_PIN, OUTPUT );
//  pinMode( CS_LED_PIN, OUTPUT );
  bitSet( DDRB, CS_REQ_PIN - CS_BIT0_PIN );
  bitSet( DDRB, CS_LED_PIN - CS_BIT0_PIN );

//serial:
  PicoSerial.begin( PICOSERIAL_BAUDRATE );

//init timer1 for heartbeat led:
  OCR1A = LED_VOLUME_MAX;
  LED_VOLUME = LED_VOLUME_MIN;
  bitSet( TCCR1B, WGM12 );
//  bitSet( TCCR1B, CS11 );   // 1Mhz
  bitSet( TCCR1B, CS10 );   // 125Khz‬
  bitSet( TCCR1B, CS11 );   // 125Khz‬
//  bitSet( TCCR1B, CS12 );   // 31.25Khz‬
  LED_TIMER_ON();
}

#define CALC_TIMEOUT( dataspeed )     ( dataspeed )
//#define CALC_TIMEOUT( dataspeed )     ( dataspeed << 1 )
//#define CALC_TIMEOUT( dataspeed )     ( ( 1 + dataspeed ) << 1 )

//bool is_timeout( unsigned long t ){
//  unsigned long d = millis();
//
//  if( t & 0x80000000 ){
//    t -= 0x80000000;
//    d -= 0x80000000;
//  }
//
//  return ( t + CALC_TIMEOUT( cs_dataspeed ) ) < d;
//}
//bool is_timeout( const unsigned long t ){
//  if( t & 0x80000000UL ){
//    return ( t - 0x80000000UL + CALC_TIMEOUT( cs_dataspeed ) ) < ( millis() - 0x80000000UL );
//  }
//  return ( t + CALC_TIMEOUT( cs_dataspeed ) ) < millis();
//}
bool is_timeout( const unsigned long t ){
  const unsigned long d = t & 0x80000000UL;
  return ( t - d + CALC_TIMEOUT( cs_dataspeed ) ) < ( millis() - d );
}
//#define is_timeout( t )   ( t & 0x80000000UL? ( t - 0x80000000UL + CALC_TIMEOUT( cs_dataspeed ) ) < ( millis() - 0x80000000UL ): ( t + CALC_TIMEOUT( cs_dataspeed ) ) < millis() )

//CZ-8NJ2 00000707070F0C0F0B0F0000*
//a:      08000707070F0B0F0B0F0200*
//b:      04000707070F0B0F0B0F0100*
//c:      02000707070F0B0F0B0F0000*
//d:      01000707070F0B0F0D0F0000*
//e1:     00080707070F0B0F0C0F0000*
//e2:     00040707070F0B0F0D0F0000*
//start:  00020707070F0B0F0B0F0000*
//select: 00010707070F0B0F0B0F0000*
//a':     08000808070F01060B0F0800*
//b':     04000708070F0E090B0F0400*
//        hid8:               raw12:
//XE-1AP  7175FF6B00000000*   00000706060F0407070F0000*
//a:      7175FF6B00000880*   08000706060F0407070F0800*
//b:      7175FF6B00000440*   04000706060F0407070F0400*
//c:      7175FF6B00000020*   02000706060F0407070F0000*
//d:      7175FF6B00000010*   01000706060F0407070F0000*
//e1:     7175FF6B00000008*   00080706060F0407070F0000*
//e2:     7175FF6B00000004*   00040706060F0407070F0000*
//start:  7175FF6B00000002*   00020706060F0407070F0000*
//select: 7175FF6B00000001*   00010706060F0407070F0000*
//a':     7175FF6B00000280*   08000706060F0407070F0200*
//b':     7175FF6B00000140*   04000706060F0407070F0100*
//up:     63FFFF6B00000000*   00000F06060F0F08070F0000*
//down:   7A00FF6B00000000*   00000007060F0005070F0000*
//left:   FF72FF6B00000000*   0000060F060F0D0F070F0000*
//right:  006DFF6B00000000*   00000700060F0600070F0000*
//tup:    696EFFFF00000000*   000007070F0F06020F0F0000*
//tdown:  696EFF0000000000*   00000707000F0602000F0000*

//return true: probably in digital mode or unconnected.
bool fetch_stickdata_analog( void ){
  const unsigned long t = millis();

//assert req
  ASSERT_REQ();
  
  for( byte i = 0; i < 12; ++i ){
//wait ack falling edge & clk: lhlhlhlhlhlh
    while( READ_ACK_PIN() || ( READ_CLK_PIN() == !( i & 0x01 ) ) ){
      if( is_timeout( t ) ){ DEASSERT_REQ(); return true; }
    }

    if( i == cs_dataspeed ){
//deassert req
      DEASSERT_REQ();
    }

//read nibble
    stickdata_raw[i] = READ_DATA_NIBBLE();
  }

  return false;
}

//CZ-8NJ2 00000708070F0E090B0F0000-
//a:      00080708070F0E090B0F0000-
//b:      00040708070F0E090B0F0000-
//c:      00000708070F0E090B0F4000-
//d:      00000708070F0E090B0F8000-
//e1:     08000708070F0E090B0F0000-
//e2:     04000708070F0E090B0F0000-
//start:  00000708070F0E090B0F0000-
//select: 00000708070F0E090B0F0000-
//a':     00080708070F0E090B0F0000-
//b':     00040708070F0E090B0F0000-
//up:     00000708070F0E090B0F0400-
//down:   00000708070F0E090B0F0800-
//left:   02000708070F0E090B0F0000-
//right:  01000708070F0E090B0F0000-
//tup:    00000708070F0E090B0F1000-
//tdown:  00000708070F0E090B0F2000-
//        hid8:               raw12:
//XE-1AP  FFFFFFFF00000000-   00000F0F0F0F0F0F0F0F0000-
//a:      FFFFFFFF00000080-   08000F0F0F0F0F0F0F0F0000-
//b:      FFFFFFFF00000040-   04000F0F0F0F0F0F0F0F0000-
//c:      FFFFFFFF00000020-   02000F0F0F0F0F0F0F0F0000-
//d:      FFFFFFFF00000010-   01000F0F0F0F0F0F0F0F0000-
//e1:     FFFFFFFF00000008-   00080F0F0F0F0F0F0F0F0000-
//e2:     FFFFFFFF00000004-   00040F0F0F0F0F0F0F0F0000-
//start:  FFFFFFFF0000C032-   03020F0F0F0F0F0F0F0FC000-
//select: FFFFFFFF00003C01-   00010F0F0F0F0F0F0F0F3C00-
//a':     FFFFFFFF00000080-   08000F0F0F0F0F0F0F0F0000-
//b':     FFFFFFFF00000040-   04000F0F0F0F0F0F0F0F0000-
//up:     FFFFFFFF00001000-   00000F0F0F0F0F0F0F0F1000-
//down:   FFFFFFFF00002000-   00000F0F0F0F0F0F0F0F2000-
//left:   FFFFFFFF00004000-   00000F0F0F0F0F0F0F0F4000-
//right:  FFFFFFFF00008000-   00000F0F0F0F0F0F0F0F8000-
//tup:    FFFFFFFF00000400-   00000F0F0F0F0F0F0F0F0400-
//tdown:  FFFFFFFF00000800-   00000F0F0F0F0F0F0F0F0800-

//common high: r l d u a b
//common low:  d ctdtue1e2
void fetch_stickdata_digital( void ){
  byte b;

  for( byte i = 0; i < sizeof stickdata_raw; ++i ){
    stickdata_raw[i] = 0x0f;
  }
  stickdata_raw[CS_NIBBLE_RESERVED] = 0x00;

  SET_COMMON_PIN_LOW();
  delayMicroseconds( 1 );
  b = READ_DATAPORT();
  stickdata_raw[CS_NIBBLE_ABCD] = ( ( b >> 7 ) | ( ( b & 0x40 ) >> 5 ) ) & 0x03;
  stickdata_raw[CS_NIBBLE_E1E2FG] = b & 0x0c;
  stickdata_raw[CS_NIBBLE_AABB] = ( b >> 2 ) & 0x0c;

  SET_COMMON_PIN_HIGH();
  delayMicroseconds( 1 );
  b = READ_DATAPORT();
  stickdata_raw[CS_NIBBLE_ABCD] |= b & 0x0c;
  stickdata_raw[CS_NIBBLE_AABB] |= b & 0xf0;

  if( bitRead( send_datamode, SEND_DATAMODE_XE_1AP ) ){
    b &= b << 1;
//    stickdata_raw[CS_NIBBLE_E1E2FG] |= ( ( ( b & 0x80 ) >> 6 ) | ( ( b & 0x20 ) >> 5 ) ) & 0x03;
    stickdata_raw[CS_NIBBLE_E1E2FG] |= ( b >> 6 ) & 0x02;
    stickdata_raw[CS_NIBBLE_E1E2FG] |= ( b >> 5 ) & 0x01;
  }
}
//X Y XROT YROT ZROT SLIDER btn_lobyte btn_hibyte
void conv_stickdata( void ){
  stickdata[USB_HID_Y] = stickdata_raw[CS_NIBBLE_Y0_HI] << 4;
  stickdata[USB_HID_Y] |= stickdata_raw[CS_NIBBLE_Y0_LOW];
  stickdata[USB_HID_X] = stickdata_raw[CS_NIBBLE_X0_HI] << 4;
  stickdata[USB_HID_X] |= stickdata_raw[CS_NIBBLE_X0_LOW];
  
  stickdata[USB_HID_YROT] = stickdata_raw[CS_NIBBLE_Y1_HI] << 4;
  stickdata[USB_HID_YROT] |= stickdata_raw[CS_NIBBLE_Y1_LOW];
  stickdata[USB_HID_XROT] = stickdata_raw[CS_NIBBLE_X1_HI] << 4;
  stickdata[USB_HID_XROT] |= stickdata_raw[CS_NIBBLE_X1_LOW];
  
  stickdata[USB_HID_ZROT] = 0x00;
  stickdata[USB_HID_SLIDER] = 0x00;
  
  stickdata[USB_HID_BTN_HIBYTE] = stickdata_raw[CS_NIBBLE_ABCD] << 4;
  stickdata[USB_HID_BTN_HIBYTE] |= stickdata_raw[CS_NIBBLE_E1E2FG];
  stickdata[USB_HID_BTN_LOBYTE] = stickdata_raw[CS_NIBBLE_AABB];
}
void send_stickdata( void ){
//  PROGMEM const char *hex_table = "0123456789ABCDEF";
  const bool f = bitRead( send_datamode, SEND_DATAMODE_RAW );
  const byte *data = f? stickdata_raw: stickdata;
  const byte data_size = f? sizeof stickdata_raw: sizeof stickdata;

  if( bitRead( send_datamode, SEND_DATAMODE_BIN ) ){
    PicoSerial.write( data_size + 1 );
    PicoSerial.write( data, data_size );
    PicoSerial.write( bitRead( send_datamode, SEND_DATAMODE_ANALOG )? 0xaa: 0xdd );
  } else {
    for( byte i = 0; i < data_size; ++i ){
//    PicoSerial.print( hex_table[*( data + i ) >> 4] );
//    PicoSerial.print( hex_table[*( data + i ) & 0x0f] );
      if( *( data + i ) < 0x10 ){ PicoSerial.print( '0' ); }
      PicoSerial.print( *( data + i ), HEX );
    }
    PicoSerial.println( bitRead( send_datamode, SEND_DATAMODE_ANALOG )? '*': '-' );
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  bool f_send_ack = true;
  const byte b = PicoSerial.read();
  switch( b ){
  case 'd':
    bitClear( send_datamode, SEND_DATAMODE_ENABLE );
    break;
  case 'e':
    bitSet( send_datamode, SEND_DATAMODE_REPEAT );
    bitSet( send_datamode, SEND_DATAMODE_ENABLE );
    break;
  case 's':
    bitClear( send_datamode, SEND_DATAMODE_REPEAT );
    bitSet( send_datamode, SEND_DATAMODE_ENABLE );
    break;
  case 'R': send_datamode = 0; break;
  case 'S':
  //avoid store 0xff
    bitClear( send_datamode, SEND_DATAMODE_ANALOG );
//    while( bitRead( EECR, EEPE ) ){  }
//    EECR = 0;
//    EEAR = EEPROM_ADDR_SEND_DATAMODE;
//    EEDR = send_datamode;
//    bitSet( EECR, EEMPE );
//    bitSet( EECR, EEPE );
    WRITE_EEPROM( EEPROM_ADDR_SEND_DATAMODE, send_datamode );
    break;
//  case 'i': polling_interval = 2; break;
//  case 'I': polling_interval = 200; break;
  case 'b': bitSet( send_datamode, SEND_DATAMODE_BIN ); break;
  case 't': bitClear( send_datamode, SEND_DATAMODE_BIN ); break;
  case 'r': bitSet( send_datamode, SEND_DATAMODE_RAW ); break;
  case 'h': bitClear( send_datamode, SEND_DATAMODE_RAW ); break;
  case 'x': bitSet( send_datamode, SEND_DATAMODE_XE_1AP ); break;
  case 'c': bitClear( send_datamode, SEND_DATAMODE_XE_1AP ); break;
//  case '0': cs_dataspeed = CS_DATASPEED_FULL; bitClear( send_datamode, SEND_DATAMODE_DATASPEED_LOW ); break;
//  case '2': cs_dataspeed = CS_DATASPEED_HALF; break;
//  case '4': cs_dataspeed = CS_DATASPEED_3RD; break;
//  case '8': cs_dataspeed = CS_DATASPEED_LOW; bitSet( send_datamode, SEND_DATAMODE_DATASPEED_LOW ); break;
  case '0': // fall thru
  case '1': // fall thru
  case '2': // fall thru
  case '3': // fall thru
    cs_dataspeed = 0x01 << ( b - '0' );
    send_datamode &= 0xfc;
    send_datamode |= b - '0';
    break;
  default:
    f_send_ack = false;
    break;
  }

  if( f_send_ack ){
    if( bitRead( send_datamode, SEND_DATAMODE_BIN ) ){
      PicoSerial.write( sizeof send_datamode );
      PicoSerial.write( send_datamode );
    } else {
      for( byte mask = 0x80; mask; mask >>= 1 ){
        PicoSerial.print( send_datamode & mask? '1': '0' );
      }
      PicoSerial.println();
    }
  }

  if( bitRead( send_datamode, SEND_DATAMODE_ENABLE ) ){
  //disable heartbeat
//    LED_TIMER_OFF();

    bool f;
    for( byte i = 0; ( f = fetch_stickdata_analog() ) && ( i < 2 ); ++i ){
  //wait 1ms
      delay( cs_dataspeed );
    }

  //enable heartbeat
//    LED_TIMER_ON();

//    if( fetch_stickdata_analog() ){
    if( f ){
      fetch_stickdata_digital();
      bitClear( send_datamode, SEND_DATAMODE_ANALOG );
    } else {
      bitSet( send_datamode, SEND_DATAMODE_ANALOG );
    }

    conv_stickdata();
    send_stickdata();

    if( bitRead( send_datamode, SEND_DATAMODE_REPEAT ) ){
    //wait 20ms
//      delay( 20 );
      for( byte c = 10; c; --c ){
        delay( polling_interval );
      }
    } else {
      bitClear( send_datamode, SEND_DATAMODE_ENABLE );
    }

  //heartbeat
//    LED_TURN();
      if( ( ++led_count & 0x1f ) && !bitRead( send_datamode, SEND_DATAMODE_ANALOG ) ){
        LED_VOLUME = LED_VOLUME_MIN;
      } else {
//        LED_VOLUME = LED_VOLUME? LED_VOLUME >> 1: LED_VOLUME_TOP;
        LED_VOLUME = LED_VOLUME? LED_VOLUME - 8: LED_VOLUME_TOP;
      }
  } else {
    LED_VOLUME = LED_VOLUME_MIN;
  }
}

ISR(TIMER1_COMPA_vect) {
  LED_ON();
}
ISR(TIMER1_COMPB_vect) {
  LED_OFF();
}
