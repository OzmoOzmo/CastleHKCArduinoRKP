/*
 * RKP.cpp - All the Remote Keypad comms and display commands
 *
 * Created: 3/30/2014 10:04:10 PM
 *
 *   Aritech Alarm Panel Arduino Internet Enabled Keypad -  CS350 - CD34 - CD72 - CD91 and more
 *
 *   For Arduino (UNO or Leonardo) with added Ethernet Shield
 *
 *   See Circuit Diagram for wiring instructions
 *
 *   Author: Ozmo
 *
 *   See: http://www.boards.ie/vbulletin/showthread.php?p=88215184
 *
 *
 * ref. Gen. Interrupts http://gammon.com.au/interrupts
 *      UART Interrupts http://www.electroons.com/blog/2013/02/interrupt-driven-uart-serial-communication-for-atmel-avr/
        Low Level UART  http://www.avrfreaks.net/forum/tut-soft-using-usart-serial-communications?name=PNphpBB2&file=viewtopic&t=45341
        Bits explained  http://maxembedded.com/2013/09/the-usart-of-the-avr/
*/

/*http://www.appelsiini.net/2011/simple-usart-with-avr-libc
UCSR0A Bit #	Name	Description
bit 7	RXC0	USART Receive Complete. Set when data is available and the data register has not be read yet.
bit 6	TXC0	USART Transmit Complete. Set when all data has transmitted.
bit 5	UDRE0	USART Data Register Empty. Set when the UDR0 register is empty and new data can be transmitted.
bit 4	FE0	Frame Error. Set when next byte in the UDR0 register has a framing error.
bit 3	DOR0	Data OverRun. Set when the UDR0 was not read before the next frame arrived.
bit 2	UPE0	USART Parity Error. Set when next frame in the UDR0 has a parity error.
bit 1	U2X0	USART Double Transmission Speed. When set decreases the bit time by half doubling the speed.
bit 0	MPCM0	Multi-processor Communication Mode. When set incoming data is ignored if no addressing information is provided.

UCSR0B Bit #	Name	Description
bit 7	RXCIE0	RX Complete Interrupt Enable. Set to allow receive complete interrupts.
bit 6	TXCIE0	TX Complete Interrupt Enable. Set to allow transmission complete interrupts.
bit 5	UDRIE0	USART Data Register Empty Interrupt Enable. Set to allow data register empty interrupts.
bit 4	RXEN0	Receiver Enable. Set to enable receiver.
bit 3	TXEN0	Transmitter enable. Set to enable transmitter.
bit 2	UCSZ20	USART Character Size 0. Used together with UCSZ01 and UCSZ00 to set data frame size. Available sizes are 5-bit (000), 6-bit (001), 7-bit (010), 8-bit (011) and 9-bit (111).
bit 1	RXB80	Receive Data Bit 8. When using 8 bit transmission the 8th bit received.
bit 0	TXB80	Transmit Data Bit 8. When using 8 bit transmission the 8th bit to be submitted.

UCSR0C Bit #	Name	Description
bit 7	UMSEL01  USART Mode Select 1 and 0. UMSEL01 and UMSEL00 combined select the operating mode. Available modes are asynchronous (00), synchronous (01) and master SPI (11).
bit 6	UMSEL00
bit 5	UPM01  USART Parity Mode 1 and 0. UPM01 and UPM00 select the parity. Available modes are none (00), even (10) and odd (11).
bit 4	UPM00
bit 3	USBS0	USART Stop Bit Select. Set to select 1 stop bit. Unset to select 2 stop bits.
bit 2   UCSZ01  USART Character Size 1 and 0. Used together with with UCSZ20 to set data frame size. Available sizes are 5-bit (000), 6-bit (001), 7-bit (010), 8-bit (011) and 9-bit (111).
bit 1	UCSZ00
bit 0	UCPOL0	USART Clock Polarity. Set to transmit on falling edge and sample on rising edge. Unset to transmit on rising edge and sample on falling edge.
*/


//change network settings to yours
#include <Arduino.h>
#include "RKP.h"
#include "LOG.h"
#include "Websocket.h"
#include "SMTP.h"
#include "Config.h"

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define UART_BAUD_SELECT(baudRate,xtalCpu)  (((xtalCpu) + 8UL * (baudRate)) / (16UL * (baudRate)) -1UL)

//Translations for Leonardo.
#define UCSRA UCSR1A
#define UCSRB UCSR1B
#define UBRRH UBRR1H
#define UBRRL UBRR1L

#define UCPOL UCPOL1
#define UCSZ0 UCSZ10
#define UCSZ1 UCSZ11
#define USBS USBS1
#define UPM0 UPM10
#define UPM1 UPM11
#define UMSEL0 UMSEL10
#define UMSEL1 UMSEL11

#define UCSRA UCSR1A //_SFR_MEM8(0xC8)
#define MPCM MPCM1 //0
#define U2X U2X1 //1
#define UPE UPE1 //2
#define DOR DOR1 //3
#define FE FE1 //4
#define UDRE UDRE1 //5
#define TXC TXC1 //6
#define RXC RXC1 //7

#define UCSRB UCSR1B //_SFR_MEM8(0xC9)
#define TXB8 TXB81 //0
#define RXB8 RXB81 //1
#define UCSZ2 UCSZ12 //2
#define TXEN TXEN1 //3
#define RXEN RXEN1 //4
#define UDRIE UDRIE1 //5
#define TXCIE TXCIE1 //6
#define RXCIE RXCIE1 //7

#define UDR UDR1 //_SFR_MEM8(0xCE)
#define UDR_0 UDR1_0 //0
#define UDR_1 UDR1_1 //1
#define UDR_2 UDR1_2 //2
#define UDR_3 UDR1_3 //3
#define UDR_4 UDR1_4 //4
#define UDR_5 UDR1_5 //5
#define UDR_6 UDR1_6 //6
#define UDR_7 UDR1_7 //7

#define UCSRC UCSR1C //_SFR_MEM8(0xCA)
#define UCPOL UCPOL1 //0
#define UCSZ0 UCSZ10 //1
#define UCSZ1 UCSZ11 //2
#define USBS USBS1 //3
#define UPM0 UPM10 //4
#define UPM1 UPM11 //5
#define UMSEL0 UMSEL10 //6
#define UMSEL1 UMSEL11 //7


#define nSerialBaudKP_RX 1658
#define ixMaxPanel 40	//40 bytes enough


const char PROGMEM allmonths[] = {"JANFEBMARAPRMAYJUNJULAUGSEPOCTNOVDEC"};
const char PROGMEM alldays[] = {"SUNMONTUEWEDTHUFRISAT"};

bool RKPClass::dateFlash = true;
bool RKPClass::mbIsPanelWarning = false;
bool RKPClass::mbIsPanelAlarm = false;
volatile bool RKPClass::bScreenHasUpdated = false;

volatile int RKPClass::_nLen=0;
byte RKPClass::_r[10];

unsigned long RKPClass::timeToSwitchOffLed = 0;

FIFO RKPClass::fifo;
byte RKPClass::lastkey = 0xFF;


static byte RKPID;
static int bIsScanning;
static bool bWaitingNewID;
static byte RKPSerial[] = {0x41, 0xC7, 0x08};



#define DISP_BUF_LEN 16+1+2		//16 characters - space - AW (will be followed by a Terminator 0)
volatile byte RKPClass::dispBuffer[DISP_BUF_LEN + 1];


byte MapKey(char c)
{
  //map desktop keyboard presess to mobile phone keys
  if (c == 'f')                              c= '*';	//UP  (* for IPhone)
  else if (c == 'v')                         c = '#';	//DOWN (# for IPhone)
  //?else if (c == 'p')                      c = 0x0d;	//UP + DOWN (Panic)
  else if (c == 'x'||c == ';' || c == 'n')   c = 'N';	//UP + 0 or X(reject) (WAIT on IPhone numpad)
  else if (c == 13||c == '+'||c == 'y')      c = 'Y';	//UP + 0 or *(accept) (+ on IPhone numpad)
  
  //Keys are sent as the following: (msb 3 bits are row - lsb 3 bits are column)
  if (c == '1') return 0x01; //		   001
  if (c == '2') return 0x02; //		   010
  if (c == '3') return 0x04; //		   100
  if (c == '4') return 0x11; //		 10001
  if (c == '5') return 0x12; //		 10010
  if (c == '6') return 0x14; //		 10100
  if (c == 'Q') return 0x18; //		 11000		(not always sent?)
  if (c == '7') return 0x21; //		100001
  if (c == '8') return 0x22; //		100010
  if (c == '9') return 0x24; //		100100
  if (c == 'Y') return 0x28; // 	101000		(not always sent?)
  if (c == '*') return 0x31; //		110001
  if (c == '0') return 0x32; //		110010
  if (c == '#') return 0x34; //		110100
  if (c == 'N') return 0x38; //		111000
  return 0xFF;
}

void AddKeyPress(byte k)
{
}

void uart_init()
{
  //disable
  DDRB |= _BV(0);      //sbi(DDRB, 0);
  PORTB &= ~(_BV(0));  //cbi(PORTB, 0);
  //configure
  UCSRA = 0;
  UCSRB = (1 << TXEN) | (1 << RXEN) | (1 << RXCIE);
  UCSRC = /*1<<URSEL |*/ 1 << UCSZ1 | 1 << UCSZ0 ; //1 stop bit 0 parity
  //9bit
  UCSRB |= (1 << UCSZ2); // Character Size = 9-bit
  //baud
  UBRRL =  (unsigned char)UART_BAUD_SELECT(nSerialBaudKP_RX, F_CPU);
  UBRRH =  (unsigned char)(UART_BAUD_SELECT(nSerialBaudKP_RX, F_CPU) >> 8);
  //enable
  sei();
}


void uart_putchar(byte c, boolean isAddr)
{
  loop_until_bit_is_set(UCSR1A, UDRE1); // Wait until data register empty.

  if (isAddr)
    UCSR1B |= (1 << TXB81);
  else
    UCSR1B &= ~(1 << TXB81);
  UDR1 = c;

}

RKPClass::RKPClass()
{
}


void RKPClass::SendItems()
{
  if (_nLen!=0)
  {
    SendToPanelEx(_r, _nLen);
    _nLen=0;
    LogLn("SendItems Sent!!!");
  }
}

void RKPClass::SendToPanel(byte* r, int nLen)
{
  for(int n=0;n<nLen;n++)
    _r[n]=r[n];
  _nLen=nLen;
}


void RKPClass::SendToPanelEx(byte* r, int len)
{
  digitalWrite(LED_Stat, HIGH);
  timeToSwitchOffLed = millis() + 1000;

//cli(); //Interrupts Must be enabled - or only 4 or so bytes send without error
  //Serial1.write(r,nLen);
  //LogLn("[");
  if (len > 0)
    uart_putchar(r[0], true);
  for (int n = 1; n < len; n++)
  {
    uart_putchar(r[n], false);
    //uart_putchar('x', false);
    //delay(5);
  }
  //LogLn("]");
//sei();
//#if debug_send
  Log("A");
  if (RKPID == 0xFF)
    Log("?");
  else Log(RKPID);
  Log(">"); LogHex(r, len);   //Log((b0&0x10)==0?0:1);
//#endif

}


bool RKPClass::HKCReplyToPanel(byte* buf, int nBufLen)
{
    //check checksum
    byte ics = 0;
    for (int n = 0; n < (nBufLen-1); n++)
      ics += buf[n];
    if (ics != buf[nBufLen - 1])
    {
      Log(F("CS Fail :( "));
      LogHex(buf, nBufLen);
      return false;
    }

  //8 = 1000  //9 = 1001  //A = 1010  //B = 1011  //C = 1100  //D = 1101

  byte b0 = buf[0];  byte b1 = buf[1];

  bool bSent = false;
  bool bIsPanelMsg = ((b0 & 0x80) == 0x80);
  if (bIsPanelMsg && RKPID != -1 )
  { //message from Panel (otherwise its another keypad message)
    if (b1 == 0x00 )
    {
      /* Command#0: ping
        K0 00 01 01 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
        P  A1 00 A1 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:!.!#####################
        K1 21 00 60 FF FF FF 7E -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:!.`...~#################
        P  A2 00 A2 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:"."#####################
        K2 22 00 62 FF FF FF 81 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:".b....#################
        P  A0 00 A0 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
        K0 20 00 62 FF FF FF 7F -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:..b....#################
      */
      byte addr = (b0 & 0x0f);
      if (addr == RKPID)
      { //ACK     P B1 00 B1
        byte r[7];
        r[0] = b0 & 0x33;  //keep bit5&6(Counter) and bit1&0(address)
        r[1] = 0;          //always 0?
        r[2] = 0x60;       //can be 60, 70 (72=Tamper) (78 just after tamper happens)
        r[3] = PopKeyPress();       //keypress here eg 0x12
        r[4] = PopKeyPress();       //keypress here eg 0x12
        r[5] = PopKeyPress();       //keypress here eg 0x12
        r[6] = (byte)(r[0] + r[1] + r[2] + r[3] + r[4] + r[5]);

        SendToPanel(r, 7);
        bSent=true;
      }
      lastkey = 0xFF;
    }
    else if (b1 == 0x01 )
    {
      /* Command#1: Display message
      P  C0 01 20 54 75 65 20 32 37 20 4A 61 6E 20 32 30 BA 30 39 76 -- -- -- --:@..Tue.27.Jan.20:09v####
      K0 00 01 01 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
      */
      // C0:1100,0000    D0:1101,0000     A1: 1010,0001   but Not 9 10010001
      byte addr = (b0 & 0x30) >> 4;
      if (RKPID == 0x00)
      { //RKP0 only ever responds
        byte r[3];
        r[0] = b0 & 0x30;  ////keep bit5&6(Counter) - C0 respond 0x00; D0 respond 0x10
        r[1] = b1;
        r[2] = r[0] + r[1];
        SendToPanel(r, 3);
        bSent=true;
      }
      //Make a note to send it to the Mobile Phone
      for(int n=0;n<DISP_BUF_LEN;n++)
        dispBuffer[n]=buf[n+3];
      dispBuffer[DISP_BUF_LEN]=0;        
      dispBuffer[DISP_BUF_LEN-1]=0;
      dispBuffer[DISP_BUF_LEN-2]=0;
      bScreenHasUpdated=true;
      
      //Log("Screen Updated:"); LogLn((char*)dispBuffer);
      
    }
    else if (b1 == 0x02)
    {
      LogLn("Found Command #2");
    }
    else if (b1 == 0x03)
    {
      /* Command#3 Possibly to light leds or sound buzzer?
      Green and Red were lit
      P  C0 03 33 3F 35 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:@.3?5###################
      K0 00 03 03 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
      P  D0 03 33 3F 45 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:P.3?E###################
      K0 10 03 13 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
      P  C0 03 33 3F 35 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:@.3?5###################
      K0 00 03 03 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
      //Enter service menu - only Green Led Lit
      P  D0 03 03 3F 15 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:P..?.###################
      K0 10 03 13 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
      P  A0 00 A0 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################
      K0 20 00 72 FF FF FF 8F -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:..r....#################
      P  C0 01 0C 53 65 72 76 69 63 65 20 4D 65 6E 75 20 20 20 20 D3 -- -- -- --:@..Service.Menu....S####
      */
      byte addr = (b0 & 0x0f);
      if (addr == RKPID)
      { //ONLY RKP0 will reply to this
        byte r[3];
        r[0] = b0 & 0x33;  //keep bit5&6(Counter)  C0 respond 0x00; D0 respond 0x10 - will always by RKP 0
        r[1] = b1;
        r[2] = r[0] + r[1];
        SendToPanel(r, 3);
        bSent=true;
      }
    }
    else if (b1 == 0x04)
    {
      LogLn("Found Command #4");
    }
  }
  //These commands dont need a valid RKPID
  if (bIsPanelMsg)
  {
    //
    if (b1 == 0x05)
    {
      /* scanning start - 3 devices all logs ON
        P  C0 01 00 D3 E3 E1 EE EE E9 EE E7 A0 CB E5 F9 F0 E1 E4 F3 E3 -- -- -- --:@..Scanning.Keypadsc####  //Message to All
        K0 00 01 01 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //RKP0 Always responds (only RKP0)
        P  A0 00 A0 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //Ack P0 -> RKP0
        K0 20 00 72 FF FF FF 8F -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:..r....#################  //K0 responds (0x72 - tamper was open)
        P  81 00 81 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //Ack P0 -> RKP1
        K1 01 00 70 FF FF FF 6E -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:..p...n#################  //K1 responds (0x70 - No tamper reported)
        P  B0 00 B0 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:0.0#####################  //Ack P0 -> RKP0 (again?? why)
        K0 30 00 72 FF FF FF 9F -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:0.r....#################  //K0 responds (0x72 - tamper)
       *P  D0 05 D5 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:P.U#####################  //Panel Skips C0 - sends D0 command 05 (no one acks)      //All devices enter scan mode
        P  8F 06 95 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //Panel Sends      8F  command 06 to ALL                  //Next?
        K1 0F 06 41 C7 08 01 26 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:..AG..&#################  //K1 Responds      0F  command 06            41 C7 08 01  //08C741 is the number written on the chip sticker - 01=version
        P  9F 07 01 41 C7 08 01 B8 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...AG..8################  //Panel sends      9F          07         01 41 C7 08 01  //Assign 01 to RKP
        K1 11 07 00 18 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:....####################  //K1 Responds      11(RKP1)    07         00              //k1 says ok
        P  8F 06 95 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //Panel sends      8F  command 06                         //Next?
        K0 0F 06 87 BC 08 01 61 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...<..a#################  //K0               0F  command 06            87 BC 08 01
        P  9F 07 00 87 BC 08 01 F2 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:....<..r################  //Panel            9F  command 07         00 87 BC 08 01  //Assign 00 to RKP
        K0 10 07 00 17 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:....####################  //K0               10(RKP0)    07         00 17           //k0 says ok
        P  8F 06 95 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //PanelFirstTo2    8F          06                         //Next?
        K2 0F 06 91 CF 08 01 7E -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...O..~#################  //K2               0F          06            91 CF 08 01  //08CF91 is the number written on the chip sticker
        P  9F 07 02 91 CF 08 01 11 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:....O...################  //Panel Assign ID  9F          07         02 91 CF 08 01  //Assign 02 to RKP
        K2 12 07 00 19 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:....####################  //K2               12(RKP2)    07         00              //k1 says ok
        P  92 09 9B -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //P                92          09                         //Panel requests version number
        K2 12 09 00 00 01 04 20 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:.......#################  //K2               12(RKP2)    09         00 00 01 04     //keypad says its v1.4
        P  8F 06 95 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //P                8F          06                         //Next? (no response)
        P  8F 06 95 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //P                8F          06                         //Next? (no response)
        P  A2 00 A2 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:"."#####################  //PollK2           A2          00
        K2 22 00 6A FF FF FF 89 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:".j....#################  //K2               22          00     6A FF FF FF    //no keys
        P  D0 01 0F 44 65 76 69 63 65 73 20 66 6F 75 6E 64 20 33 20 52 -- -- -- --:P..Devices.found.3.R####
      */
      /*RKPClass::*/bIsScanning = 2;
      LogLn("Scanning Mode");
    }
    else if (b1 == 0x06 && bIsScanning > 0)
    {
      /*Request Serial - should wait here some time before sending
        P  8F 06 95 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...#####################  //Panel Sends      8F  command 06 to ALL                  //Next?
        K1 0F 06 41 C7 08 01 26 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:..AG..&#################  //K1 Responds      0F  command 06            41 C7 08 01  //08C741 is the number written on the chip sticker - 01=version
      */

      bIsScanning--;
      if (bIsScanning == 0)
      { //our turn...

        byte addr = (b0 & 0x0f);
        //if (addr == 0x0f)
        { //ACK
          byte r[8];
          r[0] = 0x1F;  //Always the Global address          15    //Sometimes this is 0x1F ????????????why??
          r[1] = b1;    //repeat command                      6
          r[2] = RKPSerial[0];  ///Our Unique ID (lBf)       41
          r[3] = RKPSerial[1];  //                           C7
          r[4] = RKPSerial[2];  //                           08
          r[5] = 0x01;              //version #1
          r[6] = (byte)(r[0] + r[1] + r[2] + r[3] + r[4] + r[5]);


          delay(16); //From Scope - 40ms from end of request to start of 07 command

          SendToPanel(r, 7);
          bSent=true;

          LogLn("Waiting for our new ID"); bIsScanning = -1; bWaitingNewID = true;
        }
      }
      else
      {
        bWaitingNewID = false;
        LogLn("Not our turn");
      }

    }
    else if (b1 == 0x07 && bWaitingNewID == true)
    {
      /* Command#7: Assign ID
      P  9F 07 01 41 C7 08 01 B8 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:...AG..8################  //Panel sends      9F          07         01 41 C7 08 01  //Assign 01 to RKP
      K1 11 07 00 18 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --:....####################  //K1 Responds      11(RKP1)    07         00              //k1 says ok
      */

      //LogLn("Set new ID");
      //Log(buf[3]);Log("=");LogLn(RKPSerial[0]);
      //Log(buf[4]);Log("=");LogLn(RKPSerial[1]);
      //Log(buf[5]);Log("=");LogLn(RKPSerial[2]);
      //LogLn("");

      if (buf[2] == 0)
      { //We dont want to become RKP#0 - that should be a real keypad
        LogLn("Nah!");
      }
      else
      {
        if (buf[3] == RKPSerial[0] && buf[4] == RKPSerial[1] && buf[5] == RKPSerial[2])
        { //Serials match - change our ID
          RKPID = buf[2];
          byte r[5];
          r[0] = (b0 & 0x30) | RKPID ;  //8x 9x Ax bX -> 0x 1x 2x 3x - where x is the RKPID
          r[1] = b1;
          r[2] = 0x00;    //? Always 0
          r[3] = r[0] + r[1] + r[2];

          delay(18); //From Scope - 40ms from end of request to start of 07 command

          SendToPanel(r, 4);
          bSent=true;
          Log("New ID#"); LogLn(RKPID); bWaitingNewID = false;
        }
        else
        {
          LogLn("Not Our Serial");
        }
      }
    }

  }//#if is from panel


  //if (bSent == true)
  //{
  //  digitalWrite(LED_Stat, HIGH);
  //  timeToSwitchOffLed = millis() + 50;
  //  //LogLn("Sent!"); LogHex(buf,nBufLen);
  //}

  return true;
}

#if defined(UART1_RX_vect)
ISR(UART1_RX_vect)
#elif defined(USART1_RX_vect)
ISR(USART1_RX_vect)
#else
#error "Don't know what the Data Register Empty vector is called for Serial1"
#endif
{
  static byte buf[64 + 5];
  static int bufix = 0;

  boolean isAddr = (UCSRB & (1 << RXB8)) != 0; //Must read First
  char a = UDR;
  static byte msglen = 0;

  if (isAddr)
  { //dump frame and start again
    if (bufix > 0)
    {
      Log("Bad Frame:");
      LogHex(buf, bufix);
    }

    bufix = 0;
    buf[bufix++] = a;
    //Log("New Frame");LogHex(buf,bufix);
    return;
  }
  buf[bufix] = a;
  bool bIsPanelMsg = ((*buf & 0x80) != 0);

  if (bufix == 2)
  {
    //how long is the message supposed to be?
    byte b0 = buf[0];
    byte b1 = buf[1];

    if (bIsPanelMsg)
    { //message from Panel (otherwise its another keypad message)
      if (b1 == 0x00 )      msglen = 3; //P  A1 00 A1
      else if (b1 == 0x01)  msglen = 20; //length is in byte[2] but message is always padded with spaces //P  D0 01 0F 44 65 76 69 63 65 73 20 66 6F 75 6E 64 20 33 20 52 -- -- -- --:P..Devices.found.3.R####
      else if (b1 == 0x02)  msglen = 4; //P  D0 02 00 D2
      else if (b1 == 0x03)  msglen = 5; //P  C0 03 33 3F 35 Possibly to light leds or sound buzzer?
      else if (b1 == 0x04)  msglen = 5; //P  C0 04 08 00 CC  happened during unset   D0 04 08 00 DC   D0 04 08 00 DC  on entering unset
      else if (b1 == 0x05)  msglen = 3; //P  D0 05 D5      All devices enter scan mode
      else if (b1 == 0x06)  msglen = 3; //P  8F 06 95      Scan Next
      else if (b1 == 0x07)  msglen = 8; //P  9F 07 01 41 C7 08 01 B8 //assign id
      else if (b1 == 0x0C)  msglen = 4; //P  00 0C 0C    D0 0C 0C E8  //When you press 0 - screen clears?
      else if (b1 == 0x0D)  msglen = 4; //P  D0 0D FF DC  //on entering eng mode
      else if (b1 == 0x0F)  msglen = 5; //P C0 0F 00 3F 0E   Leaving eng mode
      else
      {
        LogHex(buf, bufix);
        LogLn("Unknown Command");
      }
    }
    else
    {
      if (b1 == 0x00 )      msglen = 7; //K0 20 00 72 FF FF FF 8F
      else if (b1 == 0x01)  msglen = 3; //K0 00 01 01
      else if (b1 == 0x02)  msglen = 3; //   10 02 12
      else if (b1 == 0x03)  msglen = 3; //K0 10 03 13  Command#3 Possibly to light leds or sound buzzer?
      else if (b1 == 0x04)  msglen = 3; //   10 04 14 on entering Unset //rkp during unset
      else if (b1 == 0x05)  msglen = 0; //No response ever given
      else if (b1 == 0x06)  msglen = 7; //K1 0F 06 41 C7 08 01 26  (scan response)
      else if (b1 == 0x07)  msglen = 4; //K1 11 07 00 18
      else if (b1 == 0x0C)  msglen = 3; //K1 C0 0C 0C D8  //When you press 0 - screen clears
      else if (b1 == 0x0D)  msglen = 3; //K1 10 0D 1D  //on entering eng mode
      else if (b1 == 0x0F)  msglen = 3; //K1 00 0F 0F Leaving eng mode
      else
      {
        LogLn("Unknown Command");
        LogHex(buf, bufix);
      }
    }
  }
  bufix += 1;

  //Log(bufix);Log("=");LogLn(msglen);
  if (bufix == msglen)
  { //complete message
    RKPClass::HKCReplyToPanel(buf, bufix);
    Log(bIsPanelMsg ? "P " : "K "); LogHex(buf, bufix);
    bufix = 0;
    msglen = 0;
  }
}



#if _OFF
     //each byte is about 5mS
      static unsigned int V_PNL = 0, V_RKP0 = 0, V_RKP1 = 0, V_RKP2 = 0;
      int nV = analogRead(A0) / 8;
        //Serial.print(nV,HEX);Log(" ");
        if (nV > 0x30) //data HIGH
        {
          if (nV > 0x60)      //actually 0x69
            V_PNL++;
          else if (nV > 0x4C) //actually 0x4E 0x4D
            V_RKP0++;
          else if (nV > 0x47) //actually 0x48 0x49
            V_RKP1++;
          else
            V_RKP2++;
        }
      }
      Log(V_PNL > V_RKP0 ? "P  " :  V_RKP0 > V_RKP1 ? "K0 " :  V_RKP1 > V_RKP2 ? "K1 " : "K2 "); LogHex(buf, bufix);
      V_PNL = V_RKP0 = V_RKP1 = V_RKP2 = 0;
      }
      //Knock off the "We Sent To Panel" Led not less than 100ms after we turn it on
      if (timeToSwitchOffLed != 0 && millis() > timeToSwitchOffLed)
      {
        timeToSwitchOffLed = 0;
        digitalWrite(LED_Stat, LOW);
      }
 

        if (idDev == mdevID)
        {
          bool bAck = (msgbuf[0] & 0x04) != 0;	//required for good comms
          SendToPanel(mdevID, bAck);
          DisplayScreen(msgbuf, msgbufLen); //try display what device 0 sees
          //LogLn(F("-OK-"));
        }

        { //Send Email if alarm lights come on
          bool bIsPanelWarning = (msgbuf[1] & 0x04) != 0;
          if (bIsPanelWarning == true && RKPClass::mbIsPanelWarning == false)
            SMTP::QueueEmail();
          RKPClass::mbIsPanelWarning = bIsPanelWarning;

          bool bIsPanelAlarm = (msgbuf[1] & 0x02) != 0;
          if (bIsPanelAlarm == true && RKPClass::mbIsPanelAlarm == false)
            SMTP::QueueEmail();
          RKPClass::mbIsPanelAlarm = bIsPanelAlarm;
        }
#endif


char RKPClass::PopKeyPress()
{
  return (char)fifo.pop();
}

void RKPClass::PushKey( char key )
{
  fifo.push(key);
}


void RKPClass::SendDisplayToClientIfChanged()
{
  bScreenHasUpdated = false;    //possible issue if interrupt has changed this since we checked it - might miss one screen update
  //LogHex((byte*)dispBuffer,16);
  //cli();
  WebSocket::WebSocket_send((char*)dispBuffer, 16);
  //sei();
  //Log("Sent:"); LogLn((char*)dispBuffer);
}


void RKPClass::Init()
{
  uart_init();  //adjust for 9bits

  //HKC Init
  dispBuffer[16] = '|';
  dispBuffer[DISP_BUF_LEN] = 0;
  pinMode(LED_Stat, OUTPUT);
  bIsScanning = -1;
  bWaitingNewID = false;
  RKPID = -1;
}


//a 6 character keyboard buffer
byte FIFO::raw[maxkeybufsize];
FIFO::FIFO()
{
  nextIn = nextOut = count = 0;
}
void FIFO::push( byte element )
{
  if ( count >= maxkeybufsize )
  {
    Log("Too Full. Count=");LogLn(count);
    return; //lost
  }
  count++;
  raw[nextIn++] = element;
  nextIn %= maxkeybufsize;
  Log("Added Item. Count=");LogLn(count);
}

byte FIFO::pop()
{
  if (count>0)
  {
    count--;
    byte c=raw[ nextOut++];
    nextOut %= maxkeybufsize;
    
    c=MapKey(c);
    Log("Popped Item. Count=");Log(count);Log(" c=");LogLn(c);
    return c;
  }
  return 0xFF;
}

