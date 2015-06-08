/*
*   Castle KeyPad
*
*   HKC Alarm Panel Arduino Internet Enabled Keypad Emulator
*
*   For Arduino (UNO or Leonardo) with added Ethernet Shield
*
*   See Circuit Diagram for wiring instructions
*
*   V1.0  June  2015: Based of one of my other project: Aritech On the Internet
*
*   Author: Ozmo
*
*   See: http://www.boards.ie/vbulletin/showthread.php?p=88215184
*
*/

//See Config.h for all the configuration you may need

//--------Configuration End----------
#ifdef AtmelStudio
#include <Arduino.h>
#include "RKP.h"
#include "LOG.h"
#include "WebSocket.h"
#include "SMTP.h"
#else
#include "Base64.h"
#include "Config.h"
#include "Log.h"
#include "RKP.h"
#include "sha1.h"
#include "SMTP.h"
#include "WebSocket.h"

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#endif


//Flashing Led we can use to show activity
#define ledFeedback 12
int tiLast = 0;

const char* sEmail = EMAIL;

void setup()
{
  //--debug logging start
  Log_Init();

  //--Keypad start interrupt service
  RKPClass::Init();

  //--Create Web Server
  LogLn(F("\r\n-----[Start]-----"));
  WebSocket::WebSocket_EtherInit(
    IPAddress( IP_A, IP_B, IP_C, IP_D ),	//Give the device a unique IP
    IPAddress( IP_A, IP_B, IP_C, 1 )        //Gateway (your Router)
  );

  SMTP::Init(IPAddress( SMTP_IP_A, SMTP_IP_B, SMTP_IP_C, SMTP_IP_D ), sEmail);

  Log_ShowMem();
}

void loop()
{
  while (true)
  {
    //Flash status led
    int tiNow = millis();
    if (tiLast < tiNow - 500)
    {
      tiLast = tiNow;
      digitalWrite(ledFeedback, !digitalRead(ledFeedback));
    }
    
    //Knock off the "We Sent To Panel" Led not less than 100ms after we turn it on
    if (RKPClass::timeToSwitchOffLed != 0 && tiNow > RKPClass::timeToSwitchOffLed)
    {
      RKPClass::timeToSwitchOffLed = 0;
      digitalWrite(LED_Stat, LOW);
    }
    
    RKPClass::SendItems();

    if (RKPClass::bScreenHasUpdated)
      RKPClass::SendDisplayToClientIfChanged();

    //Any browser activity?	Set up new connections and deliver web pages
    WebSocket::EtherPoll();

    //Send Email if Alarm
    if (SMTP::nEmailStage >= 0)
      SMTP::SendEmailProcess();
  }
}


#ifdef AtmelStudio
int main(void)
{
  init();

#if defined(USBCON)
  USBDevice.attach();
#endif

  setup();

  for (;;) {
    loop();
  }
  return 0;
}
#endif
