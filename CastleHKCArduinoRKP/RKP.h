/*
 * RKP.h
 *
 * Created: 3/30/2014 11:18:18 PM

 */ 

#ifndef RKP_h
#define RKP_h

#include "Arduino.h"

#define maxkeybufsize 6
#define LED_Stat 13  //displays packets using LED 13

class FIFO {
public:
	FIFO();
	byte pop();
	void push( byte b );
private:
	int nextIn;
	int nextOut;
        int count;
	static byte raw[maxkeybufsize];
};


class RKPClass
{
                static FIFO fifo;
                static volatile int _nLen;
             	static byte _r[10];
      
	public:
		//void static loop_PanelMon();
		void static SendDisplayToClientIfChanged();
                void static SendItems();
		//static void DisplayScreen( byte* msgbuf, int msgbuflen);
		static char NextKeyPress();
		static char PopKeyPress();
		static void PushKey( char param1 );
		RKPClass();
		static void Init();
		//static void SendDisplayToBrowser();
                static void SendToPanel(byte* r, int nLen);  //send a message to the panel
                void static SendToPanelEx(byte* r, int len);
		static void AddKeyPress(byte k);  //Keypress from Phone- queue them up and send 3 at a time to alarm
                static volatile bool bScreenHasUpdated; //When true there is a display buffer ready to g
                static volatile byte dispBuffer[];
                
		static bool dateFlash;
		
		static bool mbIsPanelWarning;
		static bool mbIsPanelAlarm;
		static unsigned long timeToSwitchOffLed;
      //private:
                static bool HKCReplyToPanel(byte* buf, int nBufLen);
                static byte lastkey;
};
//extern RKPClass rkp;

#endif
