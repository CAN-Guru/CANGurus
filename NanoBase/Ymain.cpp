/*
 * StartTESTBTLDR.cpp
 *
 * Created: 22.02.2017 16:21:45
 * Author : Gustav
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "ownCAN.h"
#include "CAN.h"

void processRXFrame();
void boardnumAnswer();
void sendConfig(int index);

#define VERS_HIGH     0x00  // Versionsnummer vor dem Punkt
#define VERS_LOW      0x02  // Versionsnummer nach dem Punkt

uint32_t UID;
bool config_request = false;
#define CONFIG_NUM 1     // Anzahl der Konfigurationspunkte
int config_index = 0;
bool uid_request;

void setup(){

  Serial.begin(baudrate);
  sei();
  // ab hier werden die Anweisungen bei jedem Start durchlaufen
  CAN.params.HiByteAddress = '0';
  CAN.params.LoByteAddress = '0';
  UID = generateUID(UID_BASE + 0xF0, &CAN.params);
  CAN.begin(CAN_BPS_250K);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  //serial Monitor and CAN bus monitor. It can be removed later...
  CAN.hash = generateHash(UID);
  
  Serial.println("\n\rHier ist das Anwendungsprogramm...");
}
void processRXFrame()
{
  CAN.incomingMsg = getCanFrame();
  if (CAN.incomingMsg.resp_bit == false)
  {
    switch (CAN.incomingMsg.cmd)
    {
      // PING-Abfragen beantworten
      case PING:
      CAN.outgoingMsg.cmd = PING;
      // what_is_your_name(I_am_a_NanoApp, 0, &CAN.outgoingMsg);
      for (uint8_t i = 0; i < 4; i++) {
        CAN.outgoingMsg.data[i] = CAN.params.uid_device[i];
      }
      CAN.outgoingMsg.data[4] = VERS_HIGH;
      CAN.outgoingMsg.data[5] = VERS_LOW;
      CAN.outgoingMsg.data[6] = DEVTYPE_BASE >> 8;
      CAN.outgoingMsg.data[7] = DEVTYPE_BASE;
      CAN.can_answer(8);
      break;
      // config
      case CONFIG_Status:
      uid_request = true;
      for (uint8_t i=0; i<uid_num; i++)
      uid_request = uid_request && (CAN.params.uid_device[i] == CAN.incomingMsg.data[i]);
      if (uid_request==true) {
        config_request = true;
        config_index = CAN.incomingMsg.data[4];
      }
      break;
      case FOR_APP:
      if ((CAN.incomingMsg.data[1] == CAN.params.HiByteAddress) &&
      (CAN.incomingMsg.data[2] == CAN.params.LoByteAddress)) {
        CAN.outgoingMsg.cmd = APP_ANSWER;
        switch (CAN.incomingMsg.data[0])
        {
          case GO_BTLDR:
            goIntoBootloader();
            break;
          case BOARDNUM_REQUEST:
            boardnumAnswer();
            break;
        }
      }
      break;
    }
  }
} 

void boardnumAnswer(){
  CAN.outgoingMsg.data[0] = BOARDNUM_ANSWER;
  CAN.outgoingMsg.data[1] = CAN.params.HiByteAddress;
  CAN.outgoingMsg.data[2] = CAN.params.LoByteAddress;
  what_is_your_name(I_am_a_NanoApp, 3, &CAN.outgoingMsg);
  CAN.can_answer(6);
}

void sendConfig(int index) {
  uint8_t config_len[] = {4, 5, 5};
  uint8_t config_frames[][5][8] = {{
    {CONFIG_NUM-1, 1, 0, 0, 0, 0, 0, ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0')},
    {( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[0])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[0])),
      ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[1])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[1])),
      ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[2])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[2])),
    ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[3])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[3]))},
    {'C', 'A', 'N', 'g', 'u', 'r', 'u', ' '},
    {'B', 'a', 's','e', 0, 0, 0, 0}
  }
};

for (int i = 0; i < config_len[index]; i++) {
  CAN.configDataFrame(config_frames[index][i], i);
}
CAN.configTerminator(index, config_len[index]);
}

void loop()
  {
    if( Serial.available() > 0)
    {
      unsigned int 	c = Serial.read();
      switch( (unsigned char)c)
      {
        case 'b':
        Serial.println("\n\rSpringe zum Bootloader...");
        goIntoBootloader();
        break;
        default:
        Serial.print("\n\rDu hast folgendes Zeichen gesendet (b für Bootloader): ");
        Serial.println((unsigned char)c);
        break;
      }
    }
    if(CAN.available())
    {
      processRXFrame();
    }
  if (config_request) {
    config_request = false;
    sendConfig(config_index);
  }
  }
