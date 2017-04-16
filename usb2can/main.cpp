/*
 * usb2can.cpp
 *
 * Created: 21.11.2016 10:27:24
 * Author : Gustav
 */

/* CAN extended Frames with Ping/Pong sending

  This example sets up a receive and transmit mailbox on both CAN devices.
  First NODE0 sends to NODE1. When NODE1 receives it sends to NODE0. PING/PONGs forever
  and as quickly as possible - This will saturate the bus so don't have anything important connected.

  Authors: Thibaut Viard, Wilfredo Molina, Collin Kidder, Pedro Cevallos & Neil McNeight
  Created: 06/01/14
  Updated: 06/14/15

  As per wiki information:
  "CAN bus is a message-based protocol, designed specifically for automotive
  applications but now also used in other areas such as aerospace, maritime,
  industrial automation and medical equipment."

  For more info http://en.wikipedia.org/wiki/Controller_area_network

 */

#include <avr/io.h>
#include <avr/pgmspace.h>

#include <util/delay.h>
#include <inttypes.h>

#include "ownCAN.h"
#include "CAN.h"
#include "SPI.h" // required to resolve #define conflicts

void processRXFrame();
void appAnswer();
void btldrRequest();
void appRequest();
void sendConfig(int index);
uint8_t char2num (uint8_t ch);
uint8_t get1byte();

#define VERS_HIGH     0x00  // Versionsnummer vor dem Punkt
#define VERS_LOW      0x03  // Versionsnummer nach dem Punkt

#define waitingforSerial      0
#define waitingforBoardNum    1
#define waitingforNewBoardNum 2
#define waitingforBootLoader  3

char val; // Data received from the serial port
String strDataIn;
String strDataOut = "";
uint8_t subCmd;
uint8_t bnHi;
uint8_t bnLo;
bool bn_exists;
bool btldr_exists;
unsigned long previousMillis;
unsigned long interval=500;

int processStep;

uint32_t UID;

// config-Daten
#define CONFIG_NUM 0     // Anzahl der Konfigurationspunkte
int config_index = 0;
bool config_request = false;
bool uid_request;

void setup()
{
  //Set CAN speed. Note: Speed is now 250kbit/s so adjust your CAN monitor
  CAN.begin(CAN_BPS_250K);
  _delay_ms(500);  // Delay added just so we can have time to open up //serial Monitor and CAN bus monitor. It can be removed later...
  CAN.params.HiByteAddress = '0';
  CAN.params.LoByteAddress = 0x099;
  UID = generateUID(UID_BASE, &CAN.params);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  CAN.hash = generateHash(UID+CAN.params.LoByteAddress);
  //initialize serial communications at a 19200 baud rate
  Serial.begin(baudrate);
  strDataIn = "";
  processStep = waitingforSerial;
}

// Test rapid fire ping/pong of extended frames
void loop()
{
  if (config_request) {
    config_request = false;
    sendConfig(config_index);
  }
  switch (processStep)
  {
    case waitingforSerial:
    if (Serial.available() > 0) { // If data is available to read,
      val = Serial.read(); // read it and store it in val
      if (val!=limiter)
       strDataIn += val;
      else
      switch (strDataIn.charAt(0))
      {
        case '!':
        // CMD: '!' Kontakt mit hex2usb herstellen; für Portnummer
          Serial.print("$#");   // send a capital $
          Serial.flush();
          strDataIn="";
          break;
        case '?':
        // CMD: '?XY#' fragt alle Boards ab; Board mit BoardNum XY (?XY) antwortet
         bn_exists = false;
          subCmd = BOARDNUM_REQUEST;
          bnHi = strDataIn.charAt(1);
          bnLo = strDataIn.charAt(2);
          appRequest();
          processStep = waitingforBoardNum;
          Serial.flush();
          strDataIn="";
          previousMillis = millis();
          break;
        case '=':
        // CMD: '=XYAB#' ändert die Boardnummer des Boards XY in AB ab;
          bn_exists = false;
          subCmd = BOARDNUM_CHANGE;
          bnHi = strDataIn.charAt(1);
          bnLo = strDataIn.charAt(2);
          appRequest();
          processStep = waitingforNewBoardNum;
          Serial.flush();
          strDataIn="";
          previousMillis = millis();
          break;
        case '%':
        // CMD: '&XY#' schickt Board mit BoardNum XY in den Bootloaderstate
          subCmd = GO_BTLDR;
          bnHi = strDataIn.charAt(1);
          bnLo = strDataIn.charAt(2);
          appRequest();
          processStep = waitingforBootLoader;
          Serial.flush();
          strDataIn="";
          break;
        case 'd':
          CAN.outgoingMsg.cmd = BTLDR_ANSWER;
          byte cnt = char2num(get1byte()); // length
          if (cnt==0)
            CAN.outgoingMsg.data[0] = END_DATA;
          else
          {
            CAN.outgoingMsg.data[0] = MORE_DATA;
            for (byte i=1; i<=cnt; i++){
              CAN.outgoingMsg.data[i] = get1byte();
            }
          }
          CAN.can_answer2(cnt+1,false);
          processStep = waitingforSerial;
          Serial.flush();
          strDataIn="";
          break;
      } // switch
    } // if
    break; // waitingforSerial
    case waitingforBoardNum:
    case waitingforNewBoardNum:
      if (bn_exists==true){
        Serial.print("1#"); //send the message back
        processStep = waitingforSerial;
      }
      else{
        if ((millis()-previousMillis)>interval){
          Serial.print("0#"); //send the message back
          processStep = waitingforSerial;
        }
      }
      break; // waitingforBoardNum
    case waitingforBootLoader:
      if (btldr_exists==true){
        processStep = waitingforSerial;
      }
      else{
        if ((millis()-previousMillis)>interval){
          processStep = waitingforSerial;
        }
      }
      break; // waitingforBoardNum
  } // switch
  delay(50);
  if (CAN.available())
  {
    // Process
    processRXFrame();
  }
}

void processRXFrame()
{
  CAN.incomingMsg = getCanFrame();
  if (CAN.incomingMsg.resp_bit == false)
  {
    switch (CAN.incomingMsg.cmd)
    {
      case PING:
      CAN.outgoingMsg.cmd = PING;
      for (uint8_t i = 0; i < 4; i++) {
        CAN.outgoingMsg.data[i] = CAN.params.uid_device[i];
      }
      CAN.outgoingMsg.data[4] = VERS_HIGH;
      CAN.outgoingMsg.data[5] = VERS_LOW;
      CAN.outgoingMsg.data[6] = DEVTYPE_USB2CAN >> 8;
      CAN.outgoingMsg.data[7] = DEVTYPE_USB2CAN;
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
    }    
  }  
  appAnswer();
}

/*
   hier rüber laufen alle Antworten des Dekoders
*/
void appAnswer() {
  if ((CAN.incomingMsg.cmd == APP_ANSWER) &&
      (CAN.incomingMsg.resp_bit == true) &&
      (CAN.incomingMsg.data[1] == bnHi) &&
      (CAN.incomingMsg.data[2] == bnLo)) {
        switch (CAN.incomingMsg.data[0])
        {
          case BOARDNUM_ANSWER:
          bn_exists=true;
          break;
          case GO_BTLDR:
          btldr_exists=true;
          break;
        }
  }
  if ((CAN.incomingMsg.cmd == BTLDR_ANSWER) &&
      (CAN.incomingMsg.resp_bit == true)) {
        switch (CAN.incomingMsg.data[0])
        {
          case START_DATA:
           Serial.print("$#"); //send the message back
            break;
          case MORE_DATA:
            Serial.print("/#"); //send the message back
            break;
        }
    }
}

/*
   hier rüber laufen alle Anfragen an den Dekoder
*/
void appRequest() {
  CAN.outgoingMsg.cmd = FOR_APP;
  CAN.outgoingMsg.data[0] = subCmd;
  CAN.outgoingMsg.data[1] = bnHi;
  CAN.outgoingMsg.data[2] = bnLo;
  switch (subCmd)
  {
    case BOARDNUM_REQUEST:
      what_is_your_name(I_am_a_usb2can, 3, &CAN.outgoingMsg);
      CAN.can_answer2(6, false);
      break;
    case BOARDNUM_CHANGE:
    // neue Boardnum
      CAN.outgoingMsg.data[3] = strDataIn.charAt(3);
      CAN.outgoingMsg.data[4] = strDataIn.charAt(4);
      CAN.can_answer2(5, false);
      break;
    case GO_BTLDR:
      CAN.can_answer2(3, false);
      break;
  }
}

/*
   hier rüber laufen alle Anfragen an den Bootloader
*/
void btldrRequest() {
  CAN.outgoingMsg.cmd = FOR_BTLDR;
  CAN.outgoingMsg.data[0] = subCmd;
  CAN.outgoingMsg.data[1] = bnHi;
  CAN.outgoingMsg.data[2] = bnLo;
  CAN.can_answer2(3, false);
}

uint8_t char2num (uint8_t ch)
{
    // Hex-Ziffer auf ihren Wert abbilden
    if (ch >= '0' && ch <= '9') ch -= '0';
    else if (ch >= 'A' && ch <= 'F') ch -= 'A' - 10;
    else if (ch >= 'a' && ch <= 'f') ch -= 'a' - 10;
  return ch;
}

uint8_t get1byte(){
  do
    {
  } while (Serial.available() == 0 );
  return Serial.read(); // read it and store it in val
}

void sendConfig(int index) {
  uint8_t config_len[] = {4, 5, 5};
  uint8_t config_frames[][5][8] = {{
    {0, CONFIG_NUM, 0, 0, 0, 0, 0, ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0')},
    {( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[0])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[0])),
      ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[1])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[1])),
      ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[2])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[2])),
    ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[3])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[3]))},
    {'C', 'A', 'N', 'g', 'u', 'r', 'u', ' '},
    {'G', 'a', 't','e', 'w', 'a', 'y', 0}
  }
};

for (int i = 0; i < config_len[index]; i++) {
  CAN.configDataFrame(config_frames[index][i], i);
}
CAN.configTerminator(index, config_len[index]);
}
