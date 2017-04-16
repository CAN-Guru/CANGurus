/*
 * can2usb.cpp
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

#define VERS_HIGH     0x00  // Versionsnummer vor dem Punkt
#define VERS_LOW      0x02  // Versionsnummer nach dem Punkt

void processRXFrame();
void printFrame();
void sendConfig(int index);

uint32_t UID;

// EEPROM-Adressen
#define  setup_done 0x047
const uint8_t adr_setup_done = 0x00;

const uint8_t adr_HiByte = 0x01;
const uint8_t adr_LoByte = 0x02;

// config-Daten
#define CONFIG_NUM 1     // Anzahl der Konfigurationspunkte
int config_index = 0;
bool config_request = false;
bool uid_request;

void setup()
{
  uint8_t setup_todo;
  setup_todo = eeprom_read_byte(adr_setup_done);
  if (setup_todo != setup_done){
    // wurde das Setup bereits einmal durchgeführt?
    // dann wird dieser Anteil übersprungen
    // 47, weil das EEPROM (hoffentlich) nie ursprünglich diesen Inhalt hatte

    // setzt die Boardnum anfangs auf NULL
    eeprom_update_byte (( uint8_t *) adr_HiByte, '0');
    eeprom_update_byte (( uint8_t *) adr_LoByte, '0');
    CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
    CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);

    // setup_done auf "TRUE" setzen
    eeprom_update_byte (( uint8_t *) adr_setup_done, setup_done);
  }
  else{
    CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
    CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
  }  
  //Set CAN speed. Note: Speed is now 250kbit/s so adjust your CAN monitor
  CAN.begin(CAN_BPS_250K);
  _delay_ms(500);  // Delay added just so we can have time to open up //serial Monitor and CAN bus monitor. It can be removed later...
  UID = generateUID(UID_BASE, &CAN.params);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  CAN.hash = generateHash(UID+99);
  //
  Serial.begin(baudrate);
  Serial.println("--------------------------------------");
  Serial.println("CAN Monitor-Interface");
  Serial.println("--------------------------------------");
  Serial.println("100 Ready");
}

// Test rapid fire ping/pong of extended frames
void loop()
{
  if (CAN.available())
  {
    // Process
    processRXFrame();
  }
  if (config_request) {
    config_request = false;
    sendConfig(config_index);
  }
}

void boardnumAnswer(){
  CAN.outgoingMsg.data[0] = BOARDNUM_ANSWER;
  CAN.outgoingMsg.data[1] = CAN.params.HiByteAddress;
  CAN.outgoingMsg.data[2] = CAN.params.LoByteAddress;
  what_is_your_name(I_am_a_NanoApp, 3, &CAN.outgoingMsg);
  CAN.can_answer(6);
}

/*
   Ausführen, wenn eine Nachricht verfügbar ist.
   Nachricht wird geladen und anhängig vom CAN-Befehl verarbeitet.
*/
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
      CAN.outgoingMsg.data[6] = DEVTYPE_CAN2USB >> 8;
      CAN.outgoingMsg.data[7] = DEVTYPE_CAN2USB;
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
      // alle Aufträge von usb2can abarbeiten
      case FOR_APP:
      CAN.outgoingMsg.data[0] = 'a';
      if ((CAN.incomingMsg.data[1] == CAN.params.HiByteAddress) &&
      (CAN.incomingMsg.data[2] == CAN.params.LoByteAddress)) {
        CAN.outgoingMsg.cmd = APP_ANSWER;
        switch (CAN.incomingMsg.data[0])
        {
//          case GO_BTLDR:
//          goIntoBootloader();
//          break;
          case BOARDNUM_REQUEST:
          boardnumAnswer();
          break;
          case BOARDNUM_CHANGE:
          // Reihenfolge wichtig, damit mit der alten Boardnum geantwortet wird
          boardnumAnswer();
          eeprom_update_byte(( uint8_t *) adr_HiByte, CAN.incomingMsg.data[3]);
          eeprom_update_byte(( uint8_t *) adr_LoByte, CAN.incomingMsg.data[4]);
          CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
          CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
          UID = generateUID(UID_BASE, &CAN.params);
          CAN.hash = generateHash(UID+99);
          break;
        }
      }
      break;
    }
  }
  printFrame();
}

void printFrame(){
  char charVal[5];
  sprintf(charVal, "%04X", CAN.incomingMsg.hash);
  Serial.print(charVal);
  Serial.print(" ");
  sprintf(charVal, "%02X", CAN.incomingMsg.cmd);
  Serial.print(charVal);
  Serial.print(" ");
  if (CAN.incomingMsg.resp_bit==true)
    Serial.print("R ");
  else
    Serial.print("  ");
  sprintf(charVal, "%02X", CAN.incomingMsg.length);
  Serial.print(charVal);
  Serial.print(" ");
  for (byte i=0; i<CAN.incomingMsg.length; i++)
  {
    sprintf(charVal, "%02X", CAN.incomingMsg.data[i]);
    Serial.print(charVal);
    Serial.print(" ");
  }
  Serial.println();
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
    {'M', 'o', 'n','i', 't', 'o', 'r', 0}
    }
};

for (int i = 0; i < config_len[index]; i++) {
  CAN.configDataFrame(config_frames[index][i], i);
}
CAN.configTerminator(index, config_len[index]);
}
