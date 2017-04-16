/*
 * NanoBase.cpp
 *
 * Created: 21.11.2016 10:27:24
 * Author : Gustav
 */

/* CAN extended Frames

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
#include <avr/eeprom.h>

#include "ownCAN.h"
#include "CAN.h"

// EEPROM-Adressen
// EEPROM-Belegung
// adr_setup_done 00
// adr_HiByte     01
// adr_LoByte     02

#define  setup_done 0x047
const uint8_t adr_setup_done = 0x00;

const uint8_t adr_HiByte = 0x01;
const uint8_t adr_LoByte = 0x02;

// config-Daten
#define CONFIG_NUM 2     // Anzahl der Konfigurationspunkte
int config_index = 0;
bool config_request = false;
bool uid_request;

#define VERS_HIGH     0x00  // Versionsnummer vor dem Punkt
#define VERS_LOW      0x03  // Versionsnummer nach dem Punkt

uint32_t UID;

void processRXFrame();
void boardnumAnswer();
void sendConfig(int index);

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

    // setup_done auf "TRUE" setzen
    eeprom_update_byte (( uint8_t *) adr_setup_done, setup_done);
  }
  // ab hier werden die Anweisungen bei jedem Start durchlaufen
  CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
  CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
  CAN.params.moduladr = ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0');
  UID = generateUID(UID_BASE + 0xF0, &CAN.params);
  CAN.begin(CAN_BPS_250K);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  //serial Monitor and CAN bus monitor. It can be removed later...
  CAN.hash = generateHash(UID);
 // pinMode(PIN_INT0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT0), processRXFrame, LOW);
}

// main loop
void loop()
{
  if (config_request) {
    config_request = false;
    sendConfig(config_index);
  }
}

/*
   Ausführen, wenn eine Nachricht verfügbar ist.
   Nachricht wird geladen und anhängig vom CAN-Befehl verarbeitet.
*/
//Interrupt Service Routine for INT0
void processRXFrame()
{
  CAN.incomingMsg = getCanFrame();
  if (CAN.incomingMsg.resp_bit == false)
  {
    switch (CAN.incomingMsg.cmd)
    {
      case SYS_CMD:
      uid_request = true;
      for (uint8_t i=0; i<uid_num; i++)
      uid_request = uid_request && (CAN.params.uid_device[i] == CAN.incomingMsg.data[i]);
      if (uid_request==true &&
      CAN.incomingMsg.data[4]==SYS_STAT) {
	      switch (CAN.incomingMsg.data[5])
	      {
		      // Kanalnummer #1
		      case 1:
            CAN.params.moduladr = CAN.incomingMsg.data[6]*16+CAN.incomingMsg.data[7];
            // speichert die neue Adresse
            CAN.params.LoByteAddress = (uint8_t)CAN.incomingMsg.data[7]+'0';
            CAN.params.HiByteAddress = (uint8_t)CAN.incomingMsg.data[6]+'0';
            eeprom_update_byte (( uint8_t *) adr_HiByte, CAN.params.HiByteAddress);
            eeprom_update_byte (( uint8_t *) adr_LoByte, CAN.params.LoByteAddress);
		      break;
          // Kanalnummer #2
          case 2:
          switch (CAN.incomingMsg.data[7])
          {
            case 0:
            // do nothing
            break;
            case 1:
            // Restart, Werte bleiben erhalten
            // Antworten
            CAN.outgoingMsg = CAN.incomingMsg;
            CAN.outgoingMsg.data[6] = 0x01;
            CAN.can_answer(7);
            _delay_ms(3*wait_time);  // Delay added just so we can have time to open up
            // jumping to restart
            goto*0x0000;
            break;
            case 2:
            // Restart, Werte werden zurückgesetzt
            // Antworten
            CAN.outgoingMsg = CAN.incomingMsg;
            CAN.outgoingMsg.data[6] = 0x01;
            CAN.can_answer(7);
            _delay_ms(3*wait_time);  // Delay added just so we can have time to open up
            // setup_done auf "FALSE" setzen
            eeprom_update_byte (( uint8_t *) adr_setup_done, 0xFF);
            // jumping to restart
            goto*0x0000;
            break;
          }
          break;
	      }
	      // Antworten
	      CAN.outgoingMsg = CAN.incomingMsg;
	      CAN.outgoingMsg.data[6] = 0x01;
	      CAN.can_answer(7);
      }
      break;
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
      // alle Aufträge von usb2can abarbeiten
      case FOR_APP:
        if ((CAN.incomingMsg.data[1] == CAN.params.HiByteAddress) &&
            (CAN.incomingMsg.data[2] == CAN.params.LoByteAddress)) {
          CAN.outgoingMsg.cmd = APP_ANSWER;
          switch (CAN.incomingMsg.data[0])
          {
            case GO_BTLDR:
              // setup_done auf "FALSE" setzen
              eeprom_update_byte (( uint8_t *) adr_setup_done, 0xFF);
              goIntoBootloader();
              break;
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
              CAN.hash = generateHash(UID);
              CAN.params.moduladr = ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0');
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
  what_is_your_name(I_am_a_NanoBase, 3, &CAN.outgoingMsg);
  CAN.can_answer(6);
}

void sendConfig(int index) {
  uint8_t config_len[] = {4, 4, 4};
  uint8_t config_frames[][4][8] = {{
    {0, CONFIG_NUM, 0, 0, 0, 0, 0, CAN.params.moduladr},
    {( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[0])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[0])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[1])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[1])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[2])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[2])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[3])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[3]))},
    {'C', 'A', 'N', 'g', 'u', 'r', 'u', ' '},
    {'B', 'a', 's','e', 0, 0, 0, 0}},
// #1
	{{1, 2, 0, 0, 0, maxadr, 0, CAN.params.moduladr},
	{'M', 'o', 'd', 'u', 'l', 'a', 'd', 'r'},
	{'e', 's', 's', 'e', 0, '0', 0, (uint8_t)(maxadr/10)+'0'},
	{maxadr-(uint8_t)(maxadr/10)*10+'0' ,0, 'A', 'd', 'r', 0, 0, 0 }},
// #2
	{{2, 1, 3, 0, 0, 0, 0, 0},
	{'N', 'e', 'u', 's', 't', 'a', 'r', 't'},
	{0, 'N', 'e', 'i', 'n', 0, 'W', 'a'},
	{'r', 'm', 0, 'K', 'a', 'l', 't', 0}}
};

for (int i = 0; i < config_len[index]; i++) {
  CAN.configDataFrame(config_frames[index][i], i);
}
CAN.configTerminator(index, config_len[index]);
}
