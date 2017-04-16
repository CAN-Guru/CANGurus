/*
 * hall2can.cpp
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

#include "Wire.h"

// EEPROM-Belegung
// adr_setup_done 00
// adr_HiByte     01
// adr_LoByte     02
// adr_offset     03
// adr_status     04

// SRAM   2KB (ATmega328)
// EEPROM 1KB (ATmega328)

#define setup_done 0x047

const uint8_t adr_setup_done  = 0x00;
const uint8_t adr_HiByte      = 0x01;
const uint8_t adr_LoByte      = 0x02;
const uint8_t adr_offset      = 0x03;
const uint8_t adr_modulcount  = 0x04;
const uint8_t adr_status      = 0x05;

bool gotInput=false;
uint8_t offset = 0;
const uint8_t maxoffset = 4;

void processInt0();
void processInt1();
void boardnumAnswer();
void send_sensor_event(uint8_t address, uint8_t value);
void PCF_Init();
uint8_t PCF_Read(int adr);
void sendConfig(int index);

// adjust addresses if needed
const int PCF_base_adrs = 0x38;
const uint8_t maxmodulcount = 8;
uint8_t modulcount = 1;
const uint8_t inp_per_module = 8;

typedef struct __attribute__((__packed__))
{
  uint8_t address;
  uint8_t sensors[inp_per_module];
} PCFdatastruct;

PCFdatastruct PCF[maxmodulcount];

uint8_t status[inp_per_module*maxmodulcount];

// config-Daten
#define CONFIG_NUM 4     // Anzahl der Konfigurationspunkte
int config_index = 0;
bool uid_request;

#define VERS_HIGH     0x00  // Versionsnummer vor dem Punkt
#define VERS_LOW      0x05  // Versionsnummer nach dem Punkt

uint32_t UID;

void setup()
{
  uint8_t setup_todo;
  setup_todo = eeprom_read_byte(adr_setup_done);
  if (setup_todo != setup_done){
    // wurde das Setup bereits einmal durchgeführt?
    // dann wird dieser Anteil übersprungen
    // 47, weil das EEPROM (hoffentlich) nie !ursprünglich! diesen Inhalt hatte

    // setzt die Boardnum anfangs auf NULL
    eeprom_update_byte (( uint8_t *) adr_HiByte, '0');
    eeprom_update_byte (( uint8_t *) adr_LoByte, '0');
    // setzt den offset (Anzahl der Rückmelder) anfangs auf NULL
    eeprom_update_byte (( uint8_t *) adr_offset, 0);

    // setzt die Anzahl Module anfangs auf EINS
    eeprom_update_byte (( uint8_t *) adr_modulcount, modulcount);

    // status - des Rückmeldekontaktes im Steuerungssystem - auf NULL setzen
    for (uint8_t i=0; i<inp_per_module*modulcount; i++)
    {
      // '0' ist AUS
      status[i] = 0;
      eeprom_update_byte (( uint8_t *) adr_status+i, status[i]);
    }
    // setup_done auf "TRUE" setzen
    eeprom_update_byte (( uint8_t *) adr_setup_done, setup_done);
  }
  else
  {
    // Modulanzahl wird eingelesen
      modulcount = eeprom_read_byte(( uint8_t *) adr_modulcount);

    // status wird eingelesen, der Zustand beim letzten Beenden
    for (uint8_t i=0; i<inp_per_module*modulcount; i++)
      status[i] = eeprom_read_byte(( uint8_t *) adr_status+i);
  }
  // ab hier werden die Anweisungen bei jedem Start durchlaufen
  CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
  CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
  CAN.params.moduladr = ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0');
  offset = eeprom_read_byte(( uint8_t *) adr_offset);
  if (offset>maxoffset)
    offset = 0;
  UID = generateUID(UID_BASE, &CAN.params);
  CAN.begin(CAN_BPS_250K);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  //serial Monitor and CAN bus monitor. It can be removed later...
  CAN.hash = generateHash(UID);
  pinMode(PIN_INT0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT0), processInt0, LOW);
  //
  PCF_Init();
}

// Test rapid fire ping/pong of extended frames
void loop() {
// nur bei Interrupt1
  if (gotInput==true) {
    for (uint8_t j = 0; j < modulcount; j++) {
      uint8_t value = PCF_Read(PCF[j].address) ^ 0xFF;
      uint8_t mask = 0x01;
      for (uint8_t i=1; i<= inp_per_module; i++) {
        uint8_t v = 0;
        if (value & mask)
          v = 1;
        if (PCF[j].sensors[i-1]!=v){
          // sensoränderung liegt vor
          PCF[j].sensors[i-1]=v;
          if (v==1){
            // sensor ist aktiv
            uint8_t num =j * inp_per_module + i;
            if (status[num]==1)
              status[num] = 0;
            else
              status[num] = 1;
            eeprom_update_byte (( uint8_t *) adr_status+num, status[num]);
            send_sensor_event(num, status[num]);
          }
          Wire.beginTransmission(PCF[j].address);
          Wire.write(0xFF);
          Wire.endTransmission();
        }
        mask <<= 1;
      }
    }
    gotInput=false;
    _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
    attachInterrupt(digitalPinToInterrupt(PIN_INT1), processInt1, CHANGE);
  }
}

void PCF_Init() {
  pinMode(PIN_INT1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT1), processInt1, CHANGE);
  /* PCF class */
  Wire.begin();
  for (uint8_t j = 0; j < modulcount; j++) {
    PCF[j].address = PCF_base_adrs + j;
    // Preset sensor values
    for (uint8_t i = 0; i < inp_per_module; i++){
    // alle sensoren sind passiv
      PCF[j].sensors[i] = 1;
      // aktuellen status an zentrale melden
      uint8_t num =j * inp_per_module + i;
      send_sensor_event(num, status[num]);
      _delay_ms(wait_time);  // Delay added just so we can have time to open up
      }
    Wire.beginTransmission(PCF[j].address);
    Wire.write(0xFF);
    Wire.endTransmission();
  }
  processInt1();
}

/*
 * get input data
 */
uint8_t PCF_Read(int adr){
  uint8_t _data = 0;
//  Wire.beginTransmission(adr);
  Wire.requestFrom(adr, 1);
  while (!Wire.available())
  {
  }
  _data = Wire.read();
  Wire.endTransmission();
  return _data;
}

void send_sensor_event(uint8_t address, uint8_t value)
{
  CAN.outgoingMsg.cmd = S88_EVENT;
  // Gerätekenner
  // Hi
  CAN.outgoingMsg.data[0] = 0;
  // Lo
  CAN.outgoingMsg.data[1] = offset;
  // Kontaktkennung
  // Hi
  CAN.outgoingMsg.data[2] = ((16 * offset + address) >> 8) & 0x000000FF;
  // Lo
  CAN.outgoingMsg.data[3] = (16 * offset + address) & 0x000000FF;
  // Zustand
  if (value==1)
  {
    // alt
    CAN.outgoingMsg.data[4] = 0;
    // neu
    CAN.outgoingMsg.data[5] = 1;
  } else
  {
    // alt
    CAN.outgoingMsg.data[4] = 1;
    // neu
    CAN.outgoingMsg.data[5] = 0;
  }
  // Zeit
  CAN.outgoingMsg.data[6] = 0;
  CAN.outgoingMsg.data[7] = 0;
  CAN.can_answer(8);
}

/*
   Ausführen, wenn eine Nachricht verfügbar ist.
   Nachricht wird geladen und anhängig vom CAN-Befehl verarbeitet.
*/
//Interrupt Service Routine for INT0
void processInt0()
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
            modulcount = CAN.incomingMsg.data[6]*16+CAN.incomingMsg.data[7];
            // speichert die Anzahl der Module
            eeprom_update_byte (( uint8_t *) adr_modulcount, modulcount);
            break;
          // Kanalnummer #2
          case 2:
            offset = CAN.incomingMsg.data[6]*16+CAN.incomingMsg.data[7];
            // speichert die Anzahl der Rückmelder
            eeprom_update_byte (( uint8_t *) adr_offset, offset);
          break;
          // Kanalnummer #3
          case 3:
            CAN.params.moduladr = CAN.incomingMsg.data[6]*16+CAN.incomingMsg.data[7];
            // speichert die neue Adresse
            CAN.params.LoByteAddress = (uint8_t)CAN.incomingMsg.data[7]+'0';
            CAN.params.HiByteAddress = (uint8_t)CAN.incomingMsg.data[6]+'0';
            eeprom_update_byte (( uint8_t *) adr_HiByte, CAN.params.HiByteAddress);
            eeprom_update_byte (( uint8_t *) adr_LoByte, CAN.params.LoByteAddress);
          // Kanalnummer #4
          case 4:
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
      CAN.outgoingMsg.data[6] = DEVTYPE_RM >> 8;
      CAN.outgoingMsg.data[7] = DEVTYPE_RM;
      CAN.can_answer(8);
      break;
      // config
      case CONFIG_Status:
      uid_request = true;
      for (uint8_t i=0; i<uid_num; i++)
      uid_request = uid_request && (CAN.params.uid_device[i] == CAN.incomingMsg.data[i]);
      if (uid_request==true) {
        config_index = CAN.incomingMsg.data[4];
        // Konfiguration kann nicht wie sonst in loop gesendet
        // werden, da evtl. keine PCF angeschlossen sind und
        // dann die loop nicht durchlaufen wird. Aber der interrupt 0
        // funktioniert immer
        sendConfig(config_index);
      }
     break;
      // S88_Polling-Abfragen beantworten
      case S88_Polling:
      // CMD  DLC  0   1   2   3   4
      // 10   5    Geräte UID      Modul-
      //           High        Low anzahl
      // 10   5    01  02  03  04  00
        CAN.outgoingMsg.cmd = S88_Polling;
        for (uint8_t i=1; i<= inp_per_module; i++)
          send_sensor_event(i,0);
        break;
      case S88_EVENT:
      // CMD  DLC  0   1   2   3
      // 11   4    Geräte UID
      //           High        Low
        CAN.outgoingMsg.cmd = S88_EVENT;
        CAN.outgoingMsg.data[0] = CAN.incomingMsg.data[0];
        CAN.outgoingMsg.data[1] = CAN.incomingMsg.data[1];
        CAN.outgoingMsg.data[2] = CAN.incomingMsg.data[2];
        CAN.outgoingMsg.data[3] = CAN.incomingMsg.data[3];
        CAN.outgoingMsg.data[4] = 0;
        CAN.outgoingMsg.data[5] = 0;
        CAN.outgoingMsg.data[6] = 0;
        CAN.outgoingMsg.data[7] = 0;
        CAN.can_answer(8);
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
              CAN.hash = generateHash(UID);
              CAN.params.moduladr = ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0');
              break;
          }
        }
        break;
    }
  }
}

void processInt1()
{
 gotInput=true;
 detachInterrupt(digitalPinToInterrupt(PIN_INT1));
}

void boardnumAnswer(){
  CAN.outgoingMsg.data[0] = BOARDNUM_ANSWER;
  CAN.outgoingMsg.data[1] = CAN.params.HiByteAddress;
  CAN.outgoingMsg.data[2] = CAN.params.LoByteAddress;
  what_is_your_name(I_am_a_hall2can, 3, &CAN.outgoingMsg);
  CAN.can_answer(6);
}

void sendConfig(int index) {
  uint8_t config_len[] = {5, 4, 5, 4, 4};
  uint8_t config_frames[][5][8] = {
// #0
    {{0, CONFIG_NUM, 0, 0, 0, 0, 0, CAN.params.moduladr},
    {( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[0])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[0])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[1])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[1])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[2])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[2])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[3])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[3]))},
    {'C', 'A', 'N', 'g', 'u', 'r', 'u', ' '},
    {'R', 0xc3, 0xbc, 'c', 'k', 'm', 'e', 'l'},
    {'d', 'e', 'r', 0, 0, 0, 0, 0}},
// #1
    {{1, 2, 0, 1, 0, maxmodulcount, 0, modulcount},
    {'A', 'n', 'z', 'a', 'h', 'l', ' ', 'E'},
    {'x', 'p', 'a', 'n', 'd', 'e', 'r', 0},
    {'1', 0, maxmodulcount+'0', 0, 'S', 't', 'k', 0 }},
// #2
    {{2, 2, 0, 0, 0, maxoffset, 0, offset},
    {'N', 'u', 'm', 'm', 'e', 'r', ' ', 'R'},
    { 0xc3, 0xbc, 'c', 'k', 'm', 'e', 'l', 'd'},
    {'e', 'r', 0, '0', 0, maxoffset+'0', 0, 'N'},
    {'u', 'm', 0, 0, 0, 0, 0, 0 }},
// #3
    {{3, 2, 0, 0, 0, maxadr, 0, CAN.params.moduladr},
    {'M', 'o', 'd', 'u', 'l', 'a', 'd', 'r'},
    {'e', 's', 's', 'e', 0, '0', 0, (uint8_t)(maxadr/10)+'0'},
    {maxadr-(uint8_t)(maxadr/10)*10+'0' ,0, 'A', 'd', 'r', 0, 0, 0 }},
// #4
    {{4, 1, 3, 0, 0, 0, 0, 0},
    {'N', 'e', 'u', 's', 't', 'a', 'r', 't'},
    {0, 'N', 'e', 'i', 'n', 0, 'W', 'a'},
    {'r', 'm', 0, 'K', 'a', 'l', 't', 0}}
   };
  for (int i = 0; i < config_len[index]; i++) {
    CAN.configDataFrame(config_frames[index][i], i);
  }
  CAN.configTerminator(index, config_len[index]);
}
