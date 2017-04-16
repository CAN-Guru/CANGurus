/*
 * NanoApp.cpp
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
#include "Servo.h"

// Protokollkonstante
#define PROT  MM_ACC

// Anzahl der Magnetartikel
#define num_accs 4

// EEPROM-Adressen
#define  setup_done 0x047
const uint8_t adr_setup_done = 0x00;

const uint8_t adr_HiByte = 0x01;
const uint8_t adr_LoByte = 0x02;
const uint8_t adr_SrvDel = 0x03;
// locids benötigen 2 byte
#define locid0          0x04
#define locid1          0x06
#define locid2          0x08
#define locid3          0x0A
const uint8_t reg_locids[num_accs] = {locid0, locid1, locid2, locid3};    //EEPROM-Speicherplätze der Local-IDs
const uint8_t acc_state  = 0x0C;  // ab dieser Adresse werden die Weichenstellungen gespeichert

// config-Daten
#define CONFIG_NUM 3     // Anzahl der Konfigurationspunkte
int config_index = 0;
bool config_request = false;
bool uid_request;

#define VERS_HIGH     0x00  // Versionsnummer vor dem Punkt
#define VERS_LOW      0x06  // Versionsnummer nach dem Punkt

uint32_t UID;

// EEPROM-Belegung
// adr_setup_done 00
// adr_HiByte     01
// adr_LoByte     02
// adr_SrvDel     03
// EEPROM-Speicherplätze der Local-IDs
// Stellung der Magnetartikel 0xa0

void processRXFrame();
void boardnumAnswer();
void switchAcc(uint8_t acc_num);
void acc_report(uint8_t num);
void calc_locid(bool report);
void sendConfig(int index);

/*
   Variablen der Servos & Magnetartikel
*/
Sweeper Servos[num_accs];
uint8_t servoDelay;

// an diese PINs werden die Magnetartikel angeschlossen
#define PIN_0 4
#define PIN_1 5
#define PIN_2 6
#define PIN_3 7
const uint8_t acc_pin_outs[num_accs] = {PIN_0, PIN_1, PIN_2, PIN_3};    //PIN-Zuordnung

void setup()
{
  for (uint8_t i = 0; i < num_accs; i++) {
    Servos[i].SetRegID(reg_locids[i]);
  }
  uint8_t setup_todo;
  setup_todo = eeprom_read_byte(adr_setup_done);
  if (setup_todo != setup_done){
    // wurde das Setup bereits einmal durchgeführt?
    // dann wird dieser Anteil übersprungen
    // 47, weil das EEPROM (hoffentlich) nie ursprünglich diesen Inhalt hatte

    // setzt die Boardnum anfangs auf NULL
    eeprom_update_byte (( uint8_t *) adr_HiByte, '0');
    eeprom_update_byte (( uint8_t *) adr_LoByte, '0');
    eeprom_update_byte (( uint8_t *) adr_SrvDel, stdservodelay);

    // Berechnen der locids
    calc_locid(false);

    // Status der Magnetartikel zu Beginn auf links setzen
    for (int i = 0; i < num_accs; i++) {
      eeprom_update_byte(( uint8_t *) acc_state + i, left);
    }

    // setup_done auf "TRUE" setzen
    eeprom_update_byte (( uint8_t *) adr_setup_done, setup_done);
  }
  else
  {
    CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
    CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
    // locids einlesen in lokales array
    for (int i = 0; i < num_accs; i++) {
      Servos[i].SetLocID(
        ( uint16_t )( eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].GetRegID()) << 8 |
                      eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].GetRegID() + 1)));
    }
  }
  // ab hier werden die Anweisungen bei jedem Start durchlaufen
  servoDelay = eeprom_read_byte(( uint8_t *) adr_SrvDel);
  CAN.params.moduladr = ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0');
  UID = generateUID(UID_BASE, &CAN.params);
  CAN.begin(CAN_BPS_250K);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  //serial Monitor and CAN bus monitor. It can be removed later...
  CAN.hash = generateHash(UID);
  pinMode(PIN_INT0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT0), processRXFrame, LOW);
  for (int i = 0; i < num_accs; i++) {
    // Status der Magnetartikel einlesen in lokale arrays
    Servos[i].SetPosCurr((position) eeprom_read_byte(( uint8_t *) acc_state + i));
    // Servos mit den PINs verbinden, initialisieren & Artikel setzen wie gespeichert
    Servos[i].Init(acc_pin_outs[i], servoDelay, Servos[i].GetPosCurr());
    acc_report(i);
  }
}

void calc_locid(bool report){
  // berechnet die locid aus der Adresse und der Protokollkonstante

  CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
  CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
  uint8_t baseaddress = ((CAN.params.HiByteAddress - '0') * 10 + CAN.params.LoByteAddress - '0' - 1) * 4;

  for (uint8_t i = 0; i < num_accs; i++) {
    uint16_t locid = PROT + baseaddress + i;
    eeprom_update_byte(( uint8_t *) ( uint16_t ) Servos[i].GetRegID(), locid >> 8);
    eeprom_update_byte(( uint8_t *) ( uint16_t ) Servos[i].GetRegID() + 1, locid);
    // locids einlesen in lokales array
    Servos[i].SetLocID(
    ( uint16_t )( eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].GetRegID()) << 8 |
                  eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].GetRegID() + 1)));
    if (report==true)
      acc_report(i);
  }
}

// main loop
void loop()
{
  for (int i = 0; i < num_accs; i++)
    Servos[i].Update();
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
     // Konfigurationswert ändern
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
              servoDelay = CAN.incomingMsg.data[6]*16+CAN.incomingMsg.data[7];
              eeprom_update_byte (( uint8_t *) adr_SrvDel, servoDelay);
              break;
		      // Kanalnummer #2
		      case 2:
    		      CAN.params.moduladr = CAN.incomingMsg.data[6]*16+CAN.incomingMsg.data[7];
		          // speichert die neue Adresse
		          CAN.params.LoByteAddress = CAN.incomingMsg.data[7]+'0';
    		      CAN.params.HiByteAddress = (uint8_t)CAN.incomingMsg.data[6]+'0';
		          eeprom_update_byte (( uint8_t *) adr_HiByte, CAN.params.HiByteAddress);
		          eeprom_update_byte (( uint8_t *) adr_LoByte, CAN.params.LoByteAddress);
		          break;
          // Kanalnummer #3
          case 3:
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
          CAN.outgoingMsg.data[6] = DEVTYPE_SERVO >> 8;
          CAN.outgoingMsg.data[7] = DEVTYPE_SERVO;
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
            case GO_BTLDR:
              for (int i = 0; i < num_accs; i++) {
                // Servos von den PINs entbinden
                Servos[i].Detach();
                _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
              }
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
              // berechnet die locids neu
              calc_locid(true);
              CAN.params.moduladr = ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0');
              break;
          }
        }
        break;
      case SWITCH_ACC:
        // Umsetzung nur bei gültiger Weichenadresse
        uint16_t locid = (uint16_t) ((CAN.incomingMsg.data[2] << 8) | CAN.incomingMsg.data[3]);
        for (int i = 0; i < num_accs; i++) {
          // Auf benutzte Adresse überprüfen
          if (locid == Servos[i].GetLocID()) {
            Servos[i].SetPosDest((position) CAN.incomingMsg.data[4]);
            // muss Artikel geändert werden?
            if (Servos[i].PosChg())
              switchAcc(i);
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

void switchAcc(uint8_t acc_num) {
  position set_pos = Servos[acc_num].GetPosDest();
  switch (set_pos)
    {
      case left:
        Servos[acc_num].GoLeft();
        break;
      case right:
        Servos[acc_num].GoRight();
        break;
    }
  acc_report(acc_num);
  Servos[acc_num].SetPosCurr(set_pos);
  eeprom_update_byte(( uint8_t *) acc_state + acc_num, set_pos);
}

void acc_report(uint8_t num){
  CAN.outgoingMsg.cmd = SWITCH_ACC;
  memset(CAN.outgoingMsg.data, 0x0, 0x8);
  CAN.outgoingMsg.data[2] = (uint8_t) (Servos[num].GetLocID() >> 8);
  CAN.outgoingMsg.data[3] = (uint8_t) Servos[num].GetLocID();
  CAN.outgoingMsg.data[4] = Servos[num].GetPosCurr();            /* Meldung der Lage für Märklin-Geräte.*/
  CAN.can_answer(6);
}

void sendConfig(int index) {
  uint8_t config_len[] = {4, 5, 4, 4};
  uint8_t config_frames[][5][8] = {
// #0
/*1*/    {{CONFIG_NUM-3, CONFIG_NUM, 0, 0, 0, 0, 0, CAN.params.moduladr},
/*2*/    {( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[0])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[0])),
          ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[1])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[1])),
          ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[2])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[2])),
          ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[3])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[3]))},
/*3*/    {'C', 'A', 'N', 'g', 'u', 'r', 'u', ' '},
/*4*/    {'S', 'e', 'r','v', 'o', 0, 0, 0}},
// #1
/*1*/    {{CONFIG_NUM-2, 2, 0, 5, 0, maxservodelay, 0, servoDelay},
/*2*/    {'S', 'e', 'r', 'v', 'o', 'v', 'e', 'r'},
/*3*/    {'z', 0xc3, 0xb6, 'g', 'e', 'r', 'u', 'n'},
/*4*/    {'g', 0, '5', 0, (uint8_t)(maxservodelay/10)+'0', maxservodelay-(uint8_t)(maxservodelay/10)*10+'0', 0, 'm'},
/*5*/    {'s', 0, 0, 0, 0, 0, 0, 0}},
// #2
/*1*/    {{CONFIG_NUM-1, 2, 0, 0, 0, maxadr, 0, CAN.params.moduladr},
/*2*/    {'M', 'o', 'd', 'u', 'l', 'a', 'd', 'r'},
/*3*/    {'e', 's', 's', 'e', 0, '0', 0, (uint8_t)(maxadr/10)+'0'},
/*4*/    {maxadr-(uint8_t)(maxadr/10)*10+'0' ,0, 'A', 'd', 'r', 0, 0, 0 }},
// #3
/*1*/    {{CONFIG_NUM, 1, 3, 0, 0, 0, 0, 0},
/*2*/    {'N', 'e', 'u', 's', 't', 'a', 'r', 't'},
/*3*/    {0, 'N', 'e', 'i', 'n', 0, 'W', 'a'},
/*4*/    {'r', 'm', 0, 'K', 'a', 'l', 't', 0}}
};

  for (int i = 0; i < config_len[index]; i++) {
    CAN.configDataFrame(config_frames[index][i], i);
  }
  CAN.configTerminator(index, config_len[index]);
}
