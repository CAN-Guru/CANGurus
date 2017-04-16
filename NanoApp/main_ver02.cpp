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

enum position
{ left, right };

//
struct __attribute__((__packed__)) servodatastruct
{
  Servo servo;
  uint16_t acc_locid;
  bool acc_got_cmd;
  position acc_pos_set;
  position acc_pos_is;
  uint8_t acc_pin_out;
  uint8_t reg_locid;    //EEPROM-Speicherplätze der Local-IDs
};

// EEPROM-Adressen
#define  setup_done 0x047
const uint8_t adr_setup_done = 0x00;

const uint8_t adr_HiByte = 0x01;
const uint8_t adr_LoByte = 0x02;
const uint8_t adr_SrvDel = 0x03;

// config-Daten
#define CONFIG_NUM 2     // Anzahl der Konfigurationspunkte
int config_index = 0;
bool config_request = false;
bool uid_request;

#define VERS_HIGH     0x00  // Versionsnummer vor dem Punkt
#define VERS_LOW      0x04  // Versionsnummer nach dem Punkt

// locids benötigen 2 byte
#define locid0          0x03
#define locid1          0x05
#define locid2          0x07
#define locid3          0x09
const uint8_t reg_locids[num_accs] = {locid0, locid1, locid2, locid3};    //EEPROM-Speicherplätze der Local-IDs

uint32_t UID;

#define acc_state       0x10

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
servodatastruct Servos[num_accs];
uint8_t ServoDelay;

// an diese PINs werden die Magnetartikel angeschlossen
#define PIN_0 4
#define PIN_1 5
#define PIN_2 6
#define PIN_3 7
const uint8_t acc_pin_outs[num_accs] = {PIN_0, PIN_1, PIN_2, PIN_3};    //PIN-Zuordnung

#define leftpos   70
#define rightpos  5
#define servodelay 25

void setup()
{
  for (uint8_t i = 0; i < num_accs; i++) {
    Servos[i].acc_pin_out = acc_pin_outs[i];
    Servos[i].reg_locid = reg_locids[i];
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
    eeprom_update_byte (( uint8_t *) adr_SrvDel, servodelay);

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
    ServoDelay = eeprom_read_byte(( uint8_t *) adr_SrvDel);
    // locids einlesen in lokales array
    for (int i = 0; i < num_accs; i++) {
      Servos[i].acc_locid = ( uint16_t )( eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].reg_locid) << 8 | eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].reg_locid + 1));
    }
  }
  // ab hier werden die Anweisungen bei jedem Start durchlaufen
  UID = generateUID(UID_BASE, &CAN.params);
  CAN.begin(CAN_BPS_250K);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  //serial Monitor and CAN bus monitor. It can be removed later...
  CAN.hash = generateHash(UID);
  pinMode(PIN_INT0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT0), processRXFrame, LOW);
  for (int i = 0; i < num_accs; i++) {
    // Status der Magnetartikel einlesen in lokale arrays
    Servos[i].acc_pos_is = (position) eeprom_read_byte(( uint8_t *) acc_state + i);
    // Artikel setzen wie gespeichert
    Servos[i].acc_pos_set = Servos[i].acc_pos_is;
    Servos[i].acc_got_cmd = false;
    // Servos mit den PINs verbinden
    Servos[i].servo.attach(Servos[i].acc_pin_out);
    switchAcc(i);
  }
}

void calc_locid(bool report){
  // berechnet die locid aus der Adresse und der Protokollkonstante

  CAN.params.HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
  CAN.params.LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
  ServoDelay = eeprom_read_byte(( uint8_t *) adr_SrvDel);
  uint8_t baseaddress = ((CAN.params.HiByteAddress - '0') * 10 + CAN.params.LoByteAddress - '0' - 1) * 4;

  for (uint8_t i = 0; i < num_accs; i++) {
    uint16_t locid = PROT + baseaddress + i;
    eeprom_update_byte(( uint8_t *) ( uint16_t ) Servos[i].reg_locid, locid >> 8);
    eeprom_update_byte(( uint8_t *) ( uint16_t ) Servos[i].reg_locid + 1, locid);
    // locids einlesen in lokales array
    Servos[i].acc_locid = ( uint16_t )( eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].reg_locid) << 8 | eeprom_read_byte(( uint8_t *) ( uint16_t ) Servos[i].reg_locid + 1));
    if (report==true)
      acc_report(i);
  }
}

// main loop
void loop()
{
  for (int i = 0; i < num_accs; i++) {
    if (Servos[i].acc_got_cmd==true) {
      //Nach eingegangenem Weichenbefehl schalten
      switchAcc(i);
      Servos[i].acc_got_cmd = false;
    }
  }
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
            case 1:
              ServoDelay = CAN.incomingMsg.data[6]*16+CAN.incomingMsg.data[7];
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
                Servos[i].servo.detach();
                _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
              }
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
              break;
          }
        }
        break;
      case SWITCH_ACC:
        // Umsetzung nur bei gültiger Weichenadresse
        uint16_t locid = (uint16_t) ((CAN.incomingMsg.data[2] << 8) | CAN.incomingMsg.data[3]);
        for (int i = 0; i < num_accs; i++) {
          // Auf benutzte Adresse überprüfen
          if (locid == Servos[i].acc_locid) {
            Servos[i].acc_pos_set = (position) CAN.incomingMsg.data[4];
            // muss Artikel geändert werden?
            if (Servos[i].acc_pos_set!=Servos[i].acc_pos_is)
              Servos[i].acc_got_cmd = true;
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

void GoLeft(uint8_t i) {
  for ( uint8_t pos = rightpos; pos <= leftpos; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 10 degree
    Servos[i].servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(ServoDelay);                 // waits 15ms for the servo to reach the position
  }
}

void GoRight(uint8_t i) {
  for ( uint8_t pos = leftpos; pos >= rightpos; pos -= 1) { // goes from 180 degrees to 0 degrees
    Servos[i].servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(ServoDelay);                 // waits 15ms for the servo to reach the position
  }
}

void GoServo(uint8_t i, position set_pos){
  if (set_pos==right)
    GoRight(i);
  else
    GoLeft(i);
}

void switchAcc(uint8_t acc_num) {
  position set_pos = Servos[acc_num].acc_pos_set;
  GoServo(acc_num, set_pos);
  acc_report(acc_num);
  Servos[acc_num].acc_pos_is = set_pos;
  eeprom_update_byte(( uint8_t *) acc_state + acc_num, set_pos);
}

void acc_report(uint8_t num){
  CAN.outgoingMsg.cmd = SWITCH_ACC;
  memset(CAN.outgoingMsg.data, 0x0, 0x8);
  CAN.outgoingMsg.data[2] = (uint8_t) (Servos[num].acc_locid >> 8);
  CAN.outgoingMsg.data[3] = (uint8_t) Servos[num].acc_locid;
  CAN.outgoingMsg.data[4] = Servos[num].acc_pos_is;            /* Meldung der Lage für Märklin-Geräte.*/
  CAN.can_answer(6);
}

void sendConfig(int index) {
  uint8_t config_len[] = {4, 5, 5};
  uint8_t config_frames[][5][8] = {{
    {CONFIG_NUM-2, 1, 0, 0, 0, 0, 0, ( uint8_t ) ((CAN.params.HiByteAddress - '0')*10 + CAN.params.LoByteAddress - '0')},
    {( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[0])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[0])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[1])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[1])), 
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[2])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[2])),
     ( uint8_t ) highbyte2char(hex2dec(CAN.params.uid_device[3])), ( uint8_t ) lowbyte2char(hex2dec(CAN.params.uid_device[3]))},
    {'C', 'A', 'N', 'g', 'u', 'r', 'u', ' '},
    {'S', 'e', 'r','v', 'o', 0, 0, 0}
    }, {
    {CONFIG_NUM-1, 2, 0, 5, 0, 50, 0, ServoDelay},
    {'S', 'e', 'r', 'v', 'o', 'v', 'e', 'r'},
    {'z', 0xc3, 0xb6, 'g', 'e', 'r', 'u', 'n'},
    {'g', 0, '5', 0, '5', '0', 0, 'm'},
    {'s', 0, 0, 0, 0, 0, 0, 0}
  }
};

  for (int i = 0; i < config_len[index]; i++) {
    CAN.configDataFrame(config_frames[index][i], i);
  }
  CAN.configTerminator(index, config_len[index]);
}
