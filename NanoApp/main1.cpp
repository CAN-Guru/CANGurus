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

#define GenerateHash generateHash(UID+HiByteAddress+3*LoByteAddress)

#define  setup_done 0x047

// Protokollkonstante
#define PROT  MM_ACC

// Anzahl der Magnetartikel
#define num_accs 4

enum position
{
  left, right
};

//
typedef struct __attribute__((__packed__))
{
  Servo servo;
  uint16_t acc_locid;
  bool acc_got_cmd;
  position acc_pos_set;
  position acc_pos_is;
  uint8_t acc_pin_out;
  uint8_t reg_locid;    //EEPROM-Speicherplätze der Local-IDs
} servodatastruct;

// an diese PINs werden die Magnetartikel angeschlossen
#define PIN_0 3
#define PIN_1 4
#define PIN_2 5
#define PIN_3 6

#define PIN_INT 2

#define leftpos   120
#define rightpos  30

// EEPROM-Adressen
const uint8_t adr_setup_done = 0x00;
const uint8_t adr_HiByte = 0x01;
const uint8_t adr_LoByte = 0x02;

// locids benötigen 2 byte
#define locid0          0x03
#define locid1          0x05
#define locid2          0x07
#define locid3          0x09
#define acc_state       0x10

// EEPROM-Belegung
// adr_setup_done 00
// adr_HiByte     01
// adr_LoByte     02
// EEPROM-Speicherplätze der Local-IDs
// Stellung der Magnetartikel 0xa0

void processRXFrame();
void goIntoBootloader();
void boardnumAnswer();
void can_answer(uint8_t lng);
void switchAcc(uint8_t acc_num);
void acc_report(uint8_t num);
void calc_locid(bool report);

uint16_t hash;
CAN_Frame outgoingMsg, incomingMsg;
const uint8_t reg_locids[num_accs] = {locid0, locid1, locid2, locid3};    //EEPROM-Speicherplätze der Local-IDs
const uint8_t acc_pin_outs[num_accs] = {PIN_0, PIN_1, PIN_2, PIN_3};    //PIN-Zuordnung

/*
   Variablen der Servos & Magnetartikel
*/
servodatastruct Servos[num_accs];

void setup()
{
  for (uint8_t i = 0; i < num_accs; i++) {
    Servos[i].acc_pin_out = acc_pin_outs[i];
    Servos[i].reg_locid = (uint8_t*) reg_locids[i];
  }
  uint8_t setup_todo;
  setup_todo = eeprom_read_byte(adr_setup_done);
  if (setup_todo != setup_done){
    // wurde das Setup bereits einmal durchgeführt?
    // dann wird dieser Anteil übersprungen
    // 47, weil das EEPROM (hoffentlich) nie ursprünglich diesen Inhalt hatte

    // setzt die Boardnum anfangs auf NULL
    eeprom_update_byte (( uint8_t *) adr_HiByte, '0');
    eeprom_update_byte (( uint8_t *) adr_LoByte, '1');

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
    HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
    LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
    // locids einlesen in lokales array
    for (int i = 0; i < num_accs; i++) {
      Servos[i].acc_locid = ( uint16_t *)(eeprom_read_byte(Servos[i].reg_locid) << 8 | eeprom_read_byte(Servos[i].reg_locid + 1));
    }
  }
  // ab hier werden die Anweisungen bei jedem Start durchlaufen
  CAN.begin(CAN_BPS_250K);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  //serial Monitor and CAN bus monitor. It can be removed later...
  hash = GenerateHash;
  attachInterrupt(digitalPinToInterrupt(PIN_INT), processRXFrame, LOW);
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

  HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
  LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
  uint8_t baseaddress = ((HiByteAddress - '0') * 10 + LoByteAddress - '0' - 1) * 4;

  for (uint8_t i = 0; i < num_accs; i++) {
    uint16_t locid = PROT + baseaddress + i;
    uint8_t locid_high = locid >> 8;
    uint8_t locid_low = locid;
    eeprom_update_byte(Servos[i].reg_locid, locid_high);
    eeprom_update_byte(Servos[i].reg_locid + 1, locid_low);
    // locids einlesen in lokales array
    Servos[i].acc_locid = ( uint16_t *)(eeprom_read_byte(Servos[i].reg_locid) << 8 | eeprom_read_byte(Servos[i].reg_locid + 1));
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
}

/*
   Ausführen, wenn eine Nachricht verfügbar ist.
   Nachricht wird geladen und anhängig vom CAN-Befehl verarbeitet.
*/
//Interrupt Service Routine for INT0
void processRXFrame()
{
  incomingMsg = getCanFrame();
  if (incomingMsg.resp_bit == false)
  {
    switch (incomingMsg.cmd)
    {
      // PING-Abfragen beantworten
      case PING:
        outgoingMsg.cmd = PING;
        can_answer(0);
  	    break;
      // alle Aufträge von usb2can abarbeiten
      case FOR_APP:
        if ((incomingMsg.data[1] == HiByteAddress) &&
            (incomingMsg.data[2] == LoByteAddress)) {
          outgoingMsg.cmd = APP_ANSWER;
          switch (incomingMsg.data[0])
          {
            case GO_BTLDR:
              goIntoBootloader();
              break;
            case BOARDNUM_REQUEST:
              boardnumAnswer();
              break;
            case BOARDNUM_CHANGE:
              // Reihenfolge wichtig, damit mit der alten Boardnum geantwortet wird
              boardnumAnswer();
              eeprom_update_byte(( uint8_t *) adr_HiByte, incomingMsg.data[3]);
              eeprom_update_byte(( uint8_t *) adr_LoByte, incomingMsg.data[4]);
              HiByteAddress = eeprom_read_byte(( uint8_t *) adr_HiByte);
              LoByteAddress = eeprom_read_byte(( uint8_t *) adr_LoByte);
              hash = GenerateHash;
              // berechnet die locids neu
              calc_locid(true);
              break;
          }
        }
        break;
      case SWITCH_ACC:
        //Abhandlung nur bei gültigem Weichenbefehl
        uint16_t* locid = (uint16_t*) ((incomingMsg.data[2] << 8) | incomingMsg.data[3]);
        for (int i = 0; i < num_accs; i++) {
          //Auf benutzte Adresse überprüfen
          if (locid == Servos[i].acc_locid) {
            Servos[i].acc_pos_set = (position) incomingMsg.data[4];
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

void goIntoBootloader() {
  outgoingMsg.data[0] = GO_BTLDR;
  outgoingMsg.data[1] = HiByteAddress;
  outgoingMsg.data[2] = LoByteAddress;
  can_answer(3);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  // jumping into the Bootloader
  asm volatile ("\tjmp 0x7000 \n\t");
}

void boardnumAnswer(){
  outgoingMsg.data[0] = BOARDNUM_ANSWER;
  outgoingMsg.data[1] = HiByteAddress;
  outgoingMsg.data[2] = LoByteAddress;
  outgoingMsg.data[3] = '0';
  can_answer(4);
}

void can_answer(uint8_t lng){
  outgoingMsg.hash = hash;
  outgoingMsg.resp_bit = true;
  outgoingMsg.length = lng;
  sendCanFrame(outgoingMsg);
}

void GoLeft(uint8_t i) {
  for ( uint8_t pos = rightpos; pos <= leftpos; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 10 degree
    Servos[i].servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                 // waits 15ms for the servo to reach the position
  }
}

void GoRight(uint8_t i) {
  for ( uint8_t pos = leftpos; pos >= rightpos; pos -= 1) { // goes from 180 degrees to 0 degrees
    Servos[i].servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                 // waits 15ms for the servo to reach the position
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
  outgoingMsg.cmd = SWITCH_ACC;
  memset(outgoingMsg.data, 0x0, 0x8);
  outgoingMsg.data[2] = (uint8_t) (Servos[num].acc_locid >> 8);
  outgoingMsg.data[3] = (uint8_t) Servos[num].acc_locid;
  outgoingMsg.data[4] = Servos[num].acc_pos_is;            /* Meldung der Lage für Märklin-Geräte.*/
  can_answer(6);
}
