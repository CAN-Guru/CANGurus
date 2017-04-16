/*
 * NanoBtLdr.cpp
 *
 * Created: 21.11.2016 10:27:24
 * Author : Gustav
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/boot.h>

#include "ownCAN.h"
#include "CAN.h"

#define max_hex_data  7

CAN_Frame canFrame;
unsigned char temp; // Variable
uint16_t hash;
  // Datenpuffer f³r die Hexdaten
uint8_t flash_data[SPM_PAGESIZE];
  // Datenpuffer f³r die ankommenden Daten
  uint8_t hex_data[max_hex_data];
  // Zu schreibende Flash-Page 
uint16_t  flash_page = 0;
  // Position zum Schreiben in der Datenpuffer 
uint16_t  flash_cnt = 0;
  // Flag zum Steuern des Programmiermodus 
uint8_t  boot_state = BOOT_STATE_PARSER;
  // Empfangszustandssteuerung 
uint8_t  parser_state = PARSER_STATE_DATA;
  // Position zum Schreiben in den HEX-Puffer 
uint8_t  hex_cnt = 0;
  // Anzahl noch unverarbeiteter Daten
uint8_t  max_act_dataPtr = 0;
  // Zeiger auf das nächste (!) Datum; von 1 .. length-1
uint8_t  act_dataPtr = max_hex_data;

void setup()
{
  // Füllen der Puffer mit definierten Werten 
  memset(flash_data, 0xFF, sizeof(flash_data));
  char sregtemp = SREG;
  cli();
  temp = MCUCR;
  MCUCR = temp | (1 << IVCE);
  MCUCR = temp | (1 << IVSEL);
  SREG = sregtemp;
  //Set CAN speed. Note: Speed is now 500kbit/s so adjust your CAN monitor
  CANBase.begin(CAN_BPS_250K);
  _delay_ms(5*wait_time);  // Delay added just so we can have time to open up //serial Monitor and CAN bus monitor. It can be removed later...
  hash = generateHash(UID_BASE);
  sei();
  canFrame.cmd = BTLDR_ANSWER;
  canFrame.data[0] = START_DATA;
  canFrame.hash = hash;
  canFrame.resp_bit = true;
  canFrame.length = 1;
  sendCanFrame(canFrame);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up //serial Monitor and CAN bus monitor. It can be removed later...
}

/**
 * \brief	write a complete page to the flash memorey
 *
 * \param	page	page which should be written
 * \param	*buf	Pointer to the buffer with the data
 *
 * \see		avr-libc Documentation > Modules > Bootloader Support Utilities
 */
void program_page (uint16_t pageNo, uint8_t *buf)
{
  uint16_t i;
  uint8_t sreg;
  uint32_t adr = pageNo * SPM_PAGESIZE;
  // Disable interrupts 
  sreg = SREG;    //
  cli();    //
  eeprom_busy_wait ();    //
  boot_page_erase (adr);
  boot_spm_busy_wait (); // Wait until the memory is erased. 
  for (i = 0; i < SPM_PAGESIZE; i += 2)
  {
    // Set up little-endian word. 
    uint16_t w = *buf++;
    w += (*buf++) << 8;
    boot_page_fill (adr + i, w);
  }
  boot_page_write (adr); // Store buffer in flash page. 
  boot_spm_busy_wait(); // Wait until the memory is written.
  // Reenable RWW-section again. We need this if we want to jump back 
  // to the application after bootloading. 
  boot_rww_enable ();
  // Re-enable interrupts (if they were ever enabled).
  SREG = sreg;    //
  //
}

static uint8_t can_getc() {
  bool noDATA = true;
  if (act_dataPtr<=max_act_dataPtr){
    act_dataPtr++;
    return((uint8_t) hex_data[act_dataPtr-1]);
  }
  // Mehr Daten anfordern
  // cmd und data[0] sind vom Eingang noch besetzt
  // canFrame.cmd = BTLDR_ANSWER;
  canFrame.data[0] = MORE_DATA;
  canFrame.hash = hash;
  canFrame.resp_bit = true;
  canFrame.length = 1;
  sendCanFrame(canFrame);
  do 
  {
    if (CANBase.available()){
      canFrame = getCanFrame();
      // neue Daten ?
      if ((canFrame.cmd == BTLDR_ANSWER) &&
         (canFrame.resp_bit == false)){
        noDATA = false;
        if(canFrame.data[0] == MORE_DATA) {
          // Länge - subcmd - ein hiermit übermitteltes Datum
          max_act_dataPtr = canFrame.length-2;
          for (uint8_t i=0; i<=max_act_dataPtr; i++)
          {
            hex_data[i] = canFrame.data[i+1];
          }
          // 0: subcmd, 1: das hiermit übermitteltes Datum, 2: das nächste Datum
          act_dataPtr = 1;
        }
        if (canFrame.data[0] == END_DATA) {
          // keine Daten mehr
          parser_state = PARSER_STATE_FINISH;
        }
      }
    }      
  } while (noDATA);
  return((uint8_t) hex_data[0]);
}

void BackToApp() {
  // relocate interrupt vectors
  uint8_t reg = MCUCR & ~((1 << IVCE) | (1 << IVSEL));
  MCUCR = reg | (1 << IVCE);
  MCUCR = reg;
  // reset SPI interface to power-up state
  SPCR = 0;
  SPSR = 0;
  __asm__ __volatile__(
  "push __zero_reg__" "\n\t"
  "push __zero_reg__" "\n\t");
  // when the functions executes the 'ret' command to return to
  // its origin the AVR loads the return address from the stack. Because we
  // pushed null it instead jumps to address null which starts the main
  // application.
}

void loop()
{ 
  uint8_t c = can_getc();
  switch (parser_state)
  {
    // Parse Flash-Daten
    case PARSER_STATE_DATA:
      flash_data[flash_cnt] = c;
      flash_cnt++;
      // Puffer voll -> schreibe Page
      if (flash_cnt == SPM_PAGESIZE)
        {
          program_page((uint16_t)flash_page, flash_data);
          memset(flash_data, 0xFF, sizeof(flash_data));
          flash_cnt = 0;
          flash_page++;
        }
    break;
    // Ende
    case PARSER_STATE_FINISH:
      program_page((uint16_t)flash_page, flash_data);
      boot_state = BOOT_STATE_EXIT;
    break;
  }
  if (boot_state == BOOT_STATE_EXIT)
    BackToApp();
}
