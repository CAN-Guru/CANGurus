/*

*/

#ifndef OWN_CAN_h
#define OWN_CAN_h

#ifndef hex2usb
#include "CAN.h"
#endif // !hex2usb

// allgemein
#define wait_time_long	500
#define wait_time   		125
#define max_char		    50
#define min_char	    	2
#define min_char1	    	3
#define baudrate	    	19200

/* Zustände des Hex-File-Parsers */
#define PARSER_STATE_START    0
#define PARSER_STATE_SIZE     1
#define PARSER_STATE_ADDRESS  2
#define PARSER_STATE_TYPE     3
#define PARSER_STATE_DATA     4
#define PARSER_STATE_CHECKSUM 5
#define PARSER_STATE_ERROR	  6
#define PARSER_STATE_FINISH   7

/* Zustände des Bootloader-Programms */
#define BOOT_STATE_EXIT			0
#define BOOT_STATE_PARSER		1
#define BOOT_STATE_DATA_READY	2
#define BOOT_STATE_ERROR		3

/*
 *  Gerätetypen
 */
#define DEVTYPE_BASE      0x0050
#define DEVTYPE_USB2CAN   0x0051
#define DEVTYPE_CAN2USB   0x0052
#define DEVTYPE_SERVO     0x0053
#define DEVTYPE_RM        0x0054
#define DEVTYPE_LIGHT     0x0055
#define DEVTYPE_SIGNAL    0x0056

/*
 * Adressbereiche:
*/
#define MM_ACC 		0x3000	  //Magnetartikel Motorola
#define DCC_ACC 	0x3800	  //Magbetartikel NRMA_DCC
#define MM_TRACK 	0x0000	  //Gleissignal Motorola
#define DCC_TRACK 	0xC000	//Gleissignal NRMA_DCC

/*
 * CAN-Befehle (Märklin)
*/
#define SYS_CMD		          0x00 	//Systembefehle
#define SYS_STOP 	          0x00 	//System - Stop
#define SYS_GO		          0x01	//System - Go
#define Lok_Discovery       0x01
#define SYS_HALT	          0x02	//System - Halt
#define MFX_Bind            0x02
#define MFX_Verify          0x03
#define Lok_Speed           0x04
#define Lok_Direction       0x05
#define Lok_Function        0x06
#define Read_Config         0x07
#define Write_Config        0x08
#define SYS_STAT	          0x0b	//System - Status (sendet geänderte Konfiguration)
#define SWITCH_ACC 	        0x0B	//Magnetartikel schalten
#define CONFIG_ACC          0x0C
#define S88_Polling         0x10
#define S88_EVENT	          0x11	//Rückmelde-Event
#define SX1_Event           0x12
#define PING 		            0x18	//CAN-Teilnehmer anpingen
#define Offer_Update        0x19
#define Read_Config_Data    0x1A
#define Bootloader_CAN      0x1B
#define Bootloader_Track    0x1C
#define CONFIG_Status       0x1D
#define Data_Query          0x20
#define Config_Data_Stream  0x21

/*
 * CAN-Befehle (eigene)
*/
#define FOR_BTLDR       0x50	//Bootloader abfragen
#define BTLDR_ANSWER    0x51	//Bootloader antwortet
#define FOR_APP         0x52	//Dekoderapp abfragen
#define APP_ANSWER      0x53	//Dekoderapp antwortet

#define BOARDNUM_REQUEST  0
#define BOARDNUM_ANSWER   1
#define BOARDNUM_CHANGE   2
#define GO_BTLDR          3
#define START_DATA        4
#define MORE_DATA         5
#define END_DATA          6
#define TEST_DATA         0x99

//CBR_19200
#define limiter			'#'
#define findPort		'!'
#define getboardNum		"?"
#define goBLState		"&"
#define transferbtldr	"%"
#define newbrdnr		"="

#define TRM_BOARD_NUM     99

#define UID_BASE  0x50091900ULL    //CAN-UID
#define UID_RM    0x80220B01ULL    //CAN-UID für Rückmelder

// converts highbyte of integer to char
char highbyte2char(int num);
// converts lowbyte of integer to char
char lowbyte2char(int num);
// gives the name of the module

#ifndef hex2usb

const uint8_t PIN_INT0 = 2;
const uint8_t PIN_INT1 = 3;

#define name_count  3
const uint8_t I_am_a_usb2can [name_count] = {'u', '2', 'c'}; 
const uint8_t I_am_a_can2usb [name_count] = {'c', '2', 'u'}; 
const uint8_t I_am_a_NanoApp[name_count] = {'a', 'p', 'p'};
const uint8_t I_am_a_NanoBase[name_count] = {'b', 's', 'e'};
const uint8_t I_am_a_hall2can[name_count] = {'h', '2', 'c'};
void what_is_your_name(const uint8_t name[], uint8_t offset, CAN_Frame *outMsg);

uint8_t hex2dec(uint8_t h);
const uint8_t maxadr = 20;

// generates the specific UID
uint32_t generateUID(uint32_t uid, deviceparams *p);
// generates the hashcode
uint16_t generateHash(uint32_t uid);
// sends a canframe
void sendCanFrame(CAN_Frame frame);
//receives a canframe
CAN_Frame getCanFrame();
//
void goIntoBootloader();
#endif // !hex2usb

#endif