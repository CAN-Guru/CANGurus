
#include "stdafx.h"
#include "CAN_Defs.h"

#ifdef hex2usb
#include "hex2usb.h"
#endif 

#include "ownCAN.h"

char highbyte2char(int num){
  num /= 10;
  return char ('0' + num);
}

char lowbyte2char(int num){
	num = num - num / 10 * 10;
return char ('0' + num);
}

#ifndef hex2usb

void what_is_your_name(const uint8_t name[], uint8_t offset, CAN_Frame *outMsg){
  for (uint8_t i=0; i<name_count; i++)
  {
    outMsg->data[i+offset] = name[i];
  }
}

uint8_t hex2dec(uint8_t h){
  return h / 16 * 10 + h % 16;
}

uint32_t generateUID(uint32_t uid, deviceparams *p){
  uid += (p->HiByteAddress-'0')+3*(p->LoByteAddress-'0');
  p->uid_device[0] = (uint8_t) (uid >> 24);
  p->uid_device[1] = (uint8_t) (uid >> 16);
  p->uid_device[2] = (uint8_t) (uid >> 8);
  p->uid_device[3] = (uint8_t) uid;
  return uid;
}

uint16_t generateHash(uint32_t uid){
  uint16_t highbyte = uid >> 16;
  uint16_t lowbyte = uid;
  uint16_t hash = highbyte ^ lowbyte;
  bitWrite(hash, 7, 0);
  bitWrite(hash, 8, 1);
  bitWrite(hash, 9, 1);
  return hash;
}

void sendCanFrame(CAN_Frame frame){
  frame.extended = 1;
  frame.id = frame.cmd;
  frame.id = (frame.id << 17) | frame.hash;
  bitWrite(frame.id, 16, frame.resp_bit);
  CANBase.write(frame);
}

CAN_Frame getCanFrame(){
  CAN_Frame frame = CANBase.read();
  frame.cmd = frame.id >> 17;
  frame.resp_bit = bitRead(frame.id, 16);
  return frame;
}

void goIntoBootloader() {
  CAN.outgoingMsg.data[0] = GO_BTLDR;
  CAN.outgoingMsg.data[1] = CAN.params.HiByteAddress;
  CAN.outgoingMsg.data[2] = CAN.params.LoByteAddress;
  CAN.can_answer(3);
  _delay_ms(2*wait_time);  // Delay added just so we can have time to open up
  detachInterrupt(digitalPinToInterrupt(PIN_INT0));
  _delay_ms(3*wait_time);  // Delay added just so we can have time to open up
  // jumping into the Bootloader
  //need word address
  //so, byte address/2
  goto*0x7000/2;
}

#endif
