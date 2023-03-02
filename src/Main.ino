

#include "SPI.h"
#include "SPI_MSTransfer_MASTER.h"
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
//FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
CAN_message_t msg;
SPI_MSTransfer_MASTER<&SPI, 10, 0x1235> mySPI1235;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(0, OUTPUT);
  pinMode(33, OUTPUT);
  digitalWrite(0, HIGH);
  digitalWrite(33, LOW);
  mySPI1235.begin();

  Can0.begin();
  Can0.setBaudRate(500000);
  //Can0.setMaxMB(16);

  //Can0.enableFIFO();
  // Can0.setFIFOFilter(REJECT_ALL);
  // Can0.setFIFOFilter(0, M192_Command_Message_CANID, STD);
  // Can0.setFIFOFilter(1, MSGID_0X6B2_CANID, STD);
  //Can0.enableFIFOInterrupt();
    
  //Can0.onReceive(CanSniff);
  msg.id = 0x1839F380;
  msg.len = 8;
  msg.flags.extended = 1;
  msg.flags.remote   = 0;
  msg.flags.overrun  = 0;
  msg.flags.reserved = 0;
  Can0.mailboxStatus(); 
  
}

int8_t lookup[58] = {120, 110, 100, 93, 85, 80, 77, 73, 68, 65, 62, 59, 56, 54, 
        51, 49, 46, 44, 42, 40, 38, 37, 35, 33, 32, 30, 28, 27, 25, 
        23, 22, 20, 19, 17, 16, 14, 13, 11, 9, 7, 6, 4, 2, 1, -1, 
        -2, -4, -6, -9, -11, -13, -15, -18, -22, -25, -30, -35, -40};



void SendMessage(unsigned id, uint8_t value) {

    CAN_message_t msg;
    
    msg.id = id;
    msg.flags.extended = 0;
    msg.len = 1;
    
    msg.buf[0] = value;

    Can0.write(msg);

}

void SendMessage(unsigned id, uint8_t* value, int len) {

    CAN_message_t msg;

    if (value == nullptr)
    {
        return;
    }
    
    msg.id = id;
    msg.flags.extended = 0;
    msg.len = 1;
    
    for (int i=0; i<len; i++)
    {
      msg.buf[i] = value[i];
    }

    Can0.write(msg);
}


#define DEBUG_ADDR 0x69
uint8_t debug_buffer[8] = {0};


void loop() {
  int min = 0;
  int max = 0;
  //Min and Max ID might need to be uint8_t or 16
  uint8_t MinChanID = 0;
  uint8_t MaxChanID = 0;
  int sum = 0;
  uint8_t average = 0;
  uint16_t rxbuff = 0x0;
  uint8_t temp = 0;
  for (int i = 0; i < 7; i++){
    uint16_t txbuff = 0;
    rxbuff = mySPI1235.transfer16(&txbuff, 1, 1);
    rxbuff = (rxbuff >> 4) & 0xff;
    // Serial.println(i);
    // Serial.println(rxbuff);
    //SendMessage(DEBUG_ADDR+1, i);
    //SendMessage(DEBUG_ADDR+4, rxbuff);
    //For 26 deg C the batteries voltage output was 0.64 Volts


    delay(500);
    uint8_t temp = lookup[rxbuff - 65];
    //uint8_t temp = 50;
    if (i == 0){
      min = temp;
      max = temp;
      MinChanID = i >> 11;
      MaxChanID = i >> 11;
    }
    if (temp < min){
      min = temp;
      MinChanID = i >> 11;
    }
    if (temp > max){
      max = temp;
      MaxChanID = i >> 11;
    }
    sum = sum + temp;
    average = sum/7;
    uint8_t checksum = (0x01 + 0x01 + rxbuff + rxbuff + rxbuff);//min + max + average);
    msg.buf[0] = 0x00;//  rxbuff(0x00)  Thermistor module number
    msg.buf[1] = rxbuff; //min;
    msg.buf[2] = rxbuff; //max;
    msg.buf[3] = rxbuff; //average;
    msg.buf[4] = 0x07;//Number of thermistors enabled   (7)
    msg.buf[5] = 0x01;//Highest thermistor ID on the module
    msg.buf[6] = 0x00;//Lowest thermistor ID on the module
    msg.buf[7] = (checksum + 0x39 + 0x08);

    Can0.write(msg);
    // //Serial.println("Can sent");
    
    Can0.events();


  }

  /* average = sum/7;
  uint8_t checksum = (0x01 + 0x01 + rxbuff + rxbuff + rxbuff);//min + max + average);
  msg.buf[0] = 0x00;//  rxbuff(0x00)  Thermistor module number
  msg.buf[1] = rxbuff; //min;
  msg.buf[2] = rxbuff; //max;
  msg.buf[3] = rxbuff; //average;
  msg.buf[4] = 0x07;//Number of thermistors enabled   (7)
  msg.buf[5] = 0x01;//Highest thermistor ID on the module
  msg.buf[6] = 0x00;//Lowest thermistor ID on the module
  msg.buf[7] = (checksum + 0x39 + 0x08);

  Can0.write(msg);
  // //Serial.println("Can sent");
  
  Can0.events();
   */
  
}

