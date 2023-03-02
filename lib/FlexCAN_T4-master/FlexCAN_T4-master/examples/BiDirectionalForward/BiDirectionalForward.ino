#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

void setup(void) {
  can1.begin();
  can1.setBaudRate(250000);
  can2.begin();
  can2.setBaudRate(250000);

  msg.buf[0] = 10;
  msg.buf[1] = 20;
  msg.buf[2] = 0;
  msg.buf[3] = 100;
  msg.buf[4] = 128;
  msg.buf[5] = 64;
  msg.buf[6] = 32;
  msg.buf[7] = 16;
}

void loop() {
  if ( can1.read(msg) ) {
    can2.write(msg);
  }
  else if ( can2.read(msg) ) {
    can1.write(msg);
  }
}
