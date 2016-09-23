#include <IRremote.h>

IRsend irsend;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  irsend.sendRC5(0x40,8,10);
  irsend.sendRC5(0x80,8,9);
  delay(1);

}
