
#include <IRremote.h>

  IRrecv irrecv2(2); 
  IRrecv irrecv3(3);

decode_results results;
  
void setup() {
  Serial.begin(57600);
  irrecv2.enableIRIn(); 
  irrecv3.enableIRIn();
}

void loop() {

  if (irrecv2.decode(&results)) {
    Serial.print("1 ");
    Serial.println(results.value, HEX);
    irrecv2.resume(); 
  }
  
  if (irrecv3.decode(&results)) {
    Serial.print("2 ");
    Serial.println(results.value, HEX);
    irrecv3.resume(); 
  }

}
