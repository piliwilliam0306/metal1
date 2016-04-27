#include <SharpIR.h>

#define model 1080
// ir: the pin where your sensor is attached
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)

SharpIR SharpIR0(A0, model);
SharpIR SharpIR1(A1, model);
SharpIR SharpIR2(A2, model);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
}

void loop() {
  delay(200);   

  //unsigned long pepe1=millis();  // takes the time before the loop on the library begins

  int dis0=SharpIR0.distance();  // this returns the distance to the object you're measuring
  int dis1=SharpIR1.distance();  // this returns the distance to the object you're measuring
  int dis2=SharpIR2.distance();  // this returns the distance to the object you're measuring
  //Serial.print("Mean distance: ");  // returns it to the serial monitor
  Serial.print(dis0);
  Serial.print("  ");
  Serial.print(dis1);
  Serial.print(" ");
  Serial.println(dis2);    
  //unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement
  //Serial.print("Time taken (ms): ");
  //Serial.println(pepe2);  

}
