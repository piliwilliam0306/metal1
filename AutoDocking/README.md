# Eric-IR-Lib
- This libarary is modfied from [Arduino-IRremote](http://z3t0.github.io/Arduino-IRremote/) to support multiple IR senders and receivers.

- Currently multiple IR senders is for [Arduino Uno](https://www.arduino.cc/en/Main/ArduinoBoardUno) only. 

# Example

- *Multiple IR emitters*
```
IRsend irsend; //Declaration
irsend.send[protocol](byte Data ,int dataLength , int Pin); //Pin can only be 9 or 10
delay([depend on the protocol]); //Make sure receiver gets clear header sginal everytime 
```

- *Multiple IR receivers*
```
//Declaration 
IRrecv irrecv2([pin number]); 
IRrecv irrecv3([pin number]);

//enable receivers
irrecv2.enableIRIn(); 
irrecv3.enableIRIn();

irrecv2.decode(&[result]);
irrecv2.resume;
```
