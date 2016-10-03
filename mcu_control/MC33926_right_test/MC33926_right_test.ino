//max speed 31 rad/s
#define encoder0PinA  2
#define encoder0PinB  3

#define In1 6
#define In2 5
#define MC33926Enable 7

#define LOOPTIME 40 

#define MaxPWM 255

static double MaxSpeed = 19;

int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;

volatile long Encoderpos = 0;
volatile long unknownvalue = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;
long banana = 0;
double omega_target = 0.0;
double omega_actual = 0;

int PWM_val = 0;                                
int CPR = 64;                                   // encoder count per revolution
int gear_ratio = 30; 

float Kp = 0.85;
float Ki = 0.005;
float Kd = 0;

double error;
double pidTerm = 0;                                                            // PID correction
double sum_error, d_error=0;

double calculated_pidTerm;
double constrained_pidterm;

//current sense
int analogPin = A0;
int current = 0;
int current_send = 0;

void setup() { 
 //Set PWM frequency for D5 & D6
 // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz  
 //TCCR0B = TCCR0B & B11111000 | B00000010; 
 pinMode(encoder0PinA, INPUT); 
 digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
 pinMode(encoder0PinB, INPUT); 
 digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

 attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
 attachInterrupt(1, doEncoder, CHANGE);
 pinMode(In1, OUTPUT); 
 pinMode(In2, OUTPUT); 
 pinMode(MC33926Enable, OUTPUT);
 Serial.begin (57600);
 digitalWrite(MC33926Enable, HIGH);
} 

void loop() 
{       
   if (Serial.available() >= 4) {
    omega_target = Serial.parseFloat();
   }

  if((millis()-lastMilli) >= LOOPTIME)   
     {                                  
        dT = millis()-lastMilli;
        lastMilli = millis();
        
        getMotorData();                                                           // calculate speed

        PWM_val = (updatePid(omega_target, omega_actual));                       // compute PWM value from rad/s 

        //current = analogRead(analogPin) * 34;

        //if (current > 3500)  digitalWrite(EN, LOW); 
        
        //if (omega_target == 0 ) {PWM_val = 0; digitalWrite(MC33926Enable, HIGH);}
        
        if (omega_target == 0 ) { PWM_val = 0; sum_error =0;  }
        if (PWM_val == 0)  { analogWrite(In1,0); analogWrite(In2,0);  }
        if (PWM_val > 0)   { analogWrite(In1,abs(PWM_val));    analogWrite(In2,0);     }
        if (PWM_val < 0)    { analogWrite(In2,abs(PWM_val));    analogWrite(In1,0);     }
        printMotorInfo();
     }
}

void getMotorData()  
{                               
  static long EncoderposPre = 0;       
  //converting ticks/s to rad/s
  omega_actual = -((Encoderpos - EncoderposPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  //ticks/s to rad/s
  EncoderposPre = Encoderpos;                 
}

double updatePid(double targetValue,double currentValue)   
{            
  
  static double last_error=0;                            
  error = targetValue - currentValue; 

  sum_error = sum_error + error * dT;

  sum_error = constrain(sum_error, -2500, 2500);  //good constrain
  
  d_error = (error - last_error) / dT;
  pidTerm = Kp * error + Ki * sum_error + Kd * d_error;   
                       
  last_error = error;  

  calculated_pidTerm = pidTerm/(MaxSpeed/MaxPWM); 
  constrained_pidterm = constrain(calculated_pidTerm, -255, 255);
  
  return constrained_pidterm;
}

void doEncoder() {
//   Encoderpos++;
  pinAState = digitalRead(2);
  pinBState = digitalRead(3);

  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      Encoderpos --;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      Encoderpos ++;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      Encoderpos --;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      Encoderpos ++;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      Encoderpos --;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      Encoderpos ++;
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      Encoderpos --;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      Encoderpos ++;
  }
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
}

void printMotorInfo()  
{                                                                      
   Serial.print(" target:");                  Serial.print(omega_target);
   Serial.print(" actual:");                  Serial.print(omega_actual);
   Serial.print("  dT:");                  Serial.print(dT);
   Serial.print("  sum_err:");                  Serial.print(sum_error);
   Serial.print("  Count:");                  Serial.print(banana);
   Serial.print("  PWM_val:");                  Serial.print(PWM_val);
   Serial.print("  Encoder: ");             Serial.print(Encoderpos);

   Serial.println();
}

