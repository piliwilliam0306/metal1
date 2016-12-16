#define RIGHT_WHEEL 1
#define LEFT_WHEEL 2

#define WHEEL_TYPE LEFT_WHEEL

#define RS485Transmit    HIGH
#define RS485Receive     LOW

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

//char commandArray[3];
byte sT = 0;  //send start byte
byte sH = 0;  //send high byte
byte sL = 0;  //send low byte
byte sP = 0;  //send stop byte
byte sG = 0;

byte rT = 0;  //receive start byte
byte rH = 0;  //receive high byte
byte rL = 0;  //receive low byte
byte rP = 0;  //receive stop byte
byte rG = 0;


volatile long Encoderpos = 0;
volatile long unknownvalue = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;                    // loop timing
long dT = 0;
unsigned long cc = 0;

double omega_target = 0.0;
//double omega_target = 2.0;
double omega_actual = 0;

int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int CPR = 64;                                   // encoder count per revolution
int gear_ratio = 30;
int actual_send = 0;
int target_receive = 0;

int analogPin = A0;
unsigned int current = 0;

float Kp = 1.0;
float Ki = 0.007;
float Kd = 0;

double error;
double pidTerm = 0;                                                            // PID correction
double sum_error, d_error = 0;

double calculated_pidTerm;
double constrained_pidterm;

void setup() {
  //TCCR0B = TCCR0B & B11111000 | B00000010;
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder, CHANGE);

  //eable RS485
  pinMode(8, OUTPUT);
  digitalWrite(8, RS485Receive);

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(MC33926Enable, OUTPUT);
  digitalWrite(MC33926Enable, HIGH);
  Serial.begin (1000000);
}

void loop()
{
  Serial.write(1);
  CurrentMonitor();
  readCmd_wheel_angularVel();

  if ((millis() - lastMilli) >= LOOPTIME)
  {
    Serial.write(4);
    // enter tmed loop
    dT = millis() - lastMilli;
    lastMilli = millis();

    getMotorData();// calculate speed


    PWM_val = (updatePid(omega_target, omega_actual));                     // compute PWM value from rad/s
    if (omega_target == 0) {
      PWM_val = 0;
      sum_error = 0;
    }
    if (PWM_val == 0)  {
      analogWrite(In1, 0);
      analogWrite(In2, 0);
    }
    if (PWM_val < 0)   {
      analogWrite(In1, abs(PWM_val));
      analogWrite(In2, 0);
    }
    if (PWM_val > 0)    {
      analogWrite(In2, abs(PWM_val));
      analogWrite(In1, 0);
    }

    //printMotorInfo();
    //cc = cc +1;
  }
}

void readCmd_wheel_angularVel()
{
  Serial.write(2);
  digitalWrite(8, RS485Receive);
  if (Serial.available() > 6)
  {
    char rT = (char)Serial.read(); //read target speed from mega
    if (rT == '{')
    {
      byte rA = (byte)Serial.read();
      if (rA == WHEEL_TYPE) {
        char commandArray[4];
        Serial.readBytes(commandArray, 4);
        //byte rA = commandArray[0];
        byte rH = commandArray[0];
        byte rL = commandArray[1];
        char rP = commandArray[2];
        byte rG = commandArray[3];
        if (rP == '}')
        {
          target_receive = (rH << 8) + rL;
          omega_target = double (target_receive * (double(MaxSpeed) / 32767)); //convert received 16 bit integer to actual speed
          sendFeedback_wheel_angularVel(); //send actually speed to mega
        }
      } else {
        char garbage[4];
        Serial.readBytes(garbage, 4);
      }
    }
  }
}

void sendFeedback_wheel_angularVel()
{
  Serial.write(3);
  digitalWrite(8, RS485Transmit);

  //getMotorData();
  byte current_send;

  //convert rad/s to 16 bit integer to send
  int actual_send = int(omega_actual / (double(MaxSpeed) / 32767));

  //max current is 20400mA 20400/255 = 80
  current_send = current / 80;

  byte buf[7];
  buf[0] = '{'; //send start byte
  buf[1] = WHEEL_TYPE;
  buf[2] = highByte(actual_send);
  buf[3] = lowByte(actual_send);  //send low byte
  buf[4] = current_send;  //send current value
  buf[5] = '}'; //send stop byte
  buf[6] = 8;
  Serial.write(buf, sizeof(buf));
  delayMicroseconds(15);
  digitalWrite(8, RS485Receive);
}

void getMotorData()
{
  static long EncoderposPre = 0;
  //converting ticks/s to rad/s
  //omega_actual = 4.5;
  if (WHEEL_TYPE == LEFT_WHEEL)
    omega_actual = ((Encoderpos - EncoderposPre) * (1000 / dT)) * 2 * PI / (CPR * gear_ratio) ; //ticks/s to rad/s
  else
    omega_actual = -((Encoderpos - EncoderposPre) * (1000 / dT)) * 2 * PI / (CPR * gear_ratio) ; //ticks/s to rad/s

  EncoderposPre = Encoderpos;
}

double updatePid(double targetValue, double currentValue)
{

  static double last_error = 0;
  error = targetValue - currentValue;

  sum_error = sum_error + error * dT;

  sum_error = constrain(sum_error, -2500, 2500);

  d_error = (error - last_error) / dT;
  pidTerm = Kp * error + Ki * sum_error + Kd * d_error;

  last_error = error;
  if (WHEEL_TYPE == LEFT_WHEEL)
    calculated_pidTerm = -pidTerm / (MaxSpeed / MaxPWM);
  else
    calculated_pidTerm = pidTerm / (MaxSpeed / MaxPWM);

  constrained_pidterm = constrain(calculated_pidTerm, -255, 255);

  return constrained_pidterm;
}


void CurrentMonitor()
{
  current = analogRead(analogPin) * 34;  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
}


void doEncoder() {
  pinAState = digitalRead(2);
  pinBState = digitalRead(3);

  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      Encoderpos --;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      Encoderpos --;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      Encoderpos --;
  }
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
}

void printMotorInfo()
{
  Serial.print(" target:");                  Serial.print(omega_target);
  Serial.print(" actual:");                  Serial.print(omega_actual);
  Serial.print(" sum_error:");              Serial.print(sum_error);
  Serial.print("  error:");                  Serial.print(error);
  Serial.print("  samples:");                  Serial.print(cc);
  Serial.println();
}

