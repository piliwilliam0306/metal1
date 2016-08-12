
void sendCmd_L(int offset, int cmd_Motor, byte slave)
{
  //digitalWrite(RS485Mode1, RS485Transmit); //DE,RE=HIGH, TX enabled
  byte buf[7];
  buf[0] = '{';
  buf[1] = slave;
  buf[2] = highByte(offset); buf[3] = lowByte(offset);
  buf[4] = highByte(cmd_Motor); buf[5] = lowByte(cmd_Motor);
  buf[6] ='}';
  Serial1.write(buf, sizeof(buf));
  //delayMicroseconds(30);
  //digitalWrite(RS485Mode1, RS485Receive); //DE,RE=LOW, RX enabled
  //delayMicroseconds(100);
}

void sendCmd_R(int offset, int cmd_Motor, byte slave)
{
  //digitalWrite(RS485Mode2, RS485Transmit); //DE,RE=HIGH, TX enabled
  byte buf[7];
  buf[0] = '{';
  buf[1] = slave;
  buf[2] = highByte(offset); buf[3] = lowByte(offset);
  buf[4] = highByte(cmd_Motor); buf[5] = lowByte(cmd_Motor);
  buf[6] ='}';
  Serial2.write(buf, sizeof(buf));
  //delayMicroseconds(30);
  //digitalWrite(RS485Mode2, RS485Receive); //DE,RE=LOW, RX enabled
  //delayMicroseconds(100);
}

void CalCmd_L(byte slave)
{
  byte buf[8];
  buf[0] = '|'; buf[1] = slave; buf[2] = ID; //update ID
  buf[3] = Kp; buf[4] = Ki; buf[5] = Kd; //send PID
  buf[6] = Current_Limit; //current limit
  buf[7] ='|';
  Serial1.write(buf, sizeof(buf));
}

void CalCmd_R(byte slave)
{
  byte buf[8];
  buf[0] = '|'; buf[1] = slave; buf[2] = ID; //update ID
  buf[3] = Kp; buf[4] = Ki; buf[5] = Kd; //send PID
  buf[6] = Current_Limit; //current limit
  buf[7] ='|';
  Serial2.write(buf, sizeof(buf));
}
