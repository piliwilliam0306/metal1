
void sendCmd(int offset, int cmd_Motor, byte slave)
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

