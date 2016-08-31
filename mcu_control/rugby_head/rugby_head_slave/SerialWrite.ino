
void sendFeedback()
{
  digitalWrite(RS485Mode, RS485Transmit); //DE,RE=HIGH, TX enabled
  int actual_send = int(position_actual/(MaxAngle/32767));
  byte buf[5];
  buf[0] = '{'; //send start byte
  buf[1] = SLAVE;
  buf[2] = highByte(actual_send);  buf[3] = lowByte(actual_send);
  buf[4] = '}'; //send stop byte 
  Serial.write(buf, sizeof(buf));
  delayMicroseconds(10);
  digitalWrite(RS485Mode, RS485Receive); //DE,RE=LOW, RX enabled
}

