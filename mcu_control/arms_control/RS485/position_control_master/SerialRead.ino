
int read_slave_L(int slave)
{
  int position_receive = 0;
  double position_actual = 0;
  if (Serial1.available() >= 5) 
  {
    char rT = (char)Serial1.read(); //read target speed from mega
          if(rT == '{') //start byte
          {
              char commandArray[4];
              Serial1.readBytes(commandArray,4);
              byte rA=commandArray[0]; //device address
              byte rH=commandArray[1]; //high byte
              byte rL=commandArray[2]; //low byte
              char rP=commandArray[3]; //stop byte
              if(rP=='}' && rA==slave)         
              {             
                position_receive = (rH<<8)+rL;
                position_actual = double (position_receive*(MaxAngle/32767));
                return position_actual;
              } 
          }
  }
  else  return 0;
}

int read_slave_R(int slave)
{
  int position_receive = 0;
  double position_actual = 0;
  if (Serial2.available() >= 5) 
  {
    char rT = (char)Serial2.read(); //read target speed from mega
          if(rT == '{') //start byte
          {
              char commandArray[4];
              Serial2.readBytes(commandArray,4);
              byte rA=commandArray[0]; //device address
              byte rH=commandArray[1]; //high byte
              byte rL=commandArray[2]; //low byte
              char rP=commandArray[3]; //stop byte
              if(rP=='}' && rA==slave)         
              {             
                position_receive = (rH<<8)+rL;
                position_actual = double (position_receive*(MaxAngle/32767));
                return position_actual;
              } 
          }
  }
  else  return 0;
}
