double MaxAngle = 360;
double position_target = 0.0;
double position_actual = 0.0;

void readTest()
{
  if (Serial.available() >= 4)  cmdPos_ROS = Serial.parseFloat();
}
void get_cmd_pos() 
{
  if (Serial.available() >= 7)
  {
     char rS=(char)Serial.read();
     if(rS=='{')
     {
       char commandArray[6];
       Serial.readBytes(commandArray,6);
       byte rA=commandArray[0]; //device address
       byte rSH=commandArray[1];
       byte rSL=commandArray[2];
       byte rPH=commandArray[3];
       byte rPL=commandArray[4];
       char rF=commandArray[5];
       if(rF=='}' && rA==SLAVE) 
       { 
         pos_offset_v = ((rSH<<8)+rSL)*0.01;        
         cmdPos_ROS = ((rPH<<8)+rPL); //degree
       }
     }
  }
}
/*
void readCmd()
{
  int position_receive = 0;
  if (Serial.available() >= 5) 
  {
    char rT = (char)Serial.read(); //read target speed from mega
          if(rT == '{') //start byte
            {
              char commandArray[4];
              Serial.readBytes(commandArray,4);
              byte rA=commandArray[0]; //device address + command
              byte rH=commandArray[1]; //high byte
              byte rL=commandArray[2]; //high byte
              char rP=commandArray[3]; //stop byte
              if(rP=='}' && rA==SLAVE)         
                {                 
                  position_receive = (rH<<8)+rL;
                  position_target = double (position_receive*(MaxAngle/32767));
                  //sendFeedback();
                }
            }
  }         
}
*/
