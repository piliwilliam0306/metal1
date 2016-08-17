void go_joint_pos_ros(double target_pos_ros) 
{
  double target_pos_joint=target_pos_ros+pos_offset;
  cmdPwm = Kp * (target_pos_joint-anglePos_Joint);
  if(AXIS_TYPE == CCW)  send_cmd_to_motor(-cmdPwm);
  else                  send_cmd_to_motor(cmdPwm);
}

void send_cmd_to_motor(int cmdpwm) 
{
    int vPlus=0,vMinus=0;
    if(cmdpwm>=0)
    { vPlus=cmdpwm; vMinus=0; }
    else if(cmdpwm<0)
    { vPlus=0;  vMinus=-cmdpwm; }   
    
    if(vPlus>=MAX_PWM)       vPlus=MAX_PWM;
    else if(vPlus<-MAX_PWM)  vPlus=-MAX_PWM;
           
    if(vMinus>=MAX_PWM)      vMinus=MAX_PWM;
    else if(vMinus<-MAX_PWM) vMinus=-MAX_PWM;      

    analogWrite(MotorPin0,vMinus);  analogWrite(MotorPin1,vPlus);
}
