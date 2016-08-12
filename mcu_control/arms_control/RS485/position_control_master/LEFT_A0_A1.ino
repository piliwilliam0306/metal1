

void LEFT_A0_A1()
{
  ARM_RIGHT_A0_A1_EN =0;
  ARM_RIGHT_A2_A3_EN =0;
  ARM_LEFT_A0_A1_EN = 1;
  ARM_LEFT_A2_A3_EN = 0;
  gearRatio_Axis_0=5;
  gearRatio_Axis_1=3;
  static double cmd_Axis_0=0, cmd_Axis_1=0;
  static double cmd_Axis_1_by_0=0, cmd_Axis_1_by_1=0;

  static double cmd_Motor_1_by_0=0,cmd_Motor_1_by_1=0;
  
  static double cmd_Motor_0_increment_by_0=0;
  static double cmd_Motor_1_increment_by_0=0,cmd_Motor_1_increment_by_1=0;
  
  if(abs(cmd_Axis_0-left_angle_Axis_0)>0.01)
  {
    if((left_angle_Axis_0-cmd_Axis_0)>0){
      cmd_Motor_0_increment_by_0=increment_angle_0*gearRatio_Axis_0;
      if(ARM_RIGHT_A0_A1_EN)
        cmd_Motor_1_increment_by_0=increment_angle_0;//*gearRatio_Axis_1*(double)1/gearRatio_Axis_1; 
      else
        cmd_Motor_1_increment_by_0=-increment_angle_0;//*gearRatio_Axis_1*(double)1/gearRatio_Axis_1; 
    }
    else {
      cmd_Motor_0_increment_by_0=-increment_angle_0*gearRatio_Axis_0;
      if(ARM_RIGHT_A0_A1_EN)
        cmd_Motor_1_increment_by_0=-increment_angle_0;//*gearRatio_Axis_1*(double)1/gearRatio_Axis_1; 
      else
        cmd_Motor_1_increment_by_0=increment_angle_0;//*gearRatio_Axis_1*(double)1/gearRatio_Axis_1; 
    }
  }
  else{
    cmd_Motor_0_increment_by_0=0;
    cmd_Motor_1_increment_by_0=0;
    
    }

  if(abs(cmd_Axis_1_by_1-left_angle_Axis_1)>0.01)
  {
    if((left_angle_Axis_1-cmd_Axis_1_by_1)>0){
      cmd_Motor_1_increment_by_1=increment_angle_1*gearRatio_Axis_1; 
    }
    else {
      cmd_Motor_1_increment_by_1=-increment_angle_1*gearRatio_Axis_1; 
    }
  }
  else{
    cmd_Motor_1_increment_by_1=0;
    
    }  

    //cmd_Motor_0
    cmd_Motor_L0= cmd_Motor_L0 + cmd_Motor_0_increment_by_0;
    //cmd_Axis_0
    cmd_Axis_0=(double)cmd_Motor_L0/gearRatio_Axis_0;
    
    //cmd_Motor_1
    if(ARM_RIGHT_A0_A1_EN||ARM_LEFT_A0_A1_EN)
      cmd_Motor_1_by_0=cmd_Motor_1_by_0- cmd_Motor_1_increment_by_0;    
    else if(ARM_RIGHT_A2_A3_EN||ARM_LEFT_A2_A3_EN)
      cmd_Motor_1_by_0=cmd_Motor_1_by_0+ cmd_Motor_1_increment_by_0;    
    cmd_Motor_1_by_1=cmd_Motor_1_by_1+ cmd_Motor_1_increment_by_1;  
    cmd_Motor_L1=cmd_Motor_1_by_0+cmd_Motor_1_by_1;
    //cmd_Axis_1
    cmd_Axis_1_by_0=(double)cmd_Motor_1_by_0/ gearRatio_Axis_1;
    cmd_Axis_1_by_1=(double)cmd_Motor_1_by_1/ gearRatio_Axis_1;
    cmd_Axis_1=cmd_Axis_1_by_0+cmd_Axis_1_by_1;    

    //cmd_serial_to_uno();
    //show();
/*
    Serial.print(" ");Serial.print(left_angle_Axis_0);
    Serial.print(" ");Serial.print(cmd_Axis_0);
    Serial.print(" ");Serial.print(cmd_Motor_L0);
    Serial.print(" ");Serial.print(cmd_Motor_0_increment_by_0);

    Serial.print("       ");
    Serial.print(" ");Serial.print(left_angle_Axis_1);
    Serial.print(" ");Serial.print(cmd_Axis_1);    
    Serial.print(" ");Serial.print(cmd_Axis_1_by_0);
    Serial.print(" ");Serial.print(cmd_Axis_1_by_1);
    Serial.print(" ");Serial.print(cmd_Motor_L1);

    Serial.println(" "); 
*/    
  }
