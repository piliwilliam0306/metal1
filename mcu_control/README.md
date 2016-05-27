# mcu_control

## Modify Arduino PWM Frequency 
     Add this line in setup()
          TCCR0B = TCCR0B & B11111000 | B00000010; // set timer 0 divisor to 8 for PWM frequency of 7812.50 Hz
     Overwrite wiring.c in: 
          ~/arduino-1.6.5/hardware/arduino/avr/cores/arduino
     Edit line 31 in "wiring.c"     
     Mega Board:
          #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
     Vnh5019 Board:
          #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))


## MCU firmware
     "mega_base_ultrasonic_v1" is for base mega board.
     "vnh5019_andbot_test.ino" is to test if motor can achieve desire speed.
     "vnh5019.ino" is for the motor controller board for base. (Upload this if vnh5019 control board pass speed test.
     Please select Arduino Pro Mini when uploading codes for the motor controller boards.
     Copy "ros_lib" folder into "Arduino/libraries" folder.
