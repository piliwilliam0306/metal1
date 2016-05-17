#ifndef _ROS_andbot_WheelCmd_h
#define _ROS_andbot_WheelCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace andbot
{

  class WheelCmd : public ros::Msg
  {
    public:
      float speed1;
      bool mode1;
      float speed2;
      bool mode2;

    WheelCmd():
      speed1(0),
      mode1(0),
      speed2(0),
      mode2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->speed1);
      union {
        bool real;
        uint8_t base;
      } u_mode1;
      u_mode1.real = this->mode1;
      *(outbuffer + offset + 0) = (u_mode1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode1);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed2);
      union {
        bool real;
        uint8_t base;
      } u_mode2;
      u_mode2.real = this->mode2;
      *(outbuffer + offset + 0) = (u_mode2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed1));
      union {
        bool real;
        uint8_t base;
      } u_mode1;
      u_mode1.base = 0;
      u_mode1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode1 = u_mode1.real;
      offset += sizeof(this->mode1);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed2));
      union {
        bool real;
        uint8_t base;
      } u_mode2;
      u_mode2.base = 0;
      u_mode2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode2 = u_mode2.real;
      offset += sizeof(this->mode2);
     return offset;
    }

    const char * getType(){ return "andbot/WheelCmd"; };
    const char * getMD5(){ return "733c147faf50d6f14134c7e87d662d3c"; };

  };

}
#endif