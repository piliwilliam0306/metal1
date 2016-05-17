#ifndef _ROS_andbot_WheelFb_h
#define _ROS_andbot_WheelFb_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace andbot
{

  class WheelFb : public ros::Msg
  {
    public:
      float speed1;
      uint16_t current1;
      bool status1;
      float speed2;
      uint16_t current2;
      bool status2;

    WheelFb():
      speed1(0),
      current1(0),
      status1(0),
      speed2(0),
      current2(0),
      status2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->speed1);
      *(outbuffer + offset + 0) = (this->current1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current1);
      union {
        bool real;
        uint8_t base;
      } u_status1;
      u_status1.real = this->status1;
      *(outbuffer + offset + 0) = (u_status1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status1);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed2);
      *(outbuffer + offset + 0) = (this->current2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current2);
      union {
        bool real;
        uint8_t base;
      } u_status2;
      u_status2.real = this->status2;
      *(outbuffer + offset + 0) = (u_status2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed1));
      this->current1 =  ((uint16_t) (*(inbuffer + offset)));
      this->current1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->current1);
      union {
        bool real;
        uint8_t base;
      } u_status1;
      u_status1.base = 0;
      u_status1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status1 = u_status1.real;
      offset += sizeof(this->status1);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed2));
      this->current2 =  ((uint16_t) (*(inbuffer + offset)));
      this->current2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->current2);
      union {
        bool real;
        uint8_t base;
      } u_status2;
      u_status2.base = 0;
      u_status2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status2 = u_status2.real;
      offset += sizeof(this->status2);
     return offset;
    }

    const char * getType(){ return "andbot/WheelFb"; };
    const char * getMD5(){ return "99bd53c289e8d46b16307dd1ac681bed"; };

  };

}
#endif