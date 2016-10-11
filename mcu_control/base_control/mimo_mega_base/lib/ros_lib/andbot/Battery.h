#ifndef _ROS_andbot_Battery_h
#define _ROS_andbot_Battery_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace andbot
{

  class Battery : public ros::Msg
  {
    public:
      uint8_t capacity;
      uint16_t current;

    Battery():
      capacity(0),
      current(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->capacity >> (8 * 0)) & 0xFF;
      offset += sizeof(this->capacity);
      *(outbuffer + offset + 0) = (this->current >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->capacity =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->capacity);
      this->current =  ((uint16_t) (*(inbuffer + offset)));
      this->current |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->current);
     return offset;
    }

    const char * getType(){ return "andbot/Battery"; };
    const char * getMD5(){ return "f818fff1424893c4b50712b496df5eba"; };

  };

}
#endif