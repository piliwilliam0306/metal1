#ifndef _ROS_andbot_JointOffset_h
#define _ROS_andbot_JointOffset_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "andbot/Offset.h"

namespace andbot
{

  class JointOffset : public ros::Msg
  {
    public:
      andbot::Offset joint0;
      andbot::Offset joint1;
      andbot::Offset joint2;
      andbot::Offset joint3;

    JointOffset():
      joint0(),
      joint1(),
      joint2(),
      joint3()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->joint0.serialize(outbuffer + offset);
      offset += this->joint1.serialize(outbuffer + offset);
      offset += this->joint2.serialize(outbuffer + offset);
      offset += this->joint3.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->joint0.deserialize(inbuffer + offset);
      offset += this->joint1.deserialize(inbuffer + offset);
      offset += this->joint2.deserialize(inbuffer + offset);
      offset += this->joint3.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "andbot/JointOffset"; };
    const char * getMD5(){ return "ce981bbff8f82311c298b84fabb89ca4"; };

  };

}
#endif