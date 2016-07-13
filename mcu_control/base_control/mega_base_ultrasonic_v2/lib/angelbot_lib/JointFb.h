#ifndef _ROS_andbot_JointFb_h
#define _ROS_andbot_JointFb_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "andbot/Fb.h"

namespace andbot
{

  class JointFb : public ros::Msg
  {
    public:
      andbot::Fb joint0;
      andbot::Fb joint1;
      andbot::Fb joint2;
      andbot::Fb joint3;

    JointFb():
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

    const char * getType(){ return "andbot/JointFb"; };
    const char * getMD5(){ return "ced444daa570aeb4fd281959abac65d8"; };

  };

}
#endif