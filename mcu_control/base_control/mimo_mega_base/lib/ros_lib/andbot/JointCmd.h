#ifndef _ROS_andbot_JointCmd_h
#define _ROS_andbot_JointCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "andbot/Cmd.h"

namespace andbot
{

  class JointCmd : public ros::Msg
  {
    public:
      andbot::Cmd joint0;
      andbot::Cmd joint1;
      andbot::Cmd joint2;
      andbot::Cmd joint3;

    JointCmd():
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

    const char * getType(){ return "andbot/JointCmd"; };
    const char * getMD5(){ return "3856bd0a0dfcf1992fa06dccefaae7c7"; };

  };

}
#endif