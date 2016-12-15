#ifndef _ROS_SERVICE_DeviceCheck_h
#define _ROS_SERVICE_DeviceCheck_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rugby
{

static const char DEVICECHECK[] = "rugby/DeviceCheck";

  class DeviceCheckRequest : public ros::Msg
  {
    public:

    DeviceCheckRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return DEVICECHECK; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DeviceCheckResponse : public ros::Msg
  {
    public:

    DeviceCheckResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return DEVICECHECK; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DeviceCheck {
    public:
    typedef DeviceCheckRequest Request;
    typedef DeviceCheckResponse Response;
  };

}
#endif
