#ifndef _ROS_SERVICE_DriverState_h
#define _ROS_SERVICE_DriverState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rugby
{

static const char DRIVERSTATE[] = "rugby/DriverState";

  class DriverStateRequest : public ros::Msg
  {
    public:

    DriverStateRequest()
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

    const char * getType(){ return DRIVERSTATE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DriverStateResponse : public ros::Msg
  {
    public:

    DriverStateResponse()
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

    const char * getType(){ return DRIVERSTATE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DriverState {
    public:
    typedef DriverStateRequest Request;
    typedef DriverStateResponse Response;
  };

}
#endif
