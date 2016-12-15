#ifndef _ROS_SERVICE_Calibrate_h
#define _ROS_SERVICE_Calibrate_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rugby
{

static const char CALIBRATE[] = "rugby/Calibrate";

  class CalibrateRequest : public ros::Msg
  {
    public:

    CalibrateRequest()
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

    const char * getType(){ return CALIBRATE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CalibrateResponse : public ros::Msg
  {
    public:

    CalibrateResponse()
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

    const char * getType(){ return CALIBRATE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Calibrate {
    public:
    typedef CalibrateRequest Request;
    typedef CalibrateResponse Response;
  };

}
#endif
