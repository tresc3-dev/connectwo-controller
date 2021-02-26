/*
 * AuCAR.h
 *
 *  Created on: May 28, 2019
 *      Author: colson
 *      email: dud3722000@naver.com
 *      email: colson@korea.ac.kr
 *      name: YeongJunKim
 */

#ifndef _ROS_aucar_msgs_AuCAR_h
#define _ROS_aucar_msgs_AuCAR_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "ros/time.h"

namespace aucar_msgs
{
  class AuCAR : public ros::Msg
  {
  public:
    typedef std_msgs::Header _header_type;
    _header_type header;
    typedef std_msgs::String _string_type;
    _string_type string;

    AuCAR () :
	header (), string ()
    {

    }

    virtual int
    serialize (unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize (outbuffer + offset);
      offset += this->string.serialize (outbuffer + offset);
      return offset;
    }
    virtual int
    deserialize (unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize (inbuffer + offset);
      offset += this->string.deserialize (inbuffer + offset);
      return offset;
    }

    const char *
    getType ()
    {
      return "aucar_msgs/AuCAR";
    }
    ;
    const char *
    getMD5 ()
    {
      return "11111111110000000000111111111122";
    }
    ;
  };
}

#endif /* AUCAR_H_ */
