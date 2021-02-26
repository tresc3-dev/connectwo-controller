/*
 * SwerveDrive.h
 *
 *  Created on: 2019. 5. 28.
 *      Author: colson
 *      email: dud3722000@naver.com
 *      email: colson@korea.ac.kr
 *      name: YeongJunKim
 */

#ifndef _ROS_aucar_msgs_SwerveDrive_h
#define _ROS_aucar_msgs_SwerveDrive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

namespace aucar_msgs
{
  class SwerveDrive : public ros::Msg
  {
  public:
    typedef geometry_msgs::Vector3 _linear_type;
    _linear_type linear[4];
    typedef geometry_msgs::Vector3 _angular_type;
    _angular_type angular[4];

    SwerveDrive () :
	linear (), angular ()
    {

    }

    virtual int
    serialize (unsigned char *outbuffer) const
    {
      int offset = 0;
      for (int i = 0; i < 4; i++)
	offset += this->linear[i].serialize (outbuffer + offset);
      for (int i = 0; i < 4; i++)
	offset += this->angular[i].serialize (outbuffer + offset);
      return offset;

    }
    virtual int
    deserialize (unsigned char *inbuffer)
    {

      int offset = 0;
      for (int i = 0; i < 4; i++)
	offset += this->linear[i].deserialize (inbuffer + offset);
      for (int i = 0; i < 4; i++)
	offset += this->angular[i].deserialize (inbuffer + offset);
      return offset;
    }

    const char *
    getType ()
    {
      return "aucar_msgs/SwerveDrive";
    }
    ;
    const char *
    getMD5 ()
    {
      return "abababababdedededede121212121233";
    }
    ;
  };

}

#endif /* AUCAR_MSGS_SWERVEDRIVE_H_ */
