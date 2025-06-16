/*
* Copyright (c) 2016 Carnegie Mellon University, Guilherme Pereira <gpereira@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include "ros/ros.h"
#include "blink1/Blink.h"
#include "blink1/blinkfn.h"

#ifndef _BLINK1_CLASS

#define _BLINK1_CLASS

class Blink1
{
private:
  ros::ServiceClient client_;
  blink1::Blink srv_;
public:
  Blink1(ros::NodeHandle& n){
       client_ = n.serviceClient<blink1::Blink>("blink1/blink");    
  }
  
  bool fade(uint8_t r, uint8_t g, uint8_t b, int t){
    srv_.request.function = BL_FADE;
    srv_.request.t=t;
    srv_.request.r=r;
    srv_.request.g=g;
    srv_.request.b=b;
    if (client_.call(srv_))
    {
      return srv_.response.on; 
    }
    else
    {
      ROS_ERROR("Failed to call Blink(1) service!");
      return false;
    }  
  }
  
  bool set(uint8_t r, uint8_t g, uint8_t b){
    srv_.request.function = BL_ON;
    srv_.request.t=0;
    srv_.request.r=r;
    srv_.request.g=g;
    srv_.request.b=b;
    if (client_.call(srv_))
    {
      return srv_.response.on; 
    }
    else
    {
      ROS_ERROR("Failed to call Blink(1) service!");
      return false;
    }  
  }
  
  bool blink(uint8_t r, uint8_t g, uint8_t b, int t){
    srv_.request.function = BL_BLINK;
    srv_.request.t=t;
    srv_.request.r=r;
    srv_.request.g=g;
    srv_.request.b=b;
    if (client_.call(srv_))
    {
      return srv_.response.on; 
    }
    else
    {
      ROS_ERROR("Failed to call Blink(1) service!");
      return false;
    }  
  }
  
  bool blink(int t){
    srv_.request.function = BL_RANDBLINK;
    srv_.request.t=t;
    srv_.request.r=0;
    srv_.request.g=0;
    srv_.request.b=0;
    if (client_.call(srv_))
    {
      return srv_.response.on; 
    }
    else
    {
      ROS_ERROR("Failed to call Blink(1) service!");
      return false;
    }  
  }
  
};

#endif