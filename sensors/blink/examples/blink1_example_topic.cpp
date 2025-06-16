/*
* Copyright (c) 2016 Carnegie Mellon University, Guilherme Pereira <gpereira@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include "ros/ros.h"
#include "blink1/Blink1msg.h"
#include "blink1/blinkfn.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_blink");
 
  ros::NodeHandle n;

  ros::Publisher blink_pub = n.advertise<blink1::Blink1msg>("blink1/blink", 1);

  blink1::Blink1msg msg;
	
   
  while (ros::ok())
  {
       ros::spinOnce();

       msg.function = BL_FADE; // fade led to red in 500 miliseconds
       msg.t=500;
       msg.r=255;
       msg.g=0;
       msg.b=0;
       blink_pub.publish(msg);  	
           
       ros::Duration(5).sleep();

     
       msg.function = BL_FADE; // fade green to red in 300 miliseconds
       msg.t=300;
       msg.r=0;
       msg.g=255;
       msg.b=0;
       blink_pub.publish(msg);     
       ros::Duration(5).sleep();

               
       msg.function = BL_ON; // set led to blue imediatelly
       msg.t=0;
       msg.r=0;
       msg.g=0;
       msg.b=255;
       blink_pub.publish(msg);
       ros::Duration(5).sleep();
       
       
       msg.function = BL_RANDBLINK; // blink with randon colors with period 400 miliseconds
       msg.t=400;
       blink_pub.publish(msg);
       ros::Duration(10).sleep();
              
       
       msg.function = BL_BLINK; // blink red colors at frequency 1/800 miliseconds
       msg.t=800;
       msg.r=255;
       msg.g=0;
       msg.b=0;
       blink_pub.publish(msg);
       ros::Duration(5).sleep();
       
       
       msg.function = BL_FADE; // fade to white in 500 miliseconds
       msg.t=500;
       msg.r=255;
       msg.g=255;
       msg.b=255;
       blink_pub.publish(msg);     
       ros::Duration(10).sleep();
       
  }
  

  return 0;
}
