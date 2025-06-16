/*
* Copyright (c) 2016 Carnegie Mellon University, Guilherme Pereira <gpereira@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include "blink1/Blink1.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_blink");
 
  ros::NodeHandle n;
  
  Blink1* led;
  led=new Blink1(n);
  
  while (ros::ok())
  {
       ros::spinOnce();
       
       led->fade(255, 0, 0, 500);     // fade led to red in 500 miliseconds
       ros::Duration(5).sleep();
       led->fade(0, 255, 0, 300);     // fade green to red in 300 miliseconds
       ros::Duration(5).sleep();
       led->set(0, 0, 255);         // set led to blue imediatelly
       ros::Duration(5).sleep();
       
       led->blink(400);             // blink with randon colors with period 400 miliseconds
       ros::Duration(10).sleep();
              
       led->blink(255, 0, 0, 800);  // blink red colors at frequency 800 miliseconds
       ros::Duration(5).sleep();
       
       led->fade(255, 255, 255, 500);  // fade to white in 500 miliseconds
       ros::Duration(10).sleep();
       
  }
  

  return 0;
}
