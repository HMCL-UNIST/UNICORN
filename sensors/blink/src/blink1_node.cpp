/*
* Copyright (c) 2016 Carnegie Mellon University, Guilherme Pereira <gpereira@cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

/* 
 * This node was adapted from the original code of the blink1-mini-tool:
 * 
 * https://github.com/todbot/blink1/tree/master/commandline/blink1-mini-tool
 * 
 * blink1-mini-tool -- minimal command-line tool for controlling blink(1)s
 *                     
 * Will work on small unix-based systems that have just libusb-0.1.4
 * No need for pthread & iconv, which is needed for hidapi-based tools
 * 
 * Known to work on:
 * - Ubuntu Linux
 * - Mac OS X 
 * - TomatoUSB WRT / OpenWrt / DD-WRT
 *
 * 2012, Tod E. Kurt, http://todbot.com/blog/ , http://thingm.com/
 *
 */


#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>    // for memset() et al
#include <stdint.h>    // for uint8_t
#include <unistd.h>    // for usleep()
#include <time.h>      // for time()

#include "blink1/hiddata.h"
#include "blink1/Blink.h"
#include "blink1/blinkfn.h"
#include "blink1/Blink1msg.h" 

// taken from blink1/hardware/firmware/usbconfig.h
#define IDENT_VENDOR_NUM        0x27B8
#define IDENT_PRODUCT_NUM       0x01ED
#define IDENT_VENDOR_STRING     "ThingM"
#define IDENT_PRODUCT_STRING    "blink(1)"

usbDevice_t *dev;
bool blink_control;
bool exit_;
blink1::Blink::Request last_request;

int blink1_open(usbDevice_t **dev);
char *blink1_error_msg(int errCode);
void blink1_close(usbDevice_t *dev);
int blink1_fadeToRGB(usbDevice_t *dev, int fadeMillis, 
                     uint8_t r, uint8_t g, uint8_t b );
int blink1_setRGB(usbDevice_t *dev, uint8_t r, uint8_t g, uint8_t b );
static int  hexread(char *buffer, char *string, int buflen);

void SigintHandler(int sig)
{
  exit_=true; 
  blink1_fadeToRGB(dev, 1000, 0, 0, 0);
  ros::Duration(1).sleep();
  blink1_close(dev);
  ros::Duration(0.5).sleep();
  ros::shutdown();
}

bool blinkCB_service(blink1::Blink::Request  &req,  blink1::Blink::Response &res)
{
  if (req.function == BL_FADE){
       res.on=blink1_fadeToRGB(dev, req.t, req.r, req.g, req.b);
       blink_control=false;
       return true;
  } 
  
  if (req.function == BL_ON){       
       res.on=blink1_fadeToRGB(dev, 0.0, req.r, req.g, req.b);
       blink_control=false; 
       return true;
  }
  
  if ((req.function == BL_BLINK) || (req.function == BL_RANDBLINK)){
       blink_control=true;
       last_request.function=req.function;
       last_request.t=req.t;
       last_request.r=req.r;
       last_request.g=req.g;
       last_request.b=req.b;
       res.on=true;
       return true;
  }
}


void blinkCB_topic(const blink1::Blink1msg::ConstPtr& msg)
{

  if (msg->function == BL_FADE){
       blink1_fadeToRGB(dev, msg->t, msg->r, msg->g, msg->b);
       blink_control=false;
  } 
  else if (msg->function == BL_ON){       
       blink1_fadeToRGB(dev, 0.0, msg->r, msg->g, msg->b);
       blink_control=false; 
  }
  else if ((msg->function == BL_BLINK) || (msg->function == BL_RANDBLINK)){
       blink_control=true;
       last_request.function=msg->function;
       last_request.t=msg->t;
       last_request.r=msg->r;
       last_request.g=msg->g;
       last_request.b=msg->b;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "blink1_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);

  signal(SIGINT, SigintHandler);
  
  if( blink1_open(&dev) ) {
        ROS_ERROR("error: couldn't open blink1\n");
        exit(1);
  }
  
  blink_control = false;
  exit_=false;
  
  ros::ServiceServer service = n.advertiseService("blink1/blink", blinkCB_service);
  ros::Subscriber sub = n.subscribe("blink1/blink", 1, blinkCB_topic);
  
   while (ros::ok() && !exit_)
     {
      
      ros::spinOnce();
       

      if (blink_control){
	  if (last_request.function ==  BL_RANDBLINK)
	    blink1_fadeToRGB(dev, last_request.t/4, rand()%256, rand()%256, rand()%256);
	  else 
	    blink1_fadeToRGB(dev, last_request.t/4, last_request.r, last_request.g, last_request.b);
	  
          ros::Duration((double)last_request.t/2000).sleep();
          blink1_fadeToRGB(dev, last_request.t/4, 0, 0, 0);
          ros::Duration((double)last_request.t/2000).sleep();   
      }
      loop_rate.sleep();
     }
   

}


/**
 * Open up a blink(1) for transactions.
 * returns 0 on success, and opened device in "dev"
 * or returns non-zero error that can be decoded with blink1_error_msg()
 * FIXME: what happens when multiple are plugged in?
 */
int blink1_open(usbDevice_t **dev)
{
    return usbhidOpenDevice(dev, 
                            IDENT_VENDOR_NUM,  NULL,
                            IDENT_PRODUCT_NUM, NULL,
                            1);  // NOTE: '0' means "not using report IDs"
}

/**
 * Close a Blink1 
 */
void blink1_close(usbDevice_t *dev)
{
    usbhidCloseDevice(dev);
}

/**
 *
 */
int blink1_fadeToRGB(usbDevice_t *dev, int fadeMillis,
                        uint8_t r, uint8_t g, uint8_t b )
{
    char buf[9];
    int err;

    if( dev==NULL ) {
        return -1; // BLINK1_ERR_NOTOPEN;
    }

    int dms = fadeMillis/10;  // millis_divided_by_10

    buf[0] = 1;
    buf[1] = 'c';
    buf[2] = r;
    buf[3] = g;
    buf[4] = b;
    buf[5] = (dms >> 8);
    buf[6] = dms % 0xff;
    buf[7] = 0; // ledn
    //buf[8] = 0; // unused

    if( (err = usbhidSetReport(dev, buf, sizeof(buf))) != 0) {
        ROS_ERROR("fadeToRGB: error writing: %s\n",blink1_error_msg(err));
    }
    return err;  
}

/**
 *
 */
int blink1_setRGB(usbDevice_t *dev, uint8_t r, uint8_t g, uint8_t b )
{
    char buf[9];
    int err;

    if( dev==NULL ) {
        return -1; // BLINK1_ERR_NOTOPEN;
    }

    buf[0] = 1;
    buf[1] = 'n';
    buf[2] = r;
    buf[3] = g;
    buf[4] = b;
    
    if( (err = usbhidSetReport(dev, buf, sizeof(buf))) != 0) {
        ROS_ERROR("setRGB: error writing: %s\n",blink1_error_msg(err));
    }
    return err;  
}


//
char *blink1_error_msg(int errCode)
{
    static char buffer[80];

    switch(errCode){
        case USBOPEN_ERR_ACCESS:    return "Access to device denied";
        case USBOPEN_ERR_NOTFOUND:  return "The specified device was not found";
        case USBOPEN_ERR_IO:        return "Communication error with device";
        default:
            sprintf(buffer, "Unknown USB error %d", errCode);
            return buffer;
    }
    return NULL;    /* not reached */
}

//
static int  hexread(char *buffer, char *string, int buflen)
{
char    *s;
int     pos = 0;

    while((s = strtok(string, ", ")) != NULL && pos < buflen){
        string = NULL;
        buffer[pos++] = (char)strtol(s, NULL, 0);
    }
    return pos;
}

