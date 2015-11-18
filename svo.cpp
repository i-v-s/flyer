#include "svo.h"
#include <std_msgs/String.h>

SVO::SVO(ros::NodeHandle &nh, MSF * msf): msf(msf)
{
    subInfo = nh.subscribe("/svo/info", 100, &SVO::onInfo, this);
   	pubKey = nh.advertise<std_msgs::String>("/svo/remote_key", 10);
}

void SVO::onInfo(const svo_msgs::Info::ConstPtr& info)
{
    static int oldStage = 0, svoStart = 0;
    stage = info->stage;
    if(stage != oldStage) 
    {
        ROS_INFO("SVO stage: %d", stage);
    }
    if(stage != 3)
    {
        //zpos = 0.0;
        //zref = 0.0;
    }
    switch(stage)
    {
        case 0:
            if(svoStart > 0) svoStart --;
            else
            {
                ROS_INFO("SVO stage is 0, starting SVO");
                std_msgs::String msg;
                msg.data = "s";
                pubKey.publish(msg);
                svoStart = 20;
            }
            break;
        case 2:
            ready = true;
            break;
        case 3:
            ready = true;
            if(stage != oldStage && !msf->ready)
            {
                ROS_INFO("Initilize MSF scale");
                msf->initScale();
            }
            break;
        case 4:
            {
                static int res = 0;
                if(res > 0) res--;
                else
                {
                    ROS_INFO("SVO stage is 4, reseting SVO");
                    msf->ready = false;
                    std_msgs::String msg;
                    msg.data = "r";
                    ROS_INFO("SVO is reseting");
                    pubKey.publish(msg);
                    res = 100;
                    ROS_INFO("SVO is reseted");
                }
            }
            break;
    }
    oldStage = stage;
}

