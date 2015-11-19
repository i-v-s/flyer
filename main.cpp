#include <signal.h>
#include <ros/ros.h>
//#include <mavros/mavros.h>
#include <mavros_msgs/ManualControl.h>
//#include <mavros_msgs/CommandCode.h>

#include "svo.h"
#include "mavros.h"

//#include <mavros_msgs/OverrideRCIn.h>

SVO * svo = 0;
MSF * msf = 0;
MAVROS * mavros = 0;


bool landing = false;
double zref = 0.5, zpos = 0;
bool msfReady = false;

void onSigInt(int sig)
{
    landing = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node", ros::init_options::NoSigintHandler);
    signal(SIGINT, onSigInt);
    ros::NodeHandle nh;

    msf = new MSF(nh);
    svo = new SVO(nh, msf);
    mavros = new MAVROS(nh);

    ros::Rate loop_rate(100.0);

    double th = 0.2;
    // Pre run...
    ROS_INFO("Setting OFFBOARD mode");
    for(int t = 100; t > 0 && ros::ok(); t--)
    {
        mavros->setThrottle(th);
        geometry_msgs::Quaternion att;
        att.w = 1.0;
        att.x = 0.0;
        att.y = 0.0;
        att.z = 0.0;
        mavros->setAttitude(att);
        ros::spinOnce();
        loop_rate.sleep();
        if(!mavros->setMode("OFFBOARD"))
        {
            ROS_INFO("setMode error!");
            return 1;
        }
        if(!strcmp(mavros->state.mode.data(), "OFFBOARD"))
            break;
    }
    if(!strcmp(mavros->state.mode.data(), "OFFBOARD"))
    {
        ROS_INFO("State is OFFBOARD");
    }
    else
    {
        ROS_INFO("State is '%s', not OFFBOARD", mavros->state.mode.data());
        return 1;
    }

    // Set arming

    if(mavros->setArming(true))
    {
        ROS_INFO("Set mode: Arming enabled!");
    }
    else
    {
        ROS_INFO("Set mode: Arming not enabled!");
        return 2;
    }

    for(int t = 0; t < 500 && ros::ok() && !landing; t ++)
    {
        mavros->setThrottle(th);
        geometry_msgs::Quaternion att;
        att.w = 1.0;
        att.x = 0.0;
        att.y = 0.0;
        att.z = 0.0;
        mavros->setAttitude(att);

        ros::spinOnce();
        loop_rate.sleep();
    }

   /* for(int t = 0; t < 100 && ros::ok() && !landing; t ++)
    {
        mavros->setThrottle(th);
        ros::spinOnce();
        loop_rate.sleep();
    }*/


    /*while(ros::ok() && !landing)
    {
        mavros->setThrottle(th);

        if(offboard_commands_enabled && arm_en)
        {                
            count--;
            if(!landing)
            {
                if(th < 0.4 && svo->stage != 3) th += 0.0005;
                if(svo->stage == 3)
                {
                    if(zref != 0.0 && zpos != 0.0)
                    {
                        if(zref < zpos && th > 0.2) th -= 0.0003;
                        if(zref > zpos && th < 0.42) th += 0.0003;
                    }
                    else
                        if(th > 0.1) th -= 0.0005;
                }
            }
            else
            {
                if(th > 0.2) th -= 0.0005;
            }
            if(count == 300)
            {
                ROS_INFO("Landing!!!");
                landing = true;
            }
            if(count < 0)
            {
                ROS_INFO("Disarming!");
                mavros->setArming(false);
                break;
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
    }*/

    ROS_INFO("Landing!!!");
    while(th > 0.2)
    {
        th -= 0.001;
        mavros->setThrottle(th);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Disarming!");
    mavros->setArming(false);
}
