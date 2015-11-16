#include <ros/ros.h>
//#include <mavros/mavros.h>
#include <mavros_msgs/ManualControl.h>
//#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <svo_msgs/Info.h>
#include <sensor_fusion_comm/InitScale.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

//#include <mavros_msgs/OverrideRCIn.h>

int svoReset = 0;
int svoStart = 0;
int svoStage = 0;
bool svoReady = false;
double zref = 0, zpos = 0;
ros::Publisher svoKey;

ros::ServiceClient msf_client;// = nh.serviceClient<sensor_fusion_comm::InitScale>("/msf_pose_sensor/pose_sensor/initialize_msf_scale");

void msfPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    static int p = 0;
    static double zsum = 0;
    if(svoStage == 3)
    {
        p++;
        double z = pose->pose.pose.position.z;
        if(p < 20)
            zsum += z;
        else if(p == 20) 
        {
            zref = (zsum / 20) + 0.1;
            ROS_INFO("Zref = %f", zref);
        }
        zpos = z;
        if(!(p & 15))
        {
            ROS_INFO("z = %f", z);
        }
    }

}

void svoInfoCB(const svo_msgs::Info::ConstPtr& info)
{
    static int oldStage = 0;
    svoStage = info->stage;
    if(svoStage != oldStage) 
    {
        ROS_INFO("SVO stage: %d", svoStage);
    }
    switch(svoStage)
    {
        case 0:
            if(svoStart > 0) svoStart --;
            else
            {
                ROS_INFO("SVO stage is 0, starting SVO");
                std_msgs::String msg;
                msg.data = "s";
                svoKey.publish(msg);
                svoStart = 20;
            }
            break;
        case 2:
            svoReady = true;
            break;
        case 3:
            svoReady = true;
            if(svoStage != oldStage) 
            {
                ROS_INFO("Initilize MSF scale");
                sensor_fusion_comm::InitScale req;
                req.request.scale = 1;
                msf_client.call(req);
            }
            break;
        case 4:
            {
                static int res = 0;
                if(res > 0) res--;
                else
                {
                    ROS_INFO("SVO stage is 4, reseting SVO");
                    std_msgs::String msg;
                    msg.data = "r";
                    svoKey.publish(msg);
                    res = 20;
                }
            }
            break;
    }
    oldStage = svoStage;


/*        svoReset++;
        if(svoReset > 100)
        {
            svoReset = 0;
            ROS_INFO("Reseting SVO");
            std_msgs::String msg;
            msg.data = "r";
            svoKey.publish(msg);

        }
    }*/
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node");
    ros::NodeHandle nh;

    ros::Publisher mavros_throttle = nh.advertise<std_msgs::Float64>                   ("/mavros/setpoint_attitude/att_throttle", 100);
    ros::Publisher mavros_pose     = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",      100);
    //ros::Publisher mavros_control_pub = nh.advertise<mavros_msgs::ControlSetpoint>("/mavros/setpoint_control/setpoint",1000);
    //mavros::ControlSetpoint fmu_controller_setpoint;

    ros::ServiceClient mavros_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";
    
    ros::ServiceClient mavros_nav_guided_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/guided_enable");
    mavros_msgs::CommandBool nav_guided;
    nav_guided.request.value = true;

    msf_client = nh.serviceClient<sensor_fusion_comm::InitScale>("/msf_pose_sensor/pose_sensor/initialize_msf_scale");


    ros::Subscriber svoInfo = nh.subscribe("/svo/info", 100, svoInfoCB);
    svoKey = nh.advertise<std_msgs::String>("/svo/remote_key", 10);

    ros::Subscriber msfPose = nh.subscribe("/msf_core/pose", 10, msfPoseCB);

    ros::ServiceClient mavros_arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool arming;
    arming.request.value = true;

    mavros_msgs::CommandBool offboard;
    offboard.request.value = true;


    double th = 0.1;
    mavros_throttle.publish(th);
    //geometry_msgs::PoseStamped = 
    //ros::ServiceClient mavros_rc = nh.serviceClient<mavros_msgs::OverrideRCIn>("/mavros/rc/override");
    //mavros_msgs::OverrideRCIn rc;
    //rc.request.value = true;

    
    bool offboard_commands_enabled = false;
    bool nav_guided_enabled = false;
    bool arm_en = false;
    ros::Rate loop_rate(100.0);

    int count = 1500;
    for(int t = 100; t > 0 && ros::ok(); t--)
    {
        mavros_throttle.publish(th);
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(ros::ok())
    {
        if (!offboard_commands_enabled) {
            if(mavros_set_mode_client.call(set_mode))
            //if (mavros_nav_guided_client.call(offboard))
            {
                ROS_INFO("Set mode: OFFBOARD enabled!");
                offboard_commands_enabled = true;
                
            }
            else
            {
                ROS_INFO("Offboard mode still not enabled!");
            }
        }

        mavros_throttle.publish(th);

        /*if(!nav_guided_enabled)
        {
            if (mavros_nav_guided_client.call(nav_guided))
            {
               nav_guided_enabled = true;
               ROS_INFO("Nav guided: OFFBOARD enabled!");
            }
        }*/

        // Write desired setpoint value to fmu_controller_setpoint varialbe.

        //mavros_control_pub.publish(fmu_controller_setpoint);

        if(offboard_commands_enabled && !arm_en && svoReady)
        {
            if(mavros_arming_client.call(arming))
            {
                arm_en = true;
                ROS_INFO("Set mode: Arming enabled!");
            }
            else
            {
                ROS_INFO("Set mode: Arming not enabled!");
            }
        }
        if(offboard_commands_enabled && arm_en)
        {                
            count--;
            if(th < 0.5 && svoStage != 3) th += 0.0005;
            if(svoStage == 3)
            {
                if(zref != 0.0 && zpos != 0.0)
                {
                    if(zref < zpos && th > 0.1) th -= 0.0005;
                    if(zref > zpos && th < 0.5) th += 0.0005;
                }
                else
                    if(th > 0.1) th -= 0.0005;
            }
            if(count < 0)
            {
                ROS_INFO("Disarming!");
                arming.request.value = false;
                mavros_arming_client.call(arming);
                break;
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

}
