#include "msf.h"
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_fusion_comm/InitScale.h>


MSF::MSF(ros::NodeHandle &nh): onPose(0)
{
    client = nh.serviceClient<sensor_fusion_comm::InitScale>("/msf_pose_sensor/pose_sensor/initialize_msf_scale");
    ros::Subscriber subPose = nh.subscribe("/msf_core/pose", 10, &MSF::poseCB, this);
}

void MSF::initScale(double scale)
{
    sensor_fusion_comm::InitScale req;
    req.request.scale = 1;
    client.call(req);
    ready = true;
}

void calcOrientSVO(const geometry_msgs::Quaternion &src, geometry_msgs::Quaternion &dst)
{
    dst.x = src.x * 0.8;
    dst.y = src.y * 0.8;
    dst.z = src.z * 0.0;
    dst.w = sqrt(1.0 - dst.x * dst.x - dst.y * dst.y - dst.z * dst.z);
}

void calcOrientMSF(const geometry_msgs::Quaternion &src, geometry_msgs::Quaternion &dst)
{
    dst.x = -src.y * 1.0;
    dst.y = src.x * 1.0;
    dst.z = src.z * 0.0;
    dst.w = sqrt(1.0 - dst.x * dst.x - dst.y * dst.y - dst.z * dst.z);
}


void MSF::poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
	if(onPose) onPose(pose->pose.pose.orientation, pose->pose.pose.position);
    geometry_msgs::PoseStamped dstPose;

    /*dstPose.pose.orientation.x = pose->pose.pose.orientation.x;
    dstPose.pose.orientation.y = pose->pose.pose.orientation.y;
    dstPose.pose.orientation.z = pose->pose.pose.orientation.z;
    dstPose.pose.orientation.w = sqrt(1.0 - 0.4 * 0.4);*/

    /*calcOrientMSF(pose->pose.pose.orientation, dstPose.pose.orientation);
    mavros_pose.publish(dstPose);



    if(svo->stage == 3)
    {
        double z = pose->pose.pose.position.z;
        /*if(!zref)
        {
            if(zct < 20)
                zsum += z;
            else
            {
                zref = zsum / (double)zct + 0.5;
                zsum = 0.0;
                ROS_INFO("Zref = %f", zref);
            }
        }*/
        /*zpos = z;
        if(!(zct & 7))
        {
            ROS_INFO("z = %f, th = %f", z, th);
        }
        zct++;
    }
    else
    {
        zpos = 0;
        //zref = 0;
        zsum = 0;
        zct = 0;  
    } */

}
