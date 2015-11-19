#include "mavros.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>



MAVROS::MAVROS(ros::NodeHandle &nh)
{
	pubThrottle = nh.advertise<std_msgs::Float64>     ("/mavros/setpoint_attitude/att_throttle", 100);
    pubPose = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",     100);

    subState = nh.subscribe("/mavros/state", 10, &MAVROS::stateCB, this);

    modeClient = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    guidedClient = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/guided_enable");
    mavros_msgs::CommandBool nav_guided;
    nav_guided.request.value = true;

	armingClient = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

}

void MAVROS::stateCB(const mavros_msgs::State::ConstPtr& state)
{
    this->state = *state;
}


bool MAVROS::setMode(const char * mode)
{
    mavros_msgs::SetMode setMode;
    setMode.request.custom_mode = mode;
    return modeClient.call(setMode);
}

bool MAVROS::setArming(bool arming)
{
    mavros_msgs::CommandBool cb;
    cb.request.value = arming;
    return armingClient.call(cb);
}

void MAVROS::setAttitude(const geometry_msgs::Quaternion &o)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation = o;
    pubPose.publish(pose);
}

