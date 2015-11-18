#include <ros/ros.h>
#include <svo_msgs/Info.h>
#include "msf.h"

class SVO
{
private:
	ros::Subscriber subInfo;
	ros::Publisher pubKey;
	void onInfo(const svo_msgs::Info::ConstPtr& info);
	MSF * msf;
public:
	int stage;
	bool ready;


	SVO(ros::NodeHandle &nh, MSF * msf);

};
