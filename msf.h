#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class MSF
{
private:
	ros::Subscriber subPose;
	ros::ServiceClient client;
	void poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
public:
	void (* onPose)(const geometry_msgs::Quaternion &ori, const geometry_msgs::Point &pos);
	bool ready;
	MSF(ros::NodeHandle &nh);
	void initScale(double scale = 1.0);
};
