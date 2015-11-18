#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>

class MAVROS
{
private:
	ros::Publisher pubThrottle;
	ros::Publisher pubPose;
	ros::ServiceClient modeClient, guidedClient, armingClient;
public:
	bool setMode(const char * mode = "OFFBOARD");
	bool setArming(bool arming);
	MAVROS(ros::NodeHandle &nh);
    inline void setThrottle(double value)
    {
        std_msgs::Float64 val;
        val.data = value;
        pubThrottle.publish(val);
    }
    void setAttitude(const geometry_msgs::Quaternion &o);
};
