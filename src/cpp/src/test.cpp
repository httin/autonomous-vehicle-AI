#include "ros/ros.h"
#include "rospy_tutorials/Floats.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mytest");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<rospy_tutorials::Floats>("mytest", 10);
    ros::Rate loop_rate(10); // 10Hz
    while (ros::ok())
    {
        rospy_tutorials::Floats msg;
        std::vector<float> x(2.1,3);
        msg.data = x;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}