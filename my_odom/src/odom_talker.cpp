#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_talker");

  ros::NodeHandle n;

  ros::Publisher Element_pub = n.advertise<std_msgs::Float64MultiArray>("from_odom", 50);

  ros::Rate loop_rate(5);

  double count = 1;
  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg;
	
    msg.data[0] = count;
	msg.data[1] = count+0.1;
	msg.data[2] = count-0.1;
    Element_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}