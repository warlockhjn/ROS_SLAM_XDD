#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sr_odom/serial_recv.h>
#include <sr_odom/serial_config.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;    // robot start at the origin of the "odom" coordinate frame initially
ros::Publisher odom_pub;
tf::TransformBroadcaster* broadcaster;    // send message out using ROS and tf respectively
ros::Time current_time, last_time;
sr_odom::serial_recv::ConstPtr last_msg;

void serial_subscriber(const sr_odom::serial_recv::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    broadcaster = new tf::TransformBroadcaster();
    ros::Subscriber sub = n.subscribe("serial_data", 50, serial_subscriber);

    current_time = ros::Time::now();
    sr_odom::serial_recv::Ptr tmp_msg = sr_odom::serial_recv::Ptr(new sr_odom::serial_recv());
    tmp_msg->leftDist = tmp_msg->rightDist = 0;
    last_msg = tmp_msg;

    ros::spin();

    return 0;
}

void serial_subscriber(const sr_odom::serial_recv::ConstPtr& msg)
{
    last_time = current_time;
    current_time = ros::Time::now();

    double dt, dld, drd, dx, dth;
    dt = (current_time - last_time).toSec();
    dld = (msg->leftDist - last_msg->leftDist) * PULSE;
    drd = (msg->rightDist - last_msg->rightDist) * PULSE;
    dx = (dld + drd) / 2000.0;
    dth = (drd - dld) / 1200.0;
    x += dx;
    th += dth;
    printf("[left distance: %hu, right distance: %hu, target x: %hu, target y: %hu]\n", msg->leftDist, msg->rightDist, msg->targetX, msg->targetY);
    printf("[left distance: %lf, right distance: %lf, delta x: %lf, delta th: %lf, delta t: %lf]\n", dld, drd, dx, dth, dt);
    last_msg.reset();
    last_msg = msg;

    /* since all odometry is 6DOF we'll need a quaternion created from yaw */
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    /* first, we'll publish the transform over tf */
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    /* send the transform */
    broadcaster->sendTransform(odom_trans);

    /* next, we'll publish the odometry message over ROS */
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    /* set the position */
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    /* set the velocity */
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = dth / dt;

    /* publish the message */
    odom_pub.publish(odom);
}
