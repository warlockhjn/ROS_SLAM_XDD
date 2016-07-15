#include <cstdio>
#include <cstdlib>
#include <strings.h>
#include <memory.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1    // POSIX compliant source
#define MAX_LEN 64

typedef unsigned char BYTE;
typedef short INT16;
typedef struct {
    INT16 leftDist, rightDist, targetX, targetY;
    BYTE state1, state0;
} RecvData;

volatile bool ERROR = false;
double x = 0.0;
double y = 0.0;
double th = 0.0;														// robot start at the origin of the "odom" coordinate frame initially
ros::Time current_time, last_time;
RecvData *current_data, *last_data;

RecvData* process_data(const BYTE *pStart, const BYTE *pEnd);
void publish_odom(ros::Publisher odom_pub, tf::TransformBroadcaster broadcaster);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster broadcaster;							     // send message out using ROS and tf respectively

    ros::Rate r(10);														// publish the odometry information at a rate of 10Hz

    last_time = ros::Time::now();
    last_data = new RecvData { 0, 0 };

    int fd;
    size_t res, bias = 0, raw_len = MAX_LEN + 1;
    termios oldtio, newtio;
    BYTE buf[MAX_LEN];

    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
    if(fd < 0)
    {
        perror(MODEMDEVICE);
        return 1;
    }

    tcgetattr(fd, &oldtio);    // save current port settings

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 0;    // inter-character timer unused
    newtio.c_cc[VMIN] = 5;     // blocking read until 5 chars received

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    while(true)    // loop for input
    {
        if(bias == 0)
            res = read(fd, buf, MAX_LEN);
        else
            res = read(fd, buf + bias, MAX_LEN - bias);
        if (res == 0)
            break;
        bias += res;
        if(ERROR == true)
        {
            for(BYTE *p = buf + bias - res; p < buf + bias - 1; p++)
            {
                if(*p == 0xa5 && *(p+1) == 0x5a)
                {
                    ERROR = false;
                    memmove(buf, p, (size_t)buf + bias - (size_t)p);
                    bias = (size_t)buf + bias - (size_t)p;
                    raw_len = MAX_LEN + 1;
                    break;
                }
            }
            if(bias == MAX_LEN)
                bias = 0;
        }
        else
        {
            if(raw_len == MAX_LEN + 1 && bias > 3)
            {
                if(buf[0] != 0xa5 || buf[1] != 0x5a)
                {
                    printf("[head error! current head: 0x%02x 0x%02x]\n", buf[0], buf[1]);
                    ERROR = true;
                }
                else
                    raw_len = buf[2] + 4;
            }
            if(bias >= raw_len)
            {
                current_data = process_data(buf + 3, buf + raw_len - 1);
                if(current_data == NULL)
                    continue;
                publish_odom(odom_pub, broadcaster);
                delete last_data;
                last_data = current_data;
                bias -= raw_len;
                memmove(buf, buf + raw_len, bias);
                raw_len = MAX_LEN + 1;
            }
            if(bias == MAX_LEN)
                ERROR = true;
        }
    }

    /* restore old port settings */
    tcsetattr(fd, TCSANOW, &oldtio);
    return 0;
}

RecvData* process_data(const BYTE *pStart, const BYTE *pEnd)
{
    BYTE check = 0x00;
    for(BYTE *p = const_cast<BYTE*>(pStart); p < pEnd; check += *(p++));
    if(check != *pEnd)
    {
        printf("[sum check error! except: 0x%02x actual: 0x%02x]\n", *pEnd, check);
        return NULL;
    }
    INT16 *lDist, *rDist, *lTarg, *rTarg;
    BYTE *state1, *state0;
    RecvData *data = new RecvData {
        (pStart[0] << 8) | pStart[1], (pStart[2] << 8) | pStart[3],
        (pStart[4] << 8) | pStart[5], (pStart[6] << 8) | pStart[7],
        pStart[8], pStart[9]
    };
    return data;
}

void publish_odom(ros::Publisher odom_pub, tf::TransformBroadcaster broadcaster)
{
    ros::spinOnce();               // check for incoming messages
    last_time = current_time;
    current_time = ros::Time::now();

    double dt, dld, drd, dx, dth;
    dt = (current_time - last_time).toSec();
    if(current_data->targetY != last_data->targetY)
    {
        printf("[new action]\n");
        dld = current_data->leftDist;
        drd = current_data->rightDist;
    }
    else
    {
        dld = current_data->leftDist - last_data->leftDist;
        drd = current_data->rightDist - last_data->rightDist;
    }
    dx = (dld + drd) / 200.0;
    dth = (drd - dld) / 120.0;
    x += dx;
    th += dth;
    printf("[left distance: %hu, right distance: %hu, target x: %hu, target y: %hu]\n", current_data->leftDist, current_data->rightDist, current_data->targetX, current_data->targetY);
    //printf("[left distance: %lf, right distance: %lf, delta x: %lf, delta th: %lf, delta t: %lf]\n", dld, drd, dx, dth, dt);

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
            ros::Time::now(),
            "base_link", "base_laser"
        )
    );

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = dth / dt;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}
