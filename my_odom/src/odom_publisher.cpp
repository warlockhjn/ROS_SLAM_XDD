#include <cstdio>
#include <cstdlib>
#include <memory.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1

typedef unsigned char BYTE;
typedef unsigned short UINT32;
typedef struct {
    UINT32 leftDist, rightDist, targetX, targetY;
    BYTE state1, state0;
} SRDATA;

bool ERROR = false;
bool WAIT_FLAG = true;                    /* TRUE while no signal received */


void signal_handler_IO(int status);   /* definition of signal handler */
SRDATA* process_data(const BYTE *pStart, const BYTE *pEnd);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster broadcaster;							     // send message out using ROS and tf respectively

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;														// robot start at the origin of the "odom" coordinate frame initially

    ros::Rate r(5);														// publish the odometry information at a rate of 5Hz

    ros::Time current_time, last_time;
    last_time = ros::Time::now();

    int fd;
    size_t res, bias = 0, raw_len = 129;
    struct termios oldtio, newtio;
    struct sigaction saio;           /* definition of signal action */
    BYTE buf[128];

    /* open the device to be non-blocking (read will return immediatly) */
    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        perror(MODEMDEVICE);
        exit(-1);
    }

    /* install the signal handler before making the device asynchronous */
    saio.sa_handler = signal_handler_IO;
    sigemptyset(&saio.sa_mask);
    sigaddset(&saio.sa_mask, SIGINT);
    //saio.sa_mask = 0;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO, &saio, NULL);

    /* allow the process to receive SIGIO */
    fcntl(fd, F_SETOWN, getpid());
    /* Make the file descriptor asynchronous (the manual page says only
        O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
    fcntl(fd, F_SETFL, FASYNC);

    tcgetattr(fd, &oldtio); /* save current port settings */
    /* set new port settings for canonical input processing */
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    while(n.ok())
    {
        usleep(100000);
        /* after receiving SIGIO, wait_flag = false, input is available
            and can be read */
        if (WAIT_FLAG == false)
        {
            if(bias == 0)
                res = read(fd, buf, 128);
            else
                res = read(fd, buf + bias, 128-bias);
            printf("[read length: %zu]\n", res);
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
                        raw_len = 129;
                        break;
                    }
                }
                if(bias == 128)
                    bias = 0;
            }
            else
            {
                if(raw_len == 129 && bias > 3)
                {
                    if(buf[0] != 0xa5 || buf[1] != 0x5a)
                    {
                        printf("[head error! current head: 0x%02x 0x%02x]\n", buf[0], buf[1]);
                        ERROR = true;
                    }
                    else
                    {
                        raw_len = buf[2] + 4;
                        printf("[raw length: %zu]\n", raw_len);
                    }
                }
                if(bias >= raw_len)
                {
                    printf("[raw data: ");
                    for(int i = 0; i < raw_len; printf("0x%02x ", buf[i++]));
                    printf("]\n");
                    SRDATA* data = process_data(buf + 3, buf + raw_len - 1);
                    if(data == NULL)
                        continue;

                    ros::spinOnce();               // check for incoming messages
                    last_time = current_time;
                    current_time = ros::Time::now();

                    double dt = (current_time - last_time).toSec(),
                           dx = (data->leftDist + data->rightDist) / 200,
                           dth = (data->rightDist - data->leftDist) / 120;

                    x += dx;
                    th += dth;

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
                    broadcaster.sendTransform(
                        tf::StampedTransform(
                            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
                            ros::Time::now(),
                            "odom", "base_link"
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
                    r.sleep();

                    delete data;
                    bias -= raw_len;
                    memmove(buf, buf + raw_len, bias);
                    raw_len = 129;
                }
                if(bias == 128)
                    ERROR = true;
            }
            if (res == 0)
                break;             /* stop loop if an error occured */
            WAIT_FLAG = true;      /* wait for new input */
        }
    }

    /* restore old port settings */
    tcsetattr(fd, TCSANOW, &oldtio);
    return 0;
}

/***************************************************************************
* signal handler. sets wait_flag to false, to indicate above loop that     *
* characters have been received.                                           *
***************************************************************************/

void signal_handler_IO(int status)
{
    WAIT_FLAG = false;
}

SRDATA* process_data(const BYTE *pStart, const BYTE *pEnd)
{
    BYTE check = 0x00;
    for(BYTE *p = (BYTE*)pStart; p < pEnd; p++)
        check += *p;
    if(check != *pEnd)
        return NULL;
    UINT32 *lDist, *rDist, *lTarg, *rTarg;
    BYTE *state1, *state0;
    SRDATA *data = new SRDATA();
    data->leftDist = *((UINT32 *)pStart);
    data->rightDist = *((UINT32 *)(pStart + 2));
    data->targetX = *((UINT32 *)(pStart + 4));
    data->targetY = *((UINT32 *)(pStart + 6));
    data->state1 = *(pStart + 7);
    data->state0 = *(pStart + 8);
    return data;
}

