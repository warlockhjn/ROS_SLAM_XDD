#include <cstdio>
#include <cstdlib>
#include <strings.h>
#include <memory.h>

#include <ros/ros.h>

#include <sr_odom/serial_recv.h>
#include <sr_odom/serial_config.h>

volatile bool ERROR = false;

sr_odom::serial_recv* process_data(const BYTE *pStart, const BYTE *pEnd);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_data_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sr_odom::serial_recv>("serial_data", 50);

    int fd;
    size_t res, bias = 0, raw_len = MAX_LEN + 1;
    termios oldtio, newtio;
    BYTE buf[MAX_LEN];

    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
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

    newtio.c_cc[VTIME] = 1;    // inter-character timer unused
    newtio.c_cc[VMIN] = 5;     // blocking read until 5 chars received

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    while(n.ok())    // loop for input
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
                sr_odom::serial_recv* msg = process_data(buf + 3, buf + raw_len - 1);
                if(msg == NULL)
                    continue;
                pub.publish(sr_odom::serial_recv::Ptr(msg));
                ros::spinOnce();
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

sr_odom::serial_recv* process_data(const BYTE *pStart, const BYTE *pEnd)
{
    BYTE check = 0x00;
    for(BYTE *p = const_cast<BYTE*>(pStart); p < pEnd; check += *(p++));
    if(check != *pEnd)
    {
        printf("[sum check error! except: 0x%02x actual: 0x%02x]\n", *pEnd, check);
        return NULL;
    }
    sr_odom::serial_recv* msg = new sr_odom::serial_recv();
    msg->leftDist = (pStart[0] << 8) | pStart[1];
    msg->rightDist = (pStart[2] << 8) | pStart[3];
    msg->targetX = (pStart[4] << 8) | pStart[5];
    msg->targetY = (pStart[6] << 8) | pStart[7];
    msg->state1 = pStart[8];
    msg->state0 = pStart[9];
    return msg;
}
