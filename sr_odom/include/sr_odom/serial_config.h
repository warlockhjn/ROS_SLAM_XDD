#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1    // POSIX compliant source
#define MAX_LEN 64
#define PULSE 0.226

typedef unsigned char BYTE;
