#include "robotiq_tsf/serial_port.hpp"

#include <termios.h>
#include <unistd.h>

namespace robotiq_tsf
{

bool configureSensorTty(int fd)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
        return false;

    // Raw mode: clears ICRNL/IXON/INLCR/IGNCR/ISTRIP/BRKINT (c_iflag), OPOST
    // (c_oflag) and ICANON/ECHO/ISIG/IEXTEN (c_lflag), and sets 8-bit chars.
    // This is what keeps the binary stream from being mangled — see header.
    cfmakeraw(&tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // no hardware flow control
    tty.c_cflag |= (CREAD | CLOCAL);    // enable receiver, ignore modem control lines

    tty.c_cc[VMIN] = 0;                 // non-blocking read
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

}  // namespace robotiq_tsf
