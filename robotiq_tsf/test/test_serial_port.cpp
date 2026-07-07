// Hermetic test for the tactile-sensor tty configuration. Uses a pseudo-terminal
// (PTY) so it needs no hardware: the PTY slave stands in for the sensor's
// /dev/tty* node. The regression under test is the "flashing red" root cause —
// a tty left in cooked mode (ICRNL/IXON/OPOST) mangles the binary sensor stream.
// configureSensorTty() must force raw mode regardless of the tty's prior state.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "robotiq_tsf/serial_port.hpp"

namespace
{

// Allocate a PTY; returns the slave fd (or -1). Master fd is returned via *master
// and must be kept open for the slave to stay usable.
int openPtySlave(int *master)
{
    int m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0 || grantpt(m) != 0 || unlockpt(m) != 0)
        return -1;
    const char *name = ptsname(m);
    if (!name)
        return -1;
    int s = open(name, O_RDWR | O_NOCTTY);
    *master = m;
    return s;
}

// Force the tty into cooked mode — the state a CDC-ACM tty comes up in after a
// USB re-enumeration, which is what breaks the raw binary stream.
void makeCooked(int fd)
{
    struct termios t;
    ASSERT_EQ(tcgetattr(fd, &t), 0);
    t.c_iflag |= (ICRNL | IXON | BRKINT | ISTRIP);
    t.c_oflag |= OPOST;
    t.c_lflag |= (ICANON | ECHO | ISIG | IEXTEN);
    ASSERT_EQ(tcsetattr(fd, TCSANOW, &t), 0);
}

}  // namespace

// The core regression: after configureSensorTty(), the byte-mangling flags are
// cleared even if the tty started in cooked mode.
TEST(SerialPort, ForcesRawModeFromCooked)
{
    int master = -1;
    int slave = openPtySlave(&master);
    ASSERT_GE(slave, 0) << "could not allocate PTY: " << strerror(errno);

    makeCooked(slave);
    // sanity: the flags that mangle binary data are set before the fix runs
    struct termios before;
    ASSERT_EQ(tcgetattr(slave, &before), 0);
    ASSERT_TRUE(before.c_iflag & ICRNL);
    ASSERT_TRUE(before.c_iflag & IXON);

    ASSERT_TRUE(robotiq_tsf::configureSensorTty(slave))
        << "configureSensorTty failed: " << strerror(errno);

    struct termios after;
    ASSERT_EQ(tcgetattr(slave, &after), 0);
    EXPECT_FALSE(after.c_iflag & ICRNL) << "ICRNL still set: CR would be rewritten to NL";
    EXPECT_FALSE(after.c_iflag & IXON)  << "IXON still set: 0x11/0x13 would be eaten as flow control";
    EXPECT_FALSE(after.c_iflag & ISTRIP) << "ISTRIP still set: bytes would be stripped to 7 bits";
    EXPECT_FALSE(after.c_oflag & OPOST) << "OPOST still set: output post-processing on";
    EXPECT_FALSE(after.c_lflag & ICANON) << "ICANON still set: line-buffered, not raw";
    EXPECT_FALSE(after.c_lflag & ECHO);

    close(slave);
    close(master);
}

// Applying the config to an already-raw tty must keep it raw (idempotent).
TEST(SerialPort, IdempotentOnRawTty)
{
    int master = -1;
    int slave = openPtySlave(&master);
    ASSERT_GE(slave, 0);

    ASSERT_TRUE(robotiq_tsf::configureSensorTty(slave));
    ASSERT_TRUE(robotiq_tsf::configureSensorTty(slave));
    struct termios t;
    ASSERT_EQ(tcgetattr(slave, &t), 0);
    EXPECT_FALSE(t.c_iflag & ICRNL);
    EXPECT_FALSE(t.c_iflag & IXON);

    close(slave);
    close(master);
}

// 8-bit characters must be selected (CS8) so no data bits are lost.
TEST(SerialPort, Sets8BitChars)
{
    int master = -1;
    int slave = openPtySlave(&master);
    ASSERT_GE(slave, 0);

    ASSERT_TRUE(robotiq_tsf::configureSensorTty(slave));
    struct termios t;
    ASSERT_EQ(tcgetattr(slave, &t), 0);
    EXPECT_EQ(t.c_cflag & CSIZE, CS8);

    close(slave);
    close(master);
}
