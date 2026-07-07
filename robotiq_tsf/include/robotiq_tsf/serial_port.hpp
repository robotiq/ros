#pragma once

namespace robotiq_tsf
{

// Configure an already-open tty file descriptor for the tactile sensor:
//   * raw mode (cfmakeraw) so the binary sensor stream is delivered byte-for-byte;
//   * 115200 baud, 8N1, no flow control;
//   * non-blocking reads (VMIN=0, VTIME=0).
//
// The raw-mode step is the important one. A tty that has just been created by a
// USB (CDC-ACM) re-enumeration comes up in *cooked* mode, where c_iflag has
// ICRNL (translates 0x0D -> 0x0A) and IXON (consumes 0x11/0x13 as XON/XOFF).
// Those byte values occur throughout the binary sensor stream, so without raw
// mode the stream is silently mangled -> the packet framing desyncs -> the
// taxels read as rail-to-rail garbage ("flashing red"). termios settings
// persist on the tty across opens, so a reader that does not force raw mode is
// at the mercy of whatever the previous opener left behind.
//
// Returns true on success, false if tcgetattr/tcsetattr failed (errno set).
bool configureSensorTty(int fd);

}  // namespace robotiq_tsf
