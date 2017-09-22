/*
Read and write to a can4linux device driver.

(C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

CANboat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CANboat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with CANboat.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

//#define CANFD
#include <can4linux.h>

#include "license.h"

#define BUFFER_SIZE 900

static int debug = 0;
static int verbose = 0;
static int readonly = 0;
static int writeonly = 0;
static int passthru = 0;
static long timeout = 0;
static bool isFile;

enum ReadyDescriptor
{
  FD1_Ready = 0x0001,
  FD2_Ready = 0x0002
};

static enum ReadyDescriptor isready(int fd1, int fd2);
static int readIn(unsigned char * msg, size_t len);
static void parseAndWriteIn(int handle, const unsigned char * cmd);
static void writeRaw(int handle, const unsigned char * cmd, const size_t len);
static void writeMessage(int handle, unsigned char * cmd, size_t len, unsigned int src);
static int readNGT1(int handle);
static void messageReceived(const unsigned char * msg, size_t msgLen);
static void n2kMessageReceived(const unsigned char * msg, size_t msgLen);

int main(int argc, char ** argv)
{
  int r;
  int handle;
  struct termios attr;
  char * name = argv[0];
  char * device = 0;
  struct stat statbuf;
  int pid = 0;

  setProgName(argv[0]);
  while (argc > 1)
  {
    if (strcasecmp(argv[1], "-w") == 0)
    {
      writeonly = 1;
    }
    else if (strcasecmp(argv[1], "-p") == 0)
    {
      passthru = 1;
    }
    else if (strcasecmp(argv[1], "-r") == 0)
    {
      readonly = 1;
    }
    else if (strcasecmp(argv[1], "-v") == 0)
    {
      verbose = 1;
    }
    else if (strcasecmp(argv[1], "-t") == 0 && argc > 2)
    {
      argc--;
      argv++;
      timeout = strtol(argv[1], 0, 10);
      logDebug("timeout set to %ld seconds\n", timeout);
    }
    else if (strcasecmp(argv[1], "-d") == 0)
    {
      debug = 1;
      setLogLevel(LOGLEVEL_DEBUG);
    }
    else if (!device)
    {
      device = argv[1];
    }
    else
    {
      device = 0;
      break;
    }
    argc--;
    argv++;
  }

  if (!device)
  {
    fprintf(stderr, 
    "Usage: %s [-w] -[-p] [-r] [-v] [-d] [-t <n>] device\n"
    "\n"
    "Options:\n"
    "  -w      writeonly mode, no data is read from device\n"
    "  -r      readonly mode, no data is sent to device\n"
    "  -p      passthru mode, data on stdin is sent to stdout but not to device\n"
    "  -v      verbose\n"
    "  -d      debug\n"
    "  -t <n>  timeout, if no message is received after <n> seconds the program quits\n"
    "  <device> can be a serial device, a normal file containing a raw log,\n"
    "  or the address of a TCP server in the format tcp://<host>[:<port>]\n"
    "\n" 
    "  Examples: %s /dev/ttyUSB0\n"
    "            %s tcp://192.168.1.1:10001\n"
    "\n" 
    COPYRIGHT, name, name, name);
    exit(1);
  }

retry:
  if (debug) fprintf(stderr, "Opening %s\n", device);
  if (strncmp(device, "tcp:", STRSIZE("tcp:")) == 0)
  {
    handle = open_socket_stream(device);
    if (debug) fprintf(stderr, "socket = %d\n", handle);
    isFile = true;
    if (handle < 0)
    {
      fprintf(stderr, "Cannot open NGT-1-A TCP stream %s\n", device);
      exit(1);
    }
  }
  else
  {
    // O_NONBLOCK causes buffer overruns during write?
    handle = open(device, O_RDWR | O_NOCTTY | (writeonly ? 0 : O_NONBLOCK));
    if (debug) fprintf(stderr, "fd = %d\n", handle);
    if (handle < 0)
    {
      fprintf(stderr, "Cannot open CAN device %s\n", device);
      exit(1);
    }
    if (fstat(handle, &statbuf) < 0)
    {
      fprintf(stderr, "Cannot determine device %s\n", device);
      exit(1);
    }
    isFile = S_ISREG(statbuf.st_mode);
  }

  if (isFile)
  {
    if (debug) fprintf(stderr, "Device is a normal file, do not set the attributes.\n");
  }
  else
  {
    if (debug) fprintf(stderr, "Device is special, set the attributes.\n");

    config_par_t  cfg;
    volatile command_par_t cmd;

    cmd.cmd = CMD_STOP;
    ioctl(handle, CAN_IOCTL_COMMAND, &cmd);

    cfg.target = CONF_TIMING; 
    cfg.val1   = 250;
    ioctl(handle, CAN_IOCTL_CONFIG, &cfg);

    cmd.cmd = CMD_START;
    ioctl(handle, CAN_IOCTL_COMMAND, &cmd);
  }

  srand(time(NULL));

  for (;;)
  {
    unsigned char msg[BUFFER_SIZE];
    size_t msgLen;
    enum ReadyDescriptor r;

    r = isready(writeonly ? -1 : handle, readonly ? -1 : 0);

    if ((r & FD1_Ready) > 0)
    {
      if (!readNGT1(handle))
      {
        break;
      }
    }
    if ((r & FD2_Ready) > 0)
    {
      if (!readIn(msg, sizeof(msg)))
      {
        break;
      }
      if (!passthru)
      {
        parseAndWriteIn(handle, msg);
      }
      fprintf(stdout, "%s", msg);
      fflush(stdout);
    }
    else if (writeonly)
    {
      break;
    }
  }

  close(handle);
  return 0;
}

static enum ReadyDescriptor isready(int fd1, int fd2)
{
  fd_set fds;
  struct timeval waitfor;
  int setsize;
  int r;
  enum ReadyDescriptor ret = 0;

  FD_ZERO(&fds);
  if (fd1 >= 0)
  {
    FD_SET(fd1, &fds);
  }
  if (fd2 >= 0)
  {
    FD_SET(fd2, &fds);
  }
  waitfor.tv_sec = timeout ? timeout : 10;
  waitfor.tv_usec = 0;
  if (fd1 > fd2)
  {
    setsize = fd1 + 1;
  }
  else
  {
    setsize = fd2 + 1;
  }
  r = select(setsize, &fds, 0, 0, &waitfor);
  if (r < 0)
  {
    logAbort("I/O error; restart by quit\n");
  }
  if (r > 0)
  {
    if (fd1 >= 0 && FD_ISSET(fd1, &fds))
    {
      ret |= FD1_Ready;
    }
    if (fd2 >= 0 && FD_ISSET(fd2, &fds))
    {
      ret |= FD2_Ready;
    }
  }
  if (!ret && timeout)
  {
    logAbort("Timeout %ld seconds; restart by quit\n", timeout);
  }
  return ret;
}

static void parseAndWriteIn(int handle, const unsigned char * cmd)
{
  unsigned char msg[500];
  unsigned char * m;

  unsigned int prio;
  unsigned int pgn;
  unsigned int src;
  unsigned int dst;
  unsigned int bytes;

  char * p;
  int i;
  int b;
  unsigned int byt;
  int r;

  if (!cmd || !*cmd || *cmd == '\n')
  {
    return;
  }

  p = strchr((char *) cmd, ',');
  if (!p)
  {
    return;
  }

  r = sscanf(p, ",%u,%u,%u,%u,%u,%n", &prio, &pgn, &src, &dst, &bytes, &i);
  if (r == 5)
  {
    p += i - 1;
    m = msg;
    *m++ = (unsigned char) prio;
    *m++ = (unsigned char) pgn;
    *m++ = (unsigned char) (pgn >> 8);
    *m++ = (unsigned char) (pgn >> 16);
    *m++ = (unsigned char) dst;
    //*m++ = (unsigned char) 0;
    *m++ = (unsigned char) bytes;
    for (b = 0; m < msg + sizeof(msg) && b < bytes; b++)
    {
      if ((sscanf(p, ",%x%n", &byt, &i) == 1) && (byt < 256))
      {
        *m++ = byt;
      }
      else
      {
        logError("Unable to parse incoming message '%s' at offset %u\n", cmd, b);
        return;
      }
      p += i;
    }
  }
  else
  {
    logError("Unable to parse incoming message '%s', r = %d\n", cmd, r);
    return;
  }

  writeMessage(handle, msg, m - msg, src);
}


static void writeRaw(int handle, const unsigned char * cmd, const size_t len)
{
  if (write(handle, cmd, len) != len)
  {
    logError("Unable to write command '%.*s' to NGT-1-A device\n", (int) len, cmd);
    exit(1);
  }
  logDebug("Written %d bytes\n", (int) len);
}

/*
 * Wrap the PGN message and send to can4linux device
 */
static void writeMessage(int handle, unsigned char * cmd, size_t len, unsigned int src)
{
  if (len < 6 || len > 223 + 6) {
    logError("Cannot write can message of size %d\n", len);
    return;
  }

  static int fastPacketOrder = -1;
  if (fastPacketOrder == -1)
    fastPacketOrder = rand() % 8;
  fastPacketOrder = (fastPacketOrder + 1) % 8;

  int fastPacketIndex = len > 14 ? 0 : -1;
  canmsg_t tx[32];

  tx[0].flags = MSG_EXT;
  tx[0].cob = 0;
  tx[0].length = len > 14 ? 8 : len - 6;
  tx[0].id = ((unsigned int)cmd[0] << 26) 	// Priority
    + ((unsigned int)cmd[3] << 24) + ((unsigned int)cmd[2] << 16) + ((unsigned int)cmd[1] << 8)		// PGN
    + src;
  if (cmd[1] == 0 && cmd[2] < 240)
    // PDU1 add destination address
    tx[0].id |= cmd[4] << 8;

  cmd += 6;
  len -= 6;

  while (len > 0) {
    if (fastPacketIndex == -1) {
      memcpy(tx[0].data, cmd, len);
      len = 0;
      fastPacketIndex = 0;
    }
    else 
    {
      if (fastPacketIndex > 0) {
	tx[fastPacketIndex].flags = tx[0].flags;
	tx[fastPacketIndex].cob = tx[0].cob;
	tx[fastPacketIndex].length = 8;
	tx[fastPacketIndex].id = tx[0].id;
      }
      tx[fastPacketIndex].data[0] = fastPacketIndex + (fastPacketOrder << 5);
      if (fastPacketIndex == 0) {
        tx[0].data[1] = len;
        memcpy(tx[0].data + 2, cmd, 6);
        len -= 6;
	cmd += 6;
      } else {
        memcpy(tx[fastPacketIndex].data + 1, cmd, len > 7 ? 7 : len);
        if (len < 7) {
	  memset(tx[fastPacketIndex].data + 1 + len, 0xFF, 7 - len);
	  len = 0;
	} else {
	  len -= 7;
	  cmd += 7;
	}
      }
    }

    if (debug)
    {
      fprintf(stderr, "bucket %d: flags=%08X cob=%08X ", fastPacketIndex, tx[fastPacketIndex].flags, tx[fastPacketIndex].cob);
      fprintf(stderr, "id=%08X length=%d\n", tx[fastPacketIndex].id, tx[fastPacketIndex].length);

      fprintf(stderr, "data: ");
      for (int i = 0; i < tx[fastPacketIndex].length; i++)
      {
        unsigned char c = tx[fastPacketIndex].data[i];
        fprintf(stderr, " %02X", c);
      }
      fprintf(stderr, "\n");
    }

    fastPacketIndex++;
  }

  int sent = write(handle, &tx[0], fastPacketIndex);

  if (sent < fastPacketIndex)
  {
    logError("Unable to write msg '%.*s' to CAN device\n", (int) len, cmd);
  }
  logDebug("Written msg of len %d\n", (int) len);
}

static int readIn(unsigned char * msg, size_t msgLen)
{
  bool printed = 0;
  char * s;

  s = fgets((char *) msg, msgLen, stdin);

  if (s)
  {
    if (debug)
    {
      fprintf(stderr, "in: %s", s);
    }

    return 1;
  }
  return 0;
}

static int readNGT1(int handle)
{
  size_t i;
  ssize_t r;
  bool printed = 0;
  unsigned char c;
  canmsg_t buf[10];

  r = read(handle, buf, 1);

  if (r <= 0) /* No char read, abort message read */
  {
    logAbort("Unable to read from can4linux device\n");
  }

  logDebug("Read %d messages from device\n", (int) r);
  if (debug)
  {
    for (int j = 0; j < r; j++) {
      fprintf(stderr, "msg %d: flags=%08X cob=%08X ", j, buf[j].flags, buf[j].cob);
      fprintf(stderr, "id=%08X length=%d\n", buf[j].id, buf[j].length);

      fprintf(stderr, "data: ");
      for (i = 0; i < buf[j].length; i++)
      {
        c = buf[j].data[i];
        fprintf(stderr, " %02X", c);
      }
      fprintf(stderr, "\n");
    }
  }
  for (i = 0; i < r; i++) {
    unsigned char cbuf[4 + buf[i].length];
    memcpy(cbuf, &buf[i].id, 4);
    memcpy(cbuf + 4, buf[i].data, buf[i].length);
    messageReceived(cbuf, sizeof(cbuf));
  }

  return r;
}

static void messageReceived(const unsigned char * msg, size_t msgLen)
{
  if (msgLen < 3)
  {
    logError("Ignore short command len = %zu\n", msgLen);
    return;
  }

  logDebug("message message len = %u\n", msgLen);

  n2kMessageReceived(msg, msgLen);
}

static void n2kMessageReceived(const unsigned char * msg, size_t msgLen)
{
  unsigned int prio, src, dst;
  unsigned int pgn;
  size_t i;
  unsigned int id;
  unsigned int len;
  unsigned int data[8];
  char line[800];
  char * p;
  char dateStr[DATE_LENGTH];

  if (msgLen < 5)
  {
    logError("Ignoring N2K message - too short\n");
    return;
  }
  prio = msg[3] >> 2;
  pgn  = (unsigned int) msg[1] + 256 * ((unsigned int) msg[2] + 256 * (unsigned int) (msg[3] & 0x3));
  if (pgn >> 8 < 240)
    // PDU1 (addressed)
    dst = msg[1];
  else
    // PDU2 (broadcast)
    dst  = 255;
  src  = msg[0];
  len  = msgLen - 4;

  if (len > 223)
  {
    logError("Ignoring N2K message - too long (%u)\n", len);
    return;
  }

  p = line;

  snprintf(p, sizeof(line), "%s,%u,%u,%u,%u,%u", now(dateStr), prio, pgn, src, dst, len);
  p += strlen(line);

  len += 4;
  for (i = 4; i < len; i++)
  {
    snprintf(p, line + sizeof(line) - p, ",%02x", msg[i]);
    p += strlen(p);
  }

  puts(line);
  fflush(stdout);
}

