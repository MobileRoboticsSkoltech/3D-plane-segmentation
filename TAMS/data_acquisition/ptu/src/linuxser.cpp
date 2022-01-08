#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <poll.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <byteswap.h> /* GNU extension */
#include "ptu/linuxser.h"

namespace ptu
{

/* to keep things simpler, we'll support only a fixed number of ports
 * in the future, we can use a dynamically expanding array to get
 * nearly unlimited ports. */
#define MAX_PORTS                   16

/* default is always little endian */
#define INT_REVERSED

/* function decls */
int serial_init(void);
int serial_open(const char *portname, int baud);
int serial_close(int fd);
int serial_close_all(void);
int serial_send(int fd, void *buff, int len);
int serial_recv(int fd, void *buff, int len);
int serial_flush(int fd);
int serial_isok(int fd);
/* timeout in miliseconds */
int serial_can_recv(int fd, int timeout);

typedef struct _descriptor_t
{
  struct termios oldtio;
  int fd;
} descriptor_t;

static descriptor_t descriptors[MAX_PORTS];
static descriptor_t empty_descriptor;

/* comparitor function for sorting and searching the binary tree */
static int descriptor_compare(const void *va, const void *vb)
{
  descriptor_t *a = (descriptor_t *)va;
  descriptor_t *b = (descriptor_t *)vb;

  return (a->fd - b->fd);
}

/* lookup a descriptor */
static descriptor_t *descriptor_lookup(int fd)
{
  descriptor_t key;
  key.fd = fd;
  return (static_cast<descriptor_t *> (bsearch(&key, descriptors, MAX_PORTS, sizeof(descriptor_t), descriptor_compare)));
}

/* fixup the binary tree by sorting it */
static void descriptor_fixup(void)
{
  qsort(descriptors, MAX_PORTS, sizeof(descriptor_t), descriptor_compare);
}

/* removes a element from the binary tree */
static void descriptor_free(descriptor_t *slot)
{
  slot->fd = INT_MAX;
  descriptor_fixup();
}

/* called when exit() is called */
static void serial_shutdown(void)
{
  serial_close_all();
}

int serial_init(void)
{
  int i;

  /* fds should never be as big as MAX_INT, so we'll use this as a special
   * flag */
  empty_descriptor.fd = INT_MAX;
  for (i = 0; i < MAX_PORTS; i++)
    descriptors[i] = empty_descriptor;

  /* exiting should cause closing of ports */
  atexit(serial_shutdown);

  return 0;
}

int serial_open(const char *portname, int baud)
{
  struct termios newtio, *oldtio;
  int fd;
  descriptor_t *slot;

  /* open the port */
  if ((fd = open(portname, O_RDWR)) < 0)
  {
    return -1;
  }

  /* find an open slot */
  slot = descriptor_lookup(INT_MAX);
  if (!slot)
  { /* no open slots left */
    close(fd);
    return -5;
  }

  /* use said slot */
  slot->fd = fd;
  oldtio = &slot->oldtio;

  /* fix the structure */
  descriptor_fixup();

  /* get port settings and save them */
  if (tcgetattr(fd, oldtio) < 0)
  {
    descriptor_free(slot);
    close(fd);
    return -10;
  }

  /* set what we want */
  newtio = *oldtio;
  newtio.c_iflag = IGNBRK | IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_lflag = 0;
#if 0
  newtio.c_cc[VMIN] = 0;
  newtio.c_cc[VTIME] = 50;
#endif
  switch (baud)
  {
    case 1200:
      cfsetispeed(&newtio, B1200);
      cfsetospeed(&newtio, B1200);
      break;
    case 2400:
      cfsetispeed(&newtio, B2400);
      cfsetospeed(&newtio, B2400);
      break;
    case 4800:
      cfsetispeed(&newtio, B4800);
      cfsetospeed(&newtio, B4800);
      break;
    case 9600:
      cfsetispeed(&newtio, B9600);
      cfsetospeed(&newtio, B9600);
      break;
    case 19200:
      cfsetispeed(&newtio, B19200);
      cfsetospeed(&newtio, B19200);
      break;
    case 38400:
      cfsetispeed(&newtio, B38400);
      cfsetospeed(&newtio, B38400);
      break;
    case 57600:
      cfsetispeed(&newtio, B57600);
      cfsetospeed(&newtio, B57600);
      break;
    case 115200:
      cfsetispeed(&newtio, B115200);
      cfsetospeed(&newtio, B115200);
      break;
    case 230400:
      cfsetispeed(&newtio, B230400);
      cfsetospeed(&newtio, B230400);
      break;
    case 460800:
      cfsetispeed(&newtio, B460800);
      cfsetospeed(&newtio, B460800);
      break;
    case 921600:
      cfsetispeed(&newtio, B921600);
      cfsetospeed(&newtio, B921600);
      break;
    default:
      /* unsupported baud */
      descriptor_free(slot);
      close(fd);
      return -20;
  }

  /* flush and set new settings */
  if (tcflush(fd, TCIOFLUSH))
  {
    descriptor_free(slot);
    close(fd);
    return -30;
  }
  if (tcsetattr(fd, TCSANOW, &newtio))
  {
    descriptor_free(slot);
    close(fd);
    return -40;
  }

  return fd;
}

int serial_close(int fd)
{
  descriptor_t *slot;

  /* flush */
  serial_flush(fd);

  /* find slot and cleanup */
  if ((slot = descriptor_lookup(fd)) != NULL)
  {
    tcsetattr(fd, TCSANOW, &slot->oldtio);
    slot->fd = INT_MAX;
    descriptor_fixup();
  }

  close(fd);
  return 0;
}

int serial_close_all(void)
{
  int i;

  for (i = 0; i < MAX_PORTS; i++)
  {
    if (descriptors[i].fd != INT_MAX)
      serial_close(descriptors[i].fd);
  }

  return 0;
}

/* write and return number of bytes written */
int serial_send(int fd, void *buff, int len)
{
  int bleft;
  ssize_t bwrite;
  uint8_t *wptr = (uint8_t *)buff;

  bleft = len;
  while ((bwrite = write(fd, wptr, bleft)) >= 0 && (bleft - bwrite) > 0)
  {
    bleft -= bwrite;
    wptr += bwrite;
  }
  if (bwrite > 0)
    bleft -= bwrite;

  /* XXX: Check + report error */

  return (len - bleft);
}

int serial_recv(int fd, void *buff, int len)
{
  int bleft;
  ssize_t bread;
  uint8_t *rptr = (uint8_t *)buff;

  bleft = len;
  while ((bread = read(fd, rptr, bleft)) >= 0 && (bleft - bread) > 0)
  {
    bleft -= bread;
    rptr += bread;
  }
  if (bread > 0)
    bleft -= bread;

  /* XXX: Check + report error */

  return (len - bleft);
}

int serial_flush(int fd)
{
  return tcflush(fd, TCIOFLUSH);
}

int serial_isok(int fd)
{
  return (write(fd, NULL, 0) == 0) ? 0 : -1;
}

int serial_can_recv(int fd, int timeout)
{
  struct pollfd pfd;

  pfd.fd = fd;
  pfd.events = POLLIN;

  return poll(&pfd, 1, timeout);
}

/*** CPI interface ***/

static int linuxser_sel_baud = 0;
static void test_set_baud(void)
{
  if (!linuxser_sel_baud)
  {
    serial_init();
    linuxser_sel_baud = 9600;
  }
}

/* rx_timeout - true if timeout < 0 or if RX availabe within timeout (ms) */
static int rx_timeout(portstream_fd fd, long timeout)
{
  return (timeout < 0 || serial_can_recv(fd, timeout));
}

portstream_fd openserial(const char *port_name)
{
  int fd;

  test_set_baud();

  fd = serial_open(port_name, linuxser_sel_baud);
  if (fd < 0)
    return 0;
  return fd;
}

char setbaudrate(int baudrate)
{
  test_set_baud();
  linuxser_sel_baud = baudrate;
  return 0;
}

char closeserial(portstream_fd fd)
{
  serial_close(fd);
  return 0;
}

char SerialBytesOut(portstream_fd fd, unsigned char *buffer, int nBytes)
{
  if (serial_send(fd, buffer, nBytes) != nBytes)
    return -1;

  return 1;
}

char SerialBytesIn(portstream_fd fd, unsigned char *buffer, unsigned int nBytes, long timeoutVal)
{
  unsigned int bytes_left;
  int c;

  bytes_left = nBytes;
  while (bytes_left > 0 && rx_timeout(fd, timeoutVal))
  {
    c = serial_recv(fd, buffer, 1);
    buffer += c;
    bytes_left -= c;
  }

  if (bytes_left == 0)
    return 1;

  return -1;
}

char PeekByte(portstream_fd fd, unsigned char *byte)
{
  return 0;
}

char FlushInputBuffer(portstream_fd fd)
{
  tcflush(fd, TCIFLUSH);
  return 1;
}

char SerialStringOut(portstream_fd fd, unsigned char *buffer)
{
  if (serial_send(fd, buffer, strlen((char *)buffer)) != strlen((char *)buffer))
    return -1;
  return 1;
}

char ReadSerialLine(portstream_fd fd, unsigned char *strbuffer, long timeout, int *charsRead)
{
  *charsRead = 0;
  while (rx_timeout(fd, timeout))
  {
    if (serial_recv(fd, strbuffer, 1) != 1)
      return 0;
    if (*strbuffer == '\n')
    {
      *strbuffer = 0;
      return 1;
    }
    strbuffer++;
    (*charsRead)++;
  }

  return 0;
}

void do_delay(long msec)
{
  struct timespec ts, rem;
  int retc;

  ts.tv_sec = msec / 1000;
  ts.tv_nsec = (msec % 1000) * 1000000;
  do
  {
    retc = nanosleep(&ts, &rem);
    ts = rem;
  } while (retc == -1);
}

char GetSignedShort(portstream_fd fd, signed short *SHORTval, long timeout)
{
  SerialBytesIn(fd, (unsigned char *)SHORTval, 2, timeout);
#ifdef INT_REVERSED
  *SHORTval = bswap_16(*SHORTval);
#endif
  return 1;
}

char PutSignedShort(portstream_fd fd, signed short *SHORTval)
{
  signed short v = *SHORTval;

#ifdef INT_REVERSED
  v = bswap_16(v);
#endif
  serial_send(fd, &v, 2);
  return 1;
}

char GetUnsignedShort(portstream_fd fd, unsigned short *USHORTval, long timeout)
{
  SerialBytesIn(fd, (unsigned char *)USHORTval, 2, timeout);
#ifdef INT_REVERSED
  *USHORTval = bswap_16(*USHORTval);
#endif
  return 1;
}

char PutUnsignedShort(portstream_fd fd, unsigned short *USHORTval)
{
  unsigned short v = *USHORTval;

#ifdef INT_REVERSED
  v = bswap_16(v);
#endif
  serial_send(fd, &v, 2);
  return 1;
}

char GetSignedLong(portstream_fd fd, signed long *LONGval, long timeout)
{
  SerialBytesIn(fd, (unsigned char *)LONGval, 4, timeout);
#ifdef INT_REVERSED
  *LONGval = bswap_32(*LONGval);
#endif
  return 1;
}

char PutSignedLong(portstream_fd fd, signed long *LONGval)
{
  signed long v = *LONGval;

#ifdef INT_REVERSED
  v = bswap_32(v);
#endif
  serial_send(fd, &v, 4);
  return 1;
}

}//endof namespace ptu
