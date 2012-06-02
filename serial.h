#ifndef __J_SERIAL_H__
#define __J_SERIAL_H__

int serial_writeByte (int fd, uint8_t b);

int serial_write (int fd, const char *str);

int serial_readUntil (int fd, char *buf, char until);

int serial_init (const char *port, int baud);

#endif //__J_SERIAL_H__