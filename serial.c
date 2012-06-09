/*
 * The serial controller for the Jade Guidance Server.
 * 
 * Copyright (c) 2012, Adam Thompson <thompsona17@nku.edu>
 */

#include "serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <getopt.h>

/**
 * serial_writeByte()
 */
int serial_writeByte ( int fd, uint8_t b )
{
    int n = write( fd, &b, 1 );
    
    if( n != 1)
        return -1;    
    return 0;
}

/**
 * serial_write()
 */
int serial_write ( int fd, const char *str )
{
    int len = strlen( str );
    int n = write( fd, str, len );
    
    if( n != len)
        return -1;
    return 0;
}

/**
 * serial_readUntil()
 */
int serial_readUntil ( int fd, char *buf, char until)
{
    char b[1];
    int i = 0;
    
    do {
        int n = read( fd, b, 1 ); // Read 1 char at a time
        
        if( n == -1)
            return -1 // Failed to read
        if( n == 0 ) {
            usleep( 10 * 1000 ); // Wait 10 msec before trying again
            continue;
        }
        
        buf[i] = b[0];
        i++;
    } while( b[0] != until );
    
    buf[i] = 0; // Null terminate the string
    
    return 0;
}

/**
 * serial_init()
 * 
 * Recieves the string name of the serial port (I.E. "/dev/tty.usbserial", "COM1")
 * and the baud rate (bps) and connects to that port with that speed and 8N1.
 * The port is opened in full raw mode, so it's possible to send binary data.
 * returns valid fd; -1 on error
 */
int serial_init ( const char *port, int baud )
{
    struct termios toptions;
    int fd;
    
    /*********************************
     * Uncomment for debugging       *
     ********************************/
    //fprintf(stderr, "init_serial: opening port %s @ %d bps \n",
    //        port, baud);
    
    fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY );
    
    if( fd == -1 ) {
        perror( "serial_init: Unable to open port" );
        return -1;
    }
    
    if( tcgetattr( fd, &toptions ) < 0 ) {
        perror( "serial_init: Couldn't get term attributes" );
        return -1;
    }
    
    speed_t brate = baud; // Allows overriding below switch if ever needed
    switch(baud) 
    {
        case 4800:
            brate = B4800;
            break;
        case 9600:
            brate = B9600;
            break;
#ifdef B14400
        case 14400:
            brate = B14400;
            break;
#endif
        case 19200:
            brate = B19200;
            break;
#ifdef B28800
        case 28800:
            brate = B28800;
            break;
#endif
        case 38400:
            brate = B38400;
            break;
        case 57600:
            brate = B57600;
            break;
        case 115200:
            brate = B115200;
            break;
    }
    cfsetispeed( &toptions, brate );
    cfsetospeed( &toptions, brate );
    
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // No flow control
    toptions.c_cflag &= ~CRTSCTS;
    
    toptions.c_cflag |= CREAD | CLOCAL; // Turn on READ && ignore ctrl lines
    toptions.c_cflag &= ~( IXON | IXOFF | IXANY ); // Turn off s/w flow ctrl
    
    toptions.c_cflag &= ~( ICANON | ECHO | ECHOE | ISIG ); // Make raw
    toptions.c_cflag &= ~OPOST; // Make raw
    
    // Reference: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[vmin] = 0;
    toptions.c_cc[vtime] = 20;
    
    if( tcsetattr( fd, TCSANOW, &toptions ) < 0 ) {
        perror( "serial_init: Couldn't set term attributes");
        return -1;
    }
    
    return fd;
}