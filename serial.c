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