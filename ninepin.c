/* Copyright 2013 by Chris Osborn <fozztexx@fozztexx.com>
 *
 * This file is part of ninepin.
 *
 * ninepin is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * ninepin is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with ninepin; see the file COPYING. If not see
 * <http://www.gnu.org/licenses/>.
 */

#include "cbmdos.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/param.h>

int main(int argc, char *argv[])
{
  int dosfd;
  fd_set rd, ex;
  int count;
  struct timeval timeout;

  
  dosInitDrives();
  
  for (count = 1; count < argc && count < 5; count++)
    dosMountDisk(argv[count], count - 1);
 
  //iec8 will answer to device 8 commands?
  dosfd = open("/dev/iec8", O_RDWR);
  if (dosfd < 0)
    fprintf(stderr, "Unable to open IEC device\n");

  for (; dosfd >= 0 ;) {
    FD_ZERO(&rd);
    if (dosfd >= 0)
      FD_SET(dosfd, &rd);
    ex = rd;
    timeout.tv_sec = 0;
    timeout.tv_usec =  5000;
    select(NOFILE, &rd, NULL, &ex, &timeout);

    if (FD_ISSET(dosfd, &rd))
      dosHandleIO(dosfd);

    if (FD_ISSET(dosfd, &ex)) {
      fprintf(stderr, "Error with IEC\n");
      close(dosfd);
      dosfd = -1;
    }
  }
  
  exit(0);
}
