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

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/*
 * OPEN 3,8,3,"0:FILE,S,W"
 * 
 *  8 --> IEC device 
 *  3 --> Seconday address  ( 0-14 data channel, 15 -> command channel )
 */
typedef struct {
  unsigned char command;
  unsigned char channel;   
  uint16_t len;
  unsigned char eoi;
  unsigned char serial;
} iec_data;

enum {
  IECListenCommand   = 0x20, // + device number (0-30) 
  IECUnlistenCommand = 0x3F,
  IECTalkCommand     = 0x40, // + device number (0-30)
  IECUntalkCommand   = 0x5F,		    
  IECChannelCommand  = 0x60, // + secondary address / channel (0-15)
  IECCloseCommand    = 0xe0, // + secondary address / channel (0-15)
  IECOpenCommand     = 0xf0, // + secondary address / channel (0-15)
};

#define IECFileCommand IECCloseCommand
