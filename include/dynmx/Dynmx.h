/*
 *  Dynmx.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define DYNMX_WIN
#pragma warning (disable : 4996)
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned int      uint32_t;
typedef signed int        int32_t;
typedef unsigned long int uint64_t;
#else
#define DYNMX_MAC
#endif
