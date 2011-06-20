/*
 *  Dynmx.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef __DYNMX__
#define __DYNMX__

#include <string>

// Platform specifics
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

// GLobal defines
//----------------------------------------------------------------------------------------------------------------------
#define DEBUGGING

namespace dmx
{
  
#define DATA_BASE_DIR "/Users/thomasbuhrmann/Experiments/"
#define DATA_DIR Globals::Inst()->getDataDir()
  
// Singleton for global settings determined at runtime
 //----------------------------------------------------------------------------------------------------------------------
class Globals 
{
public:
  static Globals* Inst();
  void initialise();
  std::string getDataDir() { return dataDir; };
  
protected:
  Globals();
  
private:
  static Globals* pInstance;
  std::string dataDir; 
};
  
} // namespace dmx


#endif
