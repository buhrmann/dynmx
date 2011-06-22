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

#include "cinder/xml.h"

// Platform specifics
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define DYNMX_WIN
#pragma warning (disable : 4996)
#define USE_C99_INTS 0
#if USE_C99_INTS
typedef unsigned char     uint8_t;    /// 0...255
typedef unsigned short    uint16_t;   /// 0..65.535
typedef unsigned int      uint32_t;   /// 0..2^32
typedef signed int        int32_t;    /// -2^31...2^31
typedef unsigned long int uint64_t;   /// 0...2^64
#else
typedef int uint8_t;
typedef int uint16_t;
typedef int uint32_t;
typedef int int32_t;
typedef int uint64_t;
#endif
// MAC specific
#else
#define DYNMX_MAC
#endif

// GLobal defines
//----------------------------------------------------------------------------------------------------------------------
#define DEBUGGING 1

namespace dmx
{
  
#define DATA_BASE_DIR "/Users/thomasbuhrmann/Experiments/"
#define DATA_DIR Globals::Inst()->getDataDir()
#define SETTINGS Globals::Inst()->getSettings()

//----------------------------------------------------------------------------------------------------------------------    
// Singleton for global settings determined at runtime
//----------------------------------------------------------------------------------------------------------------------
class Globals 
{
public:
  static Globals* Inst();
  void initialise();
  std::string getDataDir() { return m_dataDir; };
  ci::XmlTree* getSettings() { return m_settings; };
  
protected:
  Globals();
  
private:
  static Globals* pInstance;
  std::string m_dataDir; 
  ci::XmlTree* m_settings;
};
  
} // namespace dmx


#endif
