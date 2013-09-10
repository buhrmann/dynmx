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
#include <pwd.h>
#include <unistd.h>

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
#define DEPLOYING 0

namespace dmx
{

#define DEFAULT_FRAMERATE 100
  
#define DATA_DIR Globals::Inst()->getDataDir()
#define SETTINGS Globals::Inst()->getSettings()

//----------------------------------------------------------------------------------------------------------------------      
static std::string pathExpandHome(const std::string& path)
{
  if(path[0] == '~')
  {
    const char *homeDir = getenv("HOME");
    
    if (!homeDir)
    {
      struct passwd* pwd = getpwuid(getuid());
      if (pwd)
        homeDir = pwd->pw_dir;
    }
    return std::string(homeDir) + path.substr(1, path.length());
  }
  else
  {
    return path;
  }
}  
  
//----------------------------------------------------------------------------------------------------------------------    
// Singleton for global settings determined at runtime
//----------------------------------------------------------------------------------------------------------------------
class Globals 
{
public:
  static Globals* Inst(const std::string& cfgFnm="");
  std::string getDataDir() { return m_dataDir; };
  ci::XmlTree* getSettings() { return m_settings; };
  
protected:
  Globals(const std::string& cfgFnm="");
  void initialise(const std::string& cfgFnm="");
  void saveToDataDir();  
  
private:
  static Globals* pInstance;
  std::string m_dataDir; 
  std::string m_fnm;
  ci::XmlTree* m_settings;
};
  
} // namespace dmx


#endif
