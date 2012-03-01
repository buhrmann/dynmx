/*
 *  Log.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 2/24/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_LOG_
#define _DMX_LOG_

#include <sstream>
#include <string>
#include <stdio.h>

namespace dmx
{

inline std::string NowTime();

enum LogLevel 
{
  klog_Error, 
  klog_Warning, 
  klog_Info, 
  klog_Debug, 
  klog_Debug1, 
  klog_Debug2, 
  klog_Debug3
};

//----------------------------------------------------------------------------------------------------------------------    
// A logger
//----------------------------------------------------------------------------------------------------------------------    
template <typename T>
class Log
{
public:
  Log();
  virtual ~Log();
  std::ostringstream& get(LogLevel level = klog_Info);
  
  static LogLevel& reportingLevel();
  static std::string toString(LogLevel level);
  static LogLevel fromString(const std::string& level);
  
protected:
  std::ostringstream os;
  
private:
  Log(const Log&);
  Log& operator =(const Log&);
};

//----------------------------------------------------------------------------------------------------------------------
// Inline implementations
//----------------------------------------------------------------------------------------------------------------------
template <typename T>
Log<T>::Log()
{
}

//----------------------------------------------------------------------------------------------------------------------
template <typename T>
std::ostringstream& Log<T>::get(LogLevel level)
{
  os << "- " << NowTime();
  os << " " << toString(level) << ": ";
  os << std::string(level > klog_Debug ? level - klog_Debug : 0, '\t');
  return os;
}

//----------------------------------------------------------------------------------------------------------------------  
template <typename T>
Log<T>::~Log()
{
  os << std::endl;
  T::Output(os.str());
}

//----------------------------------------------------------------------------------------------------------------------  
template <typename T>
LogLevel& Log<T>::reportingLevel()
{
  static LogLevel reportingLevel = klog_Debug3;
  return reportingLevel;
}

//----------------------------------------------------------------------------------------------------------------------  
template <typename T>
std::string Log<T>::toString(LogLevel level)
{
	static const char* const buffer[] = {"ERROR", "WARNING", "INFO", "DEBUG", "DEBUG1", "DEBUG2", "DEBUG3"};
  return buffer[level];
}

//----------------------------------------------------------------------------------------------------------------------  
template <typename T>
LogLevel Log<T>::fromString(const std::string& level)
{
  if (level == "DEBUG3")
    return klog_Debug3;
  if (level == "DEBUG2")
    return klog_Debug2;
  if (level == "DEBUG1")
    return klog_Debug1;
  if (level == "DEBUG")
    return klog_Debug;
  if (level == "INFO")
    return klog_Info;
  if (level == "WARNING")
    return klog_Warning;
  if (level == "ERROR")
    return klog_Error;
  
  Log<T>().Get(klog_Warning) << "Unknown logging level '" << level << "'. Using INFO level as default.";
  return klog_Info;
}

//----------------------------------------------------------------------------------------------------------------------  
// To be used with logger above
//----------------------------------------------------------------------------------------------------------------------  
class Output2File
{
public:
  static FILE*& Stream();
  static void Output(const std::string& msg);
};

//----------------------------------------------------------------------------------------------------------------------  
inline FILE*& Output2File::Stream()
{
  // TODO: use real file
  static FILE* pStream = stderr;
  return pStream;
}

//----------------------------------------------------------------------------------------------------------------------  
inline void Output2File::Output(const std::string& msg)
{   
  FILE* pStream = Stream();
  if (!pStream)
    return;
  
  fprintf(pStream, "%s", msg.c_str());
  fflush(pStream);
}

//----------------------------------------------------------------------------------------------------------------------  
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#   if defined (BUILDING_FILELOG_DLL)
#       define FILELOG_DECLSPEC   __declspec (dllexport)
#   elif defined (USING_FILELOG_DLL)
#       define FILELOG_DECLSPEC   __declspec (dllimport)
#   else
#       define FILELOG_DECLSPEC
#   endif // BUILDING_DBSIMPLE_DLL
#else
#   define FILELOG_DECLSPEC
#endif // _WIN32

class FILELOG_DECLSPEC FILELog : public Log<Output2File> {};
//typedef Log<Output2FILE> FILELog;

#ifndef FILELOG_MAX_LEVEL
#define FILELOG_MAX_LEVEL logDEBUG4
#endif

#define FILE_LOG(level) \
if (level > FILELOG_MAX_LEVEL) ;\
else if (level > FILELog::ReportingLevel() || !Output2FILE::Stream()) ; \
else FILELog().Get(level)

//----------------------------------------------------------------------------------------------------------------------  
// Platform independent time formatting
//----------------------------------------------------------------------------------------------------------------------  
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)

#include <windows.h>
inline std::string NowTime()
{
  const int MAX_LEN = 200;
  char buffer[MAX_LEN];
  if (GetTimeFormatA(LOCALE_USER_DEFAULT, 0, 0, 
                     "HH':'mm':'ss", buffer, MAX_LEN) == 0)
    return "Error in NowTime()";
  
  char result[100] = {0};
  static DWORD first = GetTickCount();
  std::sprintf(result, "%s.%03ld", buffer, (long)(GetTickCount() - first) % 1000); 
  return result;
}

#else

#include <sys/time.h>

inline std::string NowTime()
{
  char buffer[11];
  time_t t;
  time(&t);
  tm r = {0};
  strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, &r));
  struct timeval tv;
  gettimeofday(&tv, 0);
  char result[100] = {0};
  std::sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000); 
  return result;
}
  
} // namespace dmx

#endif //WIN32

#endif //__LOG_H__