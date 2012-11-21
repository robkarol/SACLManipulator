
#pragma once

#include <iostream>
#include <sstream>
#include <string.h>
#include <iomanip>
#include <time.h>
#include "windows.h"

using namespace std;

#define LOG(level) \
	if (level > Log::GetReportingLevel()) ; \
else Log().Get(level)

enum TLogLevel {logERROR, logWARNING, logINFO, logDEBUG};

/*! Logging class based on stringstreams.
	Available logging levels:
		- ERROR
		- WARNING
		- INFO
		- DEBUG
*/
class Log
{
public:
	Log();
	virtual ~Log();
	std::ostringstream& Get(TLogLevel level = logINFO);
public:
	static void SetReportingLevel(TLogLevel level);   
	static TLogLevel GetReportingLevel();
protected:
	std::ostringstream os;
private:
	Log(const Log&);
	Log& operator =(const Log&);
	string LevelToString(TLogLevel level);
	string CurrentTime();
private:
	static TLogLevel messageLevel;
};

