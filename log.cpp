#include "log.h"

TLogLevel Log::messageLevel = logINFO;


Log::Log()
{
	if(os == NULL)
	{
		os = ostringstream(ostringstream::out);
	}

}

void Log::SetReportingLevel(TLogLevel level)
{
	messageLevel = level;
}

TLogLevel Log::GetReportingLevel()
{
	return messageLevel;
}

std::ostringstream& Log::Get(TLogLevel level)
{
	os << CurrentTime() << " - ";
	os <<  LevelToString(level) << ": ";

	if(level < logDEBUG)
	{
		os << "\t";
	}
	messageLevel = level;
	return os;
}
Log::~Log()
{
	os << std::endl;
	fprintf(stderr, "%s", os.str().c_str());
	fflush(stderr);
}

string Log::LevelToString(TLogLevel l)
{
  switch(l)
  {
  case logERROR: return "ERROR";
  case logWARNING: return "WARNING";
  case logINFO: return "INFO";
  case logDEBUG: return "DEBUG";
  default: return "";
  }
}

string Log::CurrentTime()
{
	SYSTEMTIME time;
	GetLocalTime(&time);
	WORD millis = (time.wSecond * 1000) + time.wMilliseconds;

	stringstream ss;
	ss <<setw(2)<< time.wHour;
	ss <<':'<<time.wMinute;
	ss <<':'<< time.wSecond;
	ss <<'.'<<setw(3)<<time.wMilliseconds;
	return ss.str();
}