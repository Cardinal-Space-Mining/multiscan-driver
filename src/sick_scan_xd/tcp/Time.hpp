// Time.hpp
//
// created: June 6, 2011
//
// Mit "-lrt" linken!!
//

#pragma once

#ifdef WIN32
#include <sys/timeb.h>
#include <winsock2.h> // For struct timeval
#else
#include <sys/time.h>
#endif
#include <ctime>

#include "BasicDatatypes.hpp"


// Eine Zeitspanne, in [s]
class TimeDuration
{
public:
	TimeDuration();
	explicit TimeDuration(double seconds) : m_duration(seconds) { }
	
	void set(double seconds) { m_duration = seconds; }
	[[nodiscard]] inline UINT32 total_milliseconds() const;
	inline TimeDuration& operator=(const double& seconds);
	
	double m_duration{0.0};	// Zeit, in [s]
};

// Fuer td = x;
inline TimeDuration& TimeDuration::operator=(const double& seconds)
{
	m_duration = seconds;
	return *this;
}

// Zeitspanne als [ms]
inline UINT32 TimeDuration::total_milliseconds() const
{
	UINT32 ms = (UINT32)((m_duration * 1000.0) + 0.5);
	return ms;
}


class Time
{
public:
	Time();
	explicit Time(timeval time);
	~Time() = default;
	
	void set(double time);
	void set(timeval time);
	void set(UINT64 ntpSeconds, UINT32 ntpFractionalSeconds);
	void set(UINT64 ntpTime);
	[[nodiscard]] double seconds() const;
	[[nodiscard]] UINT32 total_milliseconds() const;
	
	static Time now();	// Returns the current time
	
	Time operator+(const TimeDuration& dur) const;
	Time& operator+=(const Time& other);
	Time operator+(const Time& other) const;
	Time operator-(const Time& other) const;
	Time operator-(const double& seconds) const;
	bool operator>=(const Time& other) const;
	bool operator<(const Time& other) const;
	bool operator==(const Time& other) const;
	
	[[nodiscard]] std::string toString() const;
	[[nodiscard]] std::string toLongString() const;

	static const UINT64 secondsFrom1900to1970;
	
private:
	timeval m_time{};	// Zeit, in [s]

	static const double m_secondFractionNTPtoNanoseconds; // = 2^-32 * 1e9
	static const double m_nanosecondsToSecondFractionNTP;   // = 2^32 * 1e-9

};
