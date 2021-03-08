#ifndef TIMER_H
#define TIMER_H

#include <string>
#include <chrono>
#include <Windows.h>

/**
* \brief Simple timer with millisecond precision
*
* This class is convenient for collecting performance data
*/
class Timer
{
public:
    /// Create a new timer and reset it
    Timer();

    /// Reset the timer to the current time
    void Reset();

    /// Return the number of milliseconds elapsed since the timer was last reset
    double Elapsed() const;

    /// Like  Elapsed(), but return a human-readable string
    std::string ElapsedString(bool bPrecise = false) const;

    /// Return the number of milliseconds elapsed since the timer was last reset and then reset it
    double Lap();

    /// Like Lap(), but return a human-readable string
    std::string LapString(bool bPrecise = false);

private:
    static std::string TimeString(double time, bool bPrecise = false) ;

private:
    std::chrono::system_clock::time_point m_Start;
};

class TimerV2
{
public:

    TimerV2();

public:

    float TotalTime() const;
    float DeltaTime() const;

    void Reset();
    void Start();
    void Stop();
    void Tick();

private:

    double m_SecondsPerCount;
    double m_DeltaTime;

    __int64 m_BaseTime;
    __int64 m_PausedTime;
    __int64 m_StopTime;
    __int64 m_PrevTime;
    __int64 m_CurrTime;

    bool m_bStopped;
};

#endif //TIMER_H
