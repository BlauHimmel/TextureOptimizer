#include "Timer.hpp"

#include <sstream>
#include <iomanip>

Timer::Timer()
{
    Reset();
}

void Timer::Reset()
{
    m_Start = std::chrono::system_clock::now();
}

double Timer::Elapsed() const
{
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_Start);
    return double(duration.count());
}

std::string Timer::ElapsedString(bool bPrecise) const
{
    return TimeString(Elapsed(), bPrecise);
}

double Timer::Lap()
{
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_Start);
    m_Start = now;
    return double(duration.count());
}

std::string Timer::LapString(bool bPrecise)
{
    return TimeString(Lap(), bPrecise);
}

std::string Timer::TimeString(double time, bool bPrecise)
{
    if (std::isnan(time) || std::isinf(time))
    {
        return "inf";
    }

    std::string suffix = "ms";
    if (time > 1000)
    {
        time /= 1000; suffix = "s";
        if (time > 60)
        {
            time /= 60; suffix = "m";
            if (time > 60)
            {
                time /= 60; suffix = "h";
                if (time > 12)
                {
                    time /= 12; suffix = "d";
                }
            }
        }
    }

    std::ostringstream os;
    os << std::setprecision(bPrecise ? 4 : 1) << std::fixed << time << suffix;
    return os.str();
}

TimerV2::TimerV2() :
        m_SecondsPerCount(0.0),
        m_DeltaTime(-1.0),
        m_BaseTime(0),
        m_PausedTime(0),
        m_StopTime(0),
        m_PrevTime(0),
        m_CurrTime(0),
        m_bStopped(false)
{
    __int64 CountsPerSec;
    QueryPerformanceFrequency((LARGE_INTEGER*)(&CountsPerSec));
    m_SecondsPerCount = 1.0 / static_cast<double>(CountsPerSec);
}


float TimerV2::TotalTime() const
{
    //                     |<--paused time-->|						   ��
    // ----*---------------*-----------------*------------*------------*------> time
    //  m_BaseTime       m_StopTime        StartTime     m_StopTime    m_CurrTime
    if (m_bStopped)
    {
        return static_cast<float>(double((m_StopTime - m_PausedTime) - m_BaseTime) * m_SecondsPerCount);
    }

        //                     |<--paused time-->|			  ��
        // ----*---------------*-----------------*------------*------> time
        //  m_BaseTime       m_StopTime        StartTime     m_CurrTime
    else
    {
        return static_cast<float>(double((m_CurrTime - m_PausedTime) - m_BaseTime) * m_SecondsPerCount);
    }
}

float TimerV2::DeltaTime() const
{
    return static_cast<float>(m_DeltaTime);
}

void TimerV2::Reset()
{
    __int64 CurrTime;
    QueryPerformanceCounter((LARGE_INTEGER*)(&CurrTime));

    m_BaseTime = CurrTime;
    m_PrevTime = CurrTime;
    m_StopTime = 0;
    m_bStopped = false;
}

void TimerV2::Start()
{
    //                     |<-------d------->|
    // ----*---------------*-----------------*------------> time
    //  m_BaseTime       m_StopTime        StartTime
    if (m_bStopped)
    {
        __int64 StartTime;
        QueryPerformanceCounter((LARGE_INTEGER*)(&StartTime));

        m_PausedTime += (StartTime - m_StopTime);

        m_PrevTime = StartTime;
        m_StopTime = 0;
        m_bStopped = false;
    }
}

void TimerV2::Stop()
{
    if (!m_bStopped)
    {
        __int64 CurrTime;
        QueryPerformanceCounter((LARGE_INTEGER*)(&CurrTime));

        m_StopTime = CurrTime;
        m_bStopped = true;
    }
}

void TimerV2::Tick()
{
    if (m_bStopped)
    {
        m_DeltaTime = 0.0;
        return;
    }

    __int64 CurrTime;
    QueryPerformanceCounter((LARGE_INTEGER*)(&CurrTime));

    m_CurrTime = CurrTime;
    m_DeltaTime = double(m_CurrTime - m_PrevTime) * m_SecondsPerCount;
    m_PrevTime = m_CurrTime;

    if (m_DeltaTime < 0.0)
    {
        m_DeltaTime = 0.0;
    }
}

