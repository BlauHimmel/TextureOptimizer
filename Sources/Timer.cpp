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

