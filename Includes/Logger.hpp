#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>

class Logger
{
public:
    static void Info(const char * pFormatString, ...);
    static void Warn(const char * pFormatString, ...);
    static void Error(const char * pFormatString, ...);
};

#endif //LOGGER_HPP
