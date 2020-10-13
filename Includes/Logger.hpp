#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>

#define SHOW_TEST_LOG 1

class Logger
{
public:
    static void Info(const char * pFormatString, ...);
    static void Warn(const char * pFormatString, ...);
    static void Error(const char * pFormatString, ...);
    static void Test(const char * pFormatString, ...);
};

#endif //LOGGER_HPP
