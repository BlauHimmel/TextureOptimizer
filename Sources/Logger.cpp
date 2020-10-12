#include "Logger.hpp"

#include <cstdarg>
#include <ctime>
#include <memory>

#ifdef WIN32
#include <Windows.h>
#define LIGHT_GREEN    (FOREGROUND_GREEN | FOREGROUND_INTENSITY)
#define LIGHT_RED      (FOREGROUND_RED | FOREGROUND_INTENSITY)
#define LIGHT_YELLOW   (FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY)
#define LIGHT_WHITE    (FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY)
#else
#define LIGHT_RED      "\e[1;31m"
#define LIGHT_GREEN    "\e[1;32m"
#define LIGHT_YELLOW   "\e[1;33m"
#define NONE           "\e[0m"
#endif

void Logger::Info(const char * pFormatString, ...)
{
    va_list args;
    std::unique_ptr<char> chars;
    va_start(args, pFormatString);
    {
        int length = _vscprintf(pFormatString, args) + 1;
        chars.reset(new char[length]);
        _vsnprintf(chars.get(), length, pFormatString, args);
    }
    va_end(args);

    time_t rawtime;
    struct tm * pTimeinfo;
    time (&rawtime);
    pTimeinfo = localtime (&rawtime);
    std::string time = asctime(pTimeinfo);
    time.pop_back();

#ifdef WIN32
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), LIGHT_GREEN);
    printf("[Info][%s]%s\n", time.c_str(), chars.get());
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), LIGHT_WHITE);
#else
    printf(LIGHT_GREEN "[Info][%s]%s\n" NONE, time.c_str(), chars.get());
#endif
}

void Logger::Warn(const char * pFormatString, ...)
{
    va_list args;
    std::unique_ptr<char> chars;
    va_start(args, pFormatString);
    {
        int length = _vscprintf(pFormatString, args) + 1;
        chars.reset(new char[length]);
        _vsnprintf(chars.get(), length, pFormatString, args);
    }
    va_end(args);

    time_t rawtime;
    struct tm * pTimeinfo;
    time (&rawtime);
    pTimeinfo = localtime (&rawtime);
    std::string time = asctime(pTimeinfo);
    time.pop_back();

#ifdef WIN32
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), LIGHT_YELLOW);
    printf("[Warn][%s]%s\n", time.c_str(), chars.get());
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), LIGHT_WHITE);
#else
    printf(LIGHT_YELLOW "[Warn][%s]%s\n" NONE, time.c_str(), chars.get());
#endif
}

void Logger::Error(const char * pFormatString, ...)
{
    va_list args;
    std::unique_ptr<char> chars;
    va_start(args, pFormatString);
    {
        int length = _vscprintf(pFormatString, args) + 1;
        chars.reset(new char[length]);
        _vsnprintf(chars.get(), length, pFormatString, args);
    }
    va_end(args);

    time_t rawtime;
    struct tm * pTimeinfo;
    time (&rawtime);
    pTimeinfo = localtime (&rawtime);
    std::string time = asctime(pTimeinfo);
    time.pop_back();

#ifdef WIN32
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), LIGHT_RED);
    printf("[Error][%s]%s\n", time.c_str(), chars.get());
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), LIGHT_WHITE);
#else
    printf(LIGHT_RED "[Error][%s]%s\n" NONE, time.c_str(), chars.get());
#endif
}