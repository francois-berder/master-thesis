#include "Logger.hpp"


Logger::Logger():
m_isEnabled(true),
m_mutex()
{
}

Logger& Logger::instance()
{
    static Logger logger;
    return logger;
}

void Logger::enable()
{
    m_isEnabled = true;
}

void Logger::disable()
{
    m_isEnabled = false;
}

void Logger::lock()
{
    m_mutex.lock();
}

void Logger::unlock()
{
    m_mutex.unlock();
}
