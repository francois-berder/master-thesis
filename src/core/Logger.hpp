#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#include <iostream>
#include <mutex>

class Logger
{
    public :


        Logger(const Logger &l) = delete;

        static Logger& instance();

        void enable();
        void disable();

        void lock();
        void unlock();

        template<class T>
        Logger& operator<<(T t)
        {
            if(m_isEnabled)
                std::cout << t << std::flush;

            return *this;
        }

    private :

        Logger();

        bool m_isEnabled;
        std::mutex m_mutex;
};

#define LOG(N) do { Logger::instance().lock(); Logger::instance() << N; Logger::instance().unlock(); }while(false)
#define LOGL(N) LOG(N << "\n")

#endif /* __LOGGER_HPP__ */

