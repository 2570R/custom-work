#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <stdarg.h>

class Logger {
public:
    enum class Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        FATAL,
        TELEMETRY
    };
    
    using Handle = std::function<void(Level, const std::string&)>;
    
    // Constructor
    Logger(std::ostream& output_stream, Level level = Level::INFO);
    
    // Level management
    void set_level(Level level);
    void add_handle(const Handle& handle);
    
    // Logging methods
    void telemetry(const char* format, ...) const;
    void debug(const char* format, ...) const;
    void info(const char* format, ...) const;
    void warning(const char* format, ...) const;
    void error(const char* format, ...) const;
    void fatal(const char* format, ...) const;
    
    // Static utility methods
    static std::string level_to_string(Level level);
    static std::string colorize(const std::string& message, Level level);

private:
    void log(Level level, const char* format, va_list args) const;
    const std::string format(const char* const format, va_list args) const;
    
    std::ostream& output_stream;
    Level level;
    std::vector<Handle> handles;
};

