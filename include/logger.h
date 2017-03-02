#pragma once

#include <iostream>

namespace logger {

enum LogLevel { LOG_DEBUG, LOG_STATUS, LOG_ERROR, LOG_NONE };

template <bool value, typename TrueResult, typename FalseResult>
struct Select {
  typedef TrueResult Type;
};

template <typename TrueResult, typename FalseResult>
struct Select<false, TrueResult, FalseResult> {
  typedef FalseResult Type;
};

struct NullStream {
  NullStream() {}
  template <typename T>
  inline NullStream& operator<<(const T&) {
    static NullStream N;
    return N;
  }
};

template <typename T, int Option>
struct ValueSelect {
  typedef T Type;
  static inline T& GetValue() {
    static T t = T();
    return t;
  }
};

template <>
struct ValueSelect<std::ostream, 0> {
  typedef std::ostream Type;
  static inline std::ostream& GetValue() { return std::cout; }
};

template <LogLevel Level, LogLevel CurrentLevel, typename LoggerStream,
          int StreamOption>
struct Log {
  typedef typename ValueSelect<LoggerStream, StreamOption>::Type StreamType;
  static inline LoggerStream& GetStream() {
    return ValueSelect<LoggerStream, StreamOption>::GetValue();
  }
};

template <LogLevel Level, typename LoggerStream, int StreamOption>
struct Log<Level, LOG_NONE, LoggerStream, StreamOption> {
  typedef typename ValueSelect<NullStream, StreamOption>::Type StreamType;
  static inline NullStream& GetStream() {
    return ValueSelect<NullStream, StreamOption>::GetValue();
  }
};

template <typename LoggerStream, int StreamOption>
struct Log<LOG_STATUS, LOG_ERROR, LoggerStream, StreamOption> {
  typedef typename ValueSelect<NullStream, StreamOption>::Type StreamType;
  static inline NullStream& GetStream() {
    return ValueSelect<NullStream, StreamOption>::GetValue();
  }
};

template <typename LoggerStream, int StreamOption>
struct Log<LOG_DEBUG, LOG_ERROR, LoggerStream, StreamOption> {
  typedef typename ValueSelect<NullStream, StreamOption>::Type StreamType;
  static inline NullStream& GetStream() {
    return ValueSelect<NullStream, StreamOption>::GetValue();
  }
};

template <typename LoggerStream, int StreamOption>
struct Log<LOG_DEBUG, LOG_STATUS, LoggerStream, StreamOption> {
  typedef typename ValueSelect<NullStream, StreamOption>::Type StreamType;
  static inline NullStream& GetStream() {
    return ValueSelect<NullStream, StreamOption>::GetValue();
  }
};

template <LogLevel CurrentLevel>
struct Logger {
  typedef Log<LOG_DEBUG, CurrentLevel, std::ostream, 0> DebugLog;
  typedef typename DebugLog::StreamType DebugStream;
  typedef Log<LOG_STATUS, CurrentLevel, std::ostream, 0> StatusLog;
  typedef typename StatusLog::StreamType StatusStream;
  typedef Log<LOG_ERROR, CurrentLevel, std::ostream, 0> ErrorLog;
  typedef typename ErrorLog::StreamType ErrorStream;
  typedef Log<LOG_NONE, CurrentLevel, NullStream, 0> NoneLog;
  typedef typename NoneLog::StreamType NoneStream;
  static inline DebugStream& DEBUG() { return DebugLog::GetStream(); }
  static inline StatusStream& STATUS() { return StatusLog::GetStream(); }
  static inline ErrorStream& ERROR() { return ErrorLog::GetStream(); }
  static inline NullStream& NONE() { return NoneLog::GetStream(); }
};

static const LogLevel LOG_LEVEL = LOG_DEBUG;

#define LOG(stream)                            \
  (logger::Logger<logger::LOG_LEVEL>::stream() \
   << "LOG(" << #stream << ")(" << __FILE__ << ":" << __LINE__ << "): ")
}  // namespace logger
