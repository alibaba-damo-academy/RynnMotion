#pragma once

#include <iostream>
#include <mutex>
#include <string>

namespace utils {

/**
 * @class DebugConfig
 * @brief Singleton for managing global debug/verbose output settings
 *
 * Thread-safe singleton that controls verbose logging throughout the application.
 *
 * Usage:
 *   // In main.cpp
 *   DebugConfig::getInstance().setVerbose(true);
 *
 *   // In any other file
 *   DEBUG_LOG("Loading keyframes: " << count);
 *
 *   // Or check manually
 *   if (DebugConfig::getInstance().isVerbose()) {
 *     std::cout << "Debug info..." << std::endl;
 *   }
 */
class DebugConfig {
public:
  static DebugConfig &getInstance();

  void setVerbose(bool enable);
  bool isVerbose() const;

  // Set verbose level (0=off, 1=normal, 2=detailed) - for future use
  void setVerboseLevel(int level);
  int getVerboseLevel() const;

  // Set output stream (default: std::cout)
  void setOutputStream(std::ostream *stream);
  std::ostream &getOutputStream();

  // Deleted copy constructor and assignment operator (singleton pattern)
  DebugConfig(const DebugConfig &) = delete;
  DebugConfig &operator=(const DebugConfig &) = delete;

private:
  DebugConfig();
  ~DebugConfig() = default;

  bool verbose_{false};
  int verboseLevel_{1};
  std::ostream *outputStream_{&std::cout};
  mutable std::mutex mutex_; // For thread safety
};

// Convenience macros for debug logging
#define DEBUG_LOG(msg)                                                                                     \
  do {                                                                                                     \
    if (utils::DebugConfig::getInstance().isVerbose()) {                                                   \
      utils::DebugConfig::getInstance().getOutputStream() << msg << std::endl;                             \
    }                                                                                                      \
  } while (0)

#define DEBUG_LOG_LEVEL(level, msg)                                                                        \
  do {                                                                                                     \
    if (utils::DebugConfig::getInstance().getVerboseLevel() >= level) {                                    \
      utils::DebugConfig::getInstance().getOutputStream()                                                  \
          << "[DEBUG-L" << level << "] " << msg << std::endl;                                              \
    }                                                                                                      \
  } while (0)

} // namespace utils
