#include "debug_config.hpp"

namespace utils {

DebugConfig &DebugConfig::getInstance() {
  static DebugConfig instance;
  return instance;
}

DebugConfig::DebugConfig() :
    verbose_(false),
    verboseLevel_(1),
    outputStream_(&std::cout) {
}

void DebugConfig::setVerbose(bool enable) {
  std::lock_guard<std::mutex> lock(mutex_);
  verbose_ = enable;
}

bool DebugConfig::isVerbose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return verbose_;
}

void DebugConfig::setVerboseLevel(int level) {
  std::lock_guard<std::mutex> lock(mutex_);
  verboseLevel_ = level;
}

int DebugConfig::getVerboseLevel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return verboseLevel_;
}

void DebugConfig::setOutputStream(std::ostream *stream) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream) {
    outputStream_ = stream;
  }
}

std::ostream &DebugConfig::getOutputStream() {
  std::lock_guard<std::mutex> lock(mutex_);
  return *outputStream_;
}

} // namespace utils
