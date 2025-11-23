#include "timing_manager.hpp"

#include <iostream>
#include <stdexcept>

namespace rynn {

TimingManager::TimingManager(TimeSource source) : tSource_(source) {}

void TimingManager::addChannel(const std::string& name, double frequency) {
  if (frequency <= 0.0) {
    throw std::invalid_argument("Frequency must be positive");
  }

  ChannelTimer timer;
  timer.period = 1.0 / frequency;
  timer.nextTime = 0.0; // Will trigger immediately on first call

  channels_[name] = timer;
}

bool TimingManager::shouldTrigger(const std::string& channel, double currentTime) {
  auto it = channels_.find(channel);
  if (it == channels_.end()) {
    std::cerr << "Warning: Channel '" << channel << "' not found in TimingManager" << std::endl;
    return false;
  }

  ChannelTimer& timer = it->second;

  if (currentTime >= timer.nextTime) {
    timer.nextTime += timer.period;

    // Handle case where we've fallen behind schedule - catch up to avoid rapid consecutive triggers
    if (timer.nextTime < currentTime) {
      timer.nextTime = currentTime + timer.period;
    }

    return true;
  }

  return false;
}

void TimingManager::reset(const std::string& channel, double currentTime) {
  auto it = channels_.find(channel);
  if (it == channels_.end()) {
    std::cerr << "Warning: Channel '" << channel << "' not found in TimingManager" << std::endl;
    return;
  }

  it->second.nextTime = currentTime + it->second.period;
}

void TimingManager::resetAll(double currentTime) {
  for (auto& pair : channels_) {
    pair.second.nextTime = currentTime + pair.second.period;
  }
}

double TimingManager::getPeriod(const std::string& channel) const {
  auto it = channels_.find(channel);
  if (it == channels_.end()) {
    throw std::runtime_error("Channel '" + channel + "' not found");
  }
  return it->second.period;
}

} // namespace rynn
