#pragma once

#include <map>
#include <string>

namespace rynn {

/**
 * @class TimingManager
 * @brief Unified timing manager for managing multiple update frequencies
 *
 * Supports both simulation time (from physics engine) and wall-clock time (from system clock).
 * Allows registration of multiple timing channels with independent frequencies.
 */
class TimingManager {
public:
  enum class TimeSource {
    SimTime,  // Time from physics simulation (e.g., mjData_->time)
    WallTime  // Time from system clock (e.g., std::chrono)
  };

  /**
   * @brief Constructor
   * @param source Time source to use (simulation or wall-clock)
   */
  explicit TimingManager(TimeSource source = TimeSource::SimTime);

  /**
   * @brief Add a timing channel with specified frequency
   * @param name Channel name (e.g., "control", "render", "navigation")
   * @param frequency Update frequency in Hz
   */
  void addChannel(const std::string& name, double frequency);

  /**
   * @brief Check if a channel should trigger at the current time
   * @param channel Channel name
   * @param currentTime Current time (simulation time or wall-clock time)
   * @return true if the channel should trigger, false otherwise
   */
  bool shouldTrigger(const std::string& channel, double currentTime);

  /**
   * @brief Reset a specific channel to current time
   * @param channel Channel name
   * @param currentTime Current time to reset to
   */
  void reset(const std::string& channel, double currentTime);

  /**
   * @brief Reset all channels to current time
   * @param currentTime Current time to reset to
   */
  void resetAll(double currentTime);

  /**
   * @brief Get the period of a channel
   * @param channel Channel name
   * @return Period in seconds
   */
  double getPeriod(const std::string& channel) const;

  /**
   * @brief Get the time source being used
   * @return Time source (simulation or wall-clock)
   */
  TimeSource getTimeSource() const { return tSource_; }

private:
  struct ChannelTimer {
    double period;    // Update period in seconds
    double nextTime;  // Next scheduled trigger time
  };

  TimeSource tSource_;
  std::map<std::string, ChannelTimer> channels_;
};

} // namespace rynn
