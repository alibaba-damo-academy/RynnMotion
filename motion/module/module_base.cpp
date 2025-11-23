#include "module_base.hpp"

#include <iostream>

#include "robot_manager.hpp"
#include "scene_manager.hpp"

namespace rynn {

CModuleBase::CModuleBase(const YAML::Node &yamlNode) :
    _yamlNode(yamlNode),
    startTime(std::chrono::steady_clock::now()),
    lastCallTime(std::chrono::steady_clock::now()) {
}

bool CModuleBase::resetModule() {
  return true;
}

double CModuleBase::getCurrentTime() const noexcept {
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - startTime);
  return duration.count();
}

double CModuleBase::getDurationSinceLastCall() const noexcept {
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastCallTime);
  lastCallTime = now;
  return duration.count();
}

} // namespace rynn
