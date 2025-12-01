#pragma once

#include <memory>

#include "runtime_data.hpp"
#include "lcm/lcm-cpp.hpp"
#include "lcm_record.hpp"
#include "orient_tools.hpp"

namespace utils {

class LcmLogger {
public:
  LcmLogger(lcmMotion::lcm_record &lcmStruct);
  ~LcmLogger() = default;

  void writeLogData(std::shared_ptr<data::RuntimeData> data);

private:
  lcmMotion::lcm_record &lcmStruct; // Reference to lcm_record
};

} // namespace utils
