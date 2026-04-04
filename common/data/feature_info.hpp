#pragma once

#include <map>
#include <string>
#include <vector>

namespace rynn {

struct FeatureInfo {
  std::string dtype;
  std::vector<int> shape;
  std::vector<std::string> names;
};

using FeatureMap = std::map<std::string, FeatureInfo>;

} // namespace rynn
