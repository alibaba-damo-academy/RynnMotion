#pragma once

#include <memory>

namespace data {
struct DataMsg {
  virtual ~DataMsg() = default;
  virtual std::shared_ptr<DataMsg> clone() const = 0;

protected:
  DataMsg() = default;
  DataMsg(const DataMsg &) = default;
  DataMsg &operator=(const DataMsg &) = default;
};
} // namespace data