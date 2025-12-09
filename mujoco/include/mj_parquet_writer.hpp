#pragma once

#ifdef RYNN_USE_PARQUET

#include "mj_data_writer.hpp"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>

#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace mujoco {

class ParquetWriter : public IDataWriter {
public:
  ParquetWriter();
  ~ParquetWriter() override;

  ParquetWriter(const ParquetWriter &) = delete;
  ParquetWriter &operator=(const ParquetWriter &) = delete;

  void create(const std::filesystem::path &filePath) override;
  void close() override;
  bool isOpen() const override;
  std::string getExtension() const override;

  void writeDataset(const std::string &name, const std::vector<double> &data,
                    const std::vector<size_t> &shape) override;

  void writeDataset(const std::string &name, const std::vector<float> &data,
                    const std::vector<size_t> &shape) override;

  void writeDataset(const std::string &name, const std::vector<int64_t> &data,
                    const std::vector<size_t> &shape) override;

  void writeEigenVectors(const std::string &name,
                         const std::vector<Eigen::VectorXd> &frames) override;

  void writeAttribute(const std::string &path, const std::string &attrName,
                      const std::string &value) override;

  void writeAttribute(const std::string &path, const std::string &attrName,
                      int value) override;

  void writeAttribute(const std::string &path, const std::string &attrName,
                      double value) override;

  void createGroup(const std::string &groupPath) override;

private:
  struct ColumnData {
    std::string name;
    std::shared_ptr<arrow::Array> array;
    std::shared_ptr<arrow::Field> field;
  };

  void flushToFile();

  std::filesystem::path filePath_;
  bool isOpen_{false};
  std::vector<ColumnData> columns_;
  std::map<std::string, std::string> metadata_;
};

} // namespace mujoco

#endif // RYNN_USE_PARQUET
