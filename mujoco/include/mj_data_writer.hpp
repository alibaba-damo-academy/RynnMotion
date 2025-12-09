#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace mujoco {

enum class DataFormat { None, Parquet, HDF5 };

class IDataWriter {
public:
  virtual ~IDataWriter() = default;

  virtual void create(const std::filesystem::path &filePath) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual std::string getExtension() const = 0;

  virtual void writeDataset(const std::string &name,
                            const std::vector<double> &data,
                            const std::vector<size_t> &shape) = 0;

  virtual void writeDataset(const std::string &name,
                            const std::vector<float> &data,
                            const std::vector<size_t> &shape) = 0;

  virtual void writeDataset(const std::string &name,
                            const std::vector<int64_t> &data,
                            const std::vector<size_t> &shape) = 0;

  virtual void writeEigenVectors(const std::string &name,
                                 const std::vector<Eigen::VectorXd> &frames) = 0;

  virtual void writeAttribute(const std::string &path,
                              const std::string &attrName,
                              const std::string &value) = 0;

  virtual void writeAttribute(const std::string &path,
                              const std::string &attrName,
                              int value) = 0;

  virtual void writeAttribute(const std::string &path,
                              const std::string &attrName,
                              double value) = 0;

  virtual void createGroup(const std::string &groupPath) = 0;
};

std::unique_ptr<IDataWriter> createDataWriter(DataFormat format);
DataFormat parseDataFormat(const std::string &formatStr);
DataFormat getDefaultDataFormat();
std::string dataFormatToString(DataFormat format);

} // namespace mujoco
