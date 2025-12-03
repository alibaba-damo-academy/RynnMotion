#pragma once

#include <Eigen/Dense>
#include <H5Cpp.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace mujoco {

/**
 * @class HDF5Writer
 * @brief Writes episode data to HDF5 files for dataset recording
 */
class HDF5Writer {
public:
  HDF5Writer();
  ~HDF5Writer();

  HDF5Writer(const HDF5Writer &) = delete;
  HDF5Writer &operator=(const HDF5Writer &) = delete;

  void create(const std::filesystem::path &filePath);
  void close();
  bool isOpen() const;

  void writeDataset(const std::string &name,
                    const std::vector<double> &data,
                    const std::vector<hsize_t> &shape);

  void writeDataset(const std::string &name,
                    const std::vector<float> &data,
                    const std::vector<hsize_t> &shape);

  void writeDataset(const std::string &name,
                    const std::vector<int64_t> &data,
                    const std::vector<hsize_t> &shape);

  void writeEigenVectors(const std::string &name,
                         const std::vector<Eigen::VectorXd> &frames);

  void writeAttribute(const std::string &path,
                      const std::string &attrName,
                      const std::string &value);

  void writeAttribute(const std::string &path,
                      const std::string &attrName,
                      int value);

  void writeAttribute(const std::string &path,
                      const std::string &attrName,
                      double value);

  void createGroup(const std::string &groupPath);

private:
  void ensureParentGroupExists(const std::string &path);

  std::unique_ptr<H5::H5File> file_;
};

} // namespace mujoco
