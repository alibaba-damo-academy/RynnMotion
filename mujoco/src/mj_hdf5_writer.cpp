#include "mj_hdf5_writer.hpp"

#include <algorithm>
#include <sstream>

namespace mujoco {

HDF5Writer::HDF5Writer() = default;

HDF5Writer::~HDF5Writer() {
  close();
}

void HDF5Writer::create(const std::filesystem::path &filePath) {
  close();
  std::filesystem::create_directories(filePath.parent_path());
  file_ = std::make_unique<H5::H5File>(filePath.string(), H5F_ACC_TRUNC);
}

void HDF5Writer::close() {
  if (file_) {
    file_->close();
    file_.reset();
  }
}

bool HDF5Writer::isOpen() const {
  return file_ != nullptr;
}

void HDF5Writer::writeDataset(const std::string &name,
                              const std::vector<double> &data,
                              const std::vector<hsize_t> &shape) {
  if (!file_) return;

  ensureParentGroupExists(name);
  H5::DataSpace dataspace(static_cast<int>(shape.size()), shape.data());
  H5::DataSet dataset = file_->createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
  dataset.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

void HDF5Writer::writeDataset(const std::string &name,
                              const std::vector<float> &data,
                              const std::vector<hsize_t> &shape) {
  if (!file_) return;

  ensureParentGroupExists(name);
  H5::DataSpace dataspace(static_cast<int>(shape.size()), shape.data());
  H5::DataSet dataset = file_->createDataSet(name, H5::PredType::NATIVE_FLOAT, dataspace);
  dataset.write(data.data(), H5::PredType::NATIVE_FLOAT);
}

void HDF5Writer::writeDataset(const std::string &name,
                              const std::vector<int64_t> &data,
                              const std::vector<hsize_t> &shape) {
  if (!file_) return;

  ensureParentGroupExists(name);
  H5::DataSpace dataspace(static_cast<int>(shape.size()), shape.data());
  H5::DataSet dataset = file_->createDataSet(name, H5::PredType::NATIVE_INT64, dataspace);
  dataset.write(data.data(), H5::PredType::NATIVE_INT64);
}

void HDF5Writer::writeEigenVectors(const std::string &name,
                                   const std::vector<Eigen::VectorXd> &frames) {
  if (!file_ || frames.empty()) return;

  hsize_t numFrames = frames.size();
  hsize_t vecSize = frames[0].size();

  std::vector<double> flatData;
  flatData.reserve(numFrames * vecSize);
  for (const auto &vec : frames) {
    for (int i = 0; i < vec.size(); ++i) {
      flatData.push_back(vec(i));
    }
  }

  std::vector<hsize_t> shape = {numFrames, vecSize};
  writeDataset(name, flatData, shape);
}

void HDF5Writer::writeAttribute(const std::string &path,
                                const std::string &attrName,
                                const std::string &value) {
  if (!file_) return;

  try {
    H5::DataSet dataset = file_->openDataSet(path);
    H5::StrType strType(H5::PredType::C_S1, value.size() + 1);
    H5::DataSpace attrSpace(H5S_SCALAR);
    H5::Attribute attr = dataset.createAttribute(attrName, strType, attrSpace);
    attr.write(strType, value);
  } catch (...) {
    try {
      H5::Group group = file_->openGroup(path);
      H5::StrType strType(H5::PredType::C_S1, value.size() + 1);
      H5::DataSpace attrSpace(H5S_SCALAR);
      H5::Attribute attr = group.createAttribute(attrName, strType, attrSpace);
      attr.write(strType, value);
    } catch (...) {
    }
  }
}

void HDF5Writer::writeAttribute(const std::string &path,
                                const std::string &attrName,
                                int value) {
  if (!file_) return;

  try {
    H5::DataSet dataset = file_->openDataSet(path);
    H5::DataSpace attrSpace(H5S_SCALAR);
    H5::Attribute attr = dataset.createAttribute(attrName, H5::PredType::NATIVE_INT, attrSpace);
    attr.write(H5::PredType::NATIVE_INT, &value);
  } catch (...) {
  }
}

void HDF5Writer::writeAttribute(const std::string &path,
                                const std::string &attrName,
                                double value) {
  if (!file_) return;

  try {
    H5::DataSet dataset = file_->openDataSet(path);
    H5::DataSpace attrSpace(H5S_SCALAR);
    H5::Attribute attr = dataset.createAttribute(attrName, H5::PredType::NATIVE_DOUBLE, attrSpace);
    attr.write(H5::PredType::NATIVE_DOUBLE, &value);
  } catch (...) {
  }
}

void HDF5Writer::createGroup(const std::string &groupPath) {
  if (!file_) return;

  std::istringstream ss(groupPath);
  std::string token;
  std::string currentPath;

  while (std::getline(ss, token, '/')) {
    if (token.empty()) continue;
    currentPath += "/" + token;
    try {
      file_->openGroup(currentPath);
    } catch (...) {
      file_->createGroup(currentPath);
    }
  }
}

void HDF5Writer::ensureParentGroupExists(const std::string &path) {
  auto lastSlash = path.rfind('/');
  if (lastSlash != std::string::npos && lastSlash > 0) {
    createGroup(path.substr(0, lastSlash));
  }
}

} // namespace mujoco
