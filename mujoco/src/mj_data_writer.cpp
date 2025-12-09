#include "mj_data_writer.hpp"

#ifdef RYNN_USE_PARQUET
#include "mj_parquet_writer.hpp"
#endif
#ifdef RYNN_USE_HDF5
#include "mj_hdf5_writer.hpp"
#endif

namespace mujoco {

class NullWriter : public IDataWriter {
public:
  void create(const std::filesystem::path &) override {}
  void close() override {}
  bool isOpen() const override { return false; }
  std::string getExtension() const override { return ""; }

  void writeDataset(const std::string &, const std::vector<double> &,
                    const std::vector<size_t> &) override {}
  void writeDataset(const std::string &, const std::vector<float> &,
                    const std::vector<size_t> &) override {}
  void writeDataset(const std::string &, const std::vector<int64_t> &,
                    const std::vector<size_t> &) override {}
  void writeEigenVectors(const std::string &,
                         const std::vector<Eigen::VectorXd> &) override {}

  void writeAttribute(const std::string &, const std::string &,
                      const std::string &) override {}
  void writeAttribute(const std::string &, const std::string &, int) override {}
  void writeAttribute(const std::string &, const std::string &, double) override {}

  void createGroup(const std::string &) override {}
};

DataFormat getDefaultDataFormat() {
#ifdef RYNN_USE_PARQUET
  return DataFormat::Parquet;
#elif defined(RYNN_USE_HDF5)
  return DataFormat::HDF5;
#else
  return DataFormat::None;
#endif
}

std::unique_ptr<IDataWriter> createDataWriter(DataFormat format) {
  switch (format) {
    case DataFormat::Parquet:
#ifdef RYNN_USE_PARQUET
      return std::make_unique<ParquetWriter>();
#else
      std::cerr << "[MujocoRecorder] Parquet not compiled. Use -DRYNN_USE_PARQUET=ON\n";
      return std::make_unique<NullWriter>();
#endif
    case DataFormat::HDF5:
#ifdef RYNN_USE_HDF5
      return std::make_unique<HDF5Writer>();
#else
      std::cerr << "[MujocoRecorder] HDF5 not compiled. Use -DRYNN_USE_HDF5=ON\n";
      return std::make_unique<NullWriter>();
#endif
    case DataFormat::None:
    default:
      return std::make_unique<NullWriter>();
  }
}

DataFormat parseDataFormat(const std::string &s) {
  if (s == "parquet") return DataFormat::Parquet;
  if (s == "hdf5") return DataFormat::HDF5;
  if (s == "none") return DataFormat::None;
  if (s == "auto") return getDefaultDataFormat();
  return getDefaultDataFormat();
}

std::string dataFormatToString(DataFormat f) {
  switch (f) {
    case DataFormat::Parquet: return "parquet";
    case DataFormat::HDF5: return "hdf5";
    default: return "none";
  }
}

} // namespace mujoco
