#ifdef RYNN_USE_PARQUET

#include "mj_parquet_writer.hpp"

#include <arrow/builder.h>
#include <parquet/arrow/schema.h>

namespace mujoco {

ParquetWriter::ParquetWriter() = default;

ParquetWriter::~ParquetWriter() {
  close();
}

void ParquetWriter::create(const std::filesystem::path &filePath) {
  close();
  std::filesystem::create_directories(filePath.parent_path());
  filePath_ = filePath;
  isOpen_ = true;
  columns_.clear();
  metadata_.clear();
}

void ParquetWriter::close() {
  if (isOpen_) {
    flushToFile();
    isOpen_ = false;
  }
}

bool ParquetWriter::isOpen() const {
  return isOpen_;
}

std::string ParquetWriter::getExtension() const {
  return ".parquet";
}

void ParquetWriter::writeDataset(const std::string &name,
                                 const std::vector<double> &data,
                                 const std::vector<size_t> &shape) {
  if (!isOpen_ || data.empty()) return;

  std::string cleanName = name;
  std::replace(cleanName.begin(), cleanName.end(), '/', '.');

  if (shape.size() == 1) {
    arrow::DoubleBuilder builder;
    auto status = builder.AppendValues(data);
    if (!status.ok()) return;

    std::shared_ptr<arrow::Array> array;
    status = builder.Finish(&array);
    if (!status.ok()) return;

    auto field = arrow::field(cleanName, arrow::float64());
    columns_.push_back({cleanName, array, field});

  } else if (shape.size() == 2) {
    size_t numRows = shape[0];
    size_t listSize = shape[1];

    auto valueBuilder = std::make_shared<arrow::DoubleBuilder>();
    arrow::FixedSizeListBuilder listBuilder(arrow::default_memory_pool(),
                                            valueBuilder,
                                            static_cast<int32_t>(listSize));

    for (size_t i = 0; i < numRows; ++i) {
      auto status = listBuilder.Append();
      if (!status.ok()) return;
      status = valueBuilder->AppendValues(data.data() + i * listSize, listSize);
      if (!status.ok()) return;
    }

    std::shared_ptr<arrow::Array> array;
    auto status = listBuilder.Finish(&array);
    if (!status.ok()) return;

    auto listType = arrow::fixed_size_list(arrow::float64(), static_cast<int32_t>(listSize));
    auto field = arrow::field(cleanName, listType);
    columns_.push_back({cleanName, array, field});
  }
}

void ParquetWriter::writeDataset(const std::string &name,
                                 const std::vector<float> &data,
                                 const std::vector<size_t> &shape) {
  if (!isOpen_ || data.empty()) return;

  std::string cleanName = name;
  std::replace(cleanName.begin(), cleanName.end(), '/', '.');

  if (shape.size() == 1) {
    arrow::FloatBuilder builder;
    auto status = builder.AppendValues(data);
    if (!status.ok()) return;

    std::shared_ptr<arrow::Array> array;
    status = builder.Finish(&array);
    if (!status.ok()) return;

    auto field = arrow::field(cleanName, arrow::float32());
    columns_.push_back({cleanName, array, field});

  } else if (shape.size() == 2) {
    size_t numRows = shape[0];
    size_t listSize = shape[1];

    auto valueBuilder = std::make_shared<arrow::FloatBuilder>();
    arrow::FixedSizeListBuilder listBuilder(arrow::default_memory_pool(),
                                            valueBuilder,
                                            static_cast<int32_t>(listSize));

    for (size_t i = 0; i < numRows; ++i) {
      auto status = listBuilder.Append();
      if (!status.ok()) return;
      status = valueBuilder->AppendValues(data.data() + i * listSize, listSize);
      if (!status.ok()) return;
    }

    std::shared_ptr<arrow::Array> array;
    auto status = listBuilder.Finish(&array);
    if (!status.ok()) return;

    auto listType = arrow::fixed_size_list(arrow::float32(), static_cast<int32_t>(listSize));
    auto field = arrow::field(cleanName, listType);
    columns_.push_back({cleanName, array, field});
  }
}

void ParquetWriter::writeDataset(const std::string &name,
                                 const std::vector<int64_t> &data,
                                 const std::vector<size_t> &shape) {
  if (!isOpen_ || data.empty()) return;

  std::string cleanName = name;
  std::replace(cleanName.begin(), cleanName.end(), '/', '.');

  if (shape.size() == 1) {
    arrow::Int64Builder builder;
    auto status = builder.AppendValues(data);
    if (!status.ok()) return;

    std::shared_ptr<arrow::Array> array;
    status = builder.Finish(&array);
    if (!status.ok()) return;

    auto field = arrow::field(cleanName, arrow::int64());
    columns_.push_back({cleanName, array, field});

  } else if (shape.size() == 2) {
    size_t numRows = shape[0];
    size_t listSize = shape[1];

    auto valueBuilder = std::make_shared<arrow::Int64Builder>();
    arrow::FixedSizeListBuilder listBuilder(arrow::default_memory_pool(),
                                            valueBuilder,
                                            static_cast<int32_t>(listSize));

    for (size_t i = 0; i < numRows; ++i) {
      auto status = listBuilder.Append();
      if (!status.ok()) return;
      status = valueBuilder->AppendValues(data.data() + i * listSize, listSize);
      if (!status.ok()) return;
    }

    std::shared_ptr<arrow::Array> array;
    auto status = listBuilder.Finish(&array);
    if (!status.ok()) return;

    auto listType = arrow::fixed_size_list(arrow::int64(), static_cast<int32_t>(listSize));
    auto field = arrow::field(cleanName, listType);
    columns_.push_back({cleanName, array, field});
  }
}

void ParquetWriter::writeEigenVectors(const std::string &name,
                                      const std::vector<Eigen::VectorXd> &frames) {
  if (!isOpen_ || frames.empty()) return;

  size_t numFrames = frames.size();
  size_t vecSize = frames[0].size();

  std::vector<double> flatData;
  flatData.reserve(numFrames * vecSize);
  for (const auto &vec : frames) {
    for (int i = 0; i < vec.size(); ++i) {
      flatData.push_back(vec(i));
    }
  }

  std::vector<size_t> shape = {numFrames, vecSize};
  writeDataset(name, flatData, shape);
}

void ParquetWriter::writeAttribute(const std::string &path,
                                   const std::string &attrName,
                                   const std::string &value) {
  std::string key = path + "." + attrName;
  metadata_[key] = value;
}

void ParquetWriter::writeAttribute(const std::string &path,
                                   const std::string &attrName,
                                   int value) {
  std::string key = path + "." + attrName;
  metadata_[key] = std::to_string(value);
}

void ParquetWriter::writeAttribute(const std::string &path,
                                   const std::string &attrName,
                                   double value) {
  std::string key = path + "." + attrName;
  metadata_[key] = std::to_string(value);
}

void ParquetWriter::createGroup(const std::string &) {
  // Parquet doesn't have groups, this is a no-op
}

void ParquetWriter::flushToFile() {
  if (columns_.empty()) return;

  std::vector<std::shared_ptr<arrow::Field>> fields;
  std::vector<std::shared_ptr<arrow::Array>> arrays;

  for (const auto &col : columns_) {
    fields.push_back(col.field);
    arrays.push_back(col.array);
  }

  auto schema = arrow::schema(fields);
  auto table = arrow::Table::Make(schema, arrays);

  auto outfile = arrow::io::FileOutputStream::Open(filePath_.string());
  if (!outfile.ok()) {
    std::cerr << "[ParquetWriter] Failed to open file: " << filePath_ << std::endl;
    return;
  }

  parquet::WriterProperties::Builder propBuilder;
  propBuilder.compression(parquet::Compression::SNAPPY);

  auto result = parquet::arrow::WriteTable(
      *table,
      arrow::default_memory_pool(),
      *outfile,
      /*chunk_size=*/1024,
      propBuilder.build());

  if (!result.ok()) {
    std::cerr << "[ParquetWriter] Failed to write table: " << result.ToString() << std::endl;
  }

  columns_.clear();
}

} // namespace mujoco

#endif // RYNN_USE_PARQUET
