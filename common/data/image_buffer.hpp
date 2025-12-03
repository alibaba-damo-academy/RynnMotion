#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace data {

struct ImageFrame {
  std::string cameraName;
  int width{0};
  int height{0};
  int channels{3};
  std::vector<uint8_t> data;
  double timestamp{0.0};

  ImageFrame() = default;

  ImageFrame(const std::string &name, int w, int h, int c = 3)
      : cameraName(name), width(w), height(h), channels(c) {
    data.resize(size());
  }

  size_t size() const {
    return static_cast<size_t>(width) * height * channels;
  }

  void allocate(int w, int h, int c = 3) {
    width = w;
    height = h;
    channels = c;
    data.resize(size());
  }

  void clear() {
    data.clear();
    width = 0;
    height = 0;
  }

  bool empty() const {
    return data.empty();
  }

  uint8_t *ptr() {
    return data.data();
  }

  const uint8_t *ptr() const {
    return data.data();
  }
};

struct CameraConfig {
  std::string name;
  int width{640};
  int height{480};
  int mjCamId{-1};

  CameraConfig() = default;

  CameraConfig(const std::string &name, int camId, int w = 640, int h = 480)
      : name(name), width(w), height(h), mjCamId(camId) {}
};

} // namespace data
