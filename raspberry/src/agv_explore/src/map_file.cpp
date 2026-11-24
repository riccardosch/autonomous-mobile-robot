#include "agv_explore/map_file.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace agv_explore {

bool write_map_files(const Grid &map, const std::string &path) {
  const auto dir = std::filesystem::path(path).parent_path();
  if (!dir.empty()) std::filesystem::create_directories(dir);

  std::ofstream pgm(path + ".pgm", std::ios::binary);
  pgm << "P5\n# agv_explore\n" << map.width << " " << map.height << "\n255\n";
  for (int r = map.height - 1; r >= 0; --r)  // image top row = largest y
    for (int c = 0; c < map.width; ++c) {
      const int8_t v = map.data[r * map.width + c];
      pgm.put(static_cast<char>(v < 0 ? 205 : v >= 65 ? 0 : v <= 25 ? 254 : 205));
    }

  std::ofstream yaml(path + ".yaml");
  yaml << "image: " << std::filesystem::path(path).filename().string() << ".pgm\n"
       << "mode: trinary\nresolution: " << map.resolution << "\n"
       << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0]\n"
       << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  return pgm.good() && yaml.good();
}

std::string expand_home(const std::string &path) {
  const char *home = std::getenv("HOME");
  if (path.rfind("~", 0) == 0 && home) return std::string(home) + path.substr(1);
  return path;
}

std::string default_map_name(std::time_t t) {
  char buf[32];
  std::strftime(buf, sizeof buf, "house_%Y%m%d_%H%M", std::localtime(&t));
  return buf;
}

}  // namespace agv_explore
