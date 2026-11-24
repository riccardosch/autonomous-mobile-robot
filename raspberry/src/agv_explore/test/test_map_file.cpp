#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "agv_explore/map_file.hpp"
#include "test_grid.hpp"

using namespace agv_explore;

TEST(MapFile, WritesNav2TrinaryImageTopRowFirst) {
  Grid g = grid({
    "#.",    // top row (largest y)
    ".?",
  });
  g.resolution = 0.05;
  g.origin_x = -1.0;
  g.origin_y = 2.0;
  const auto dir = std::filesystem::temp_directory_path() / "agv_explore_test";
  ASSERT_TRUE(write_map_files(g, (dir / "m").string()));

  std::ifstream pgm(dir / "m.pgm", std::ios::binary);
  std::string magic, comment;
  int w, h, maxval;
  pgm >> magic;
  pgm.ignore();
  std::getline(pgm, comment);
  pgm >> w >> h >> maxval;
  pgm.get();
  unsigned char px[4];
  pgm.read(reinterpret_cast<char *>(px), 4);
  EXPECT_EQ(magic, "P5");
  EXPECT_EQ(w, 2);
  EXPECT_EQ(h, 2);
  EXPECT_EQ(px[0], 0);     // wall
  EXPECT_EQ(px[1], 254);   // free
  EXPECT_EQ(px[2], 254);
  EXPECT_EQ(px[3], 205);   // unknown

  std::ifstream yaml(dir / "m.yaml");
  std::stringstream text;
  text << yaml.rdbuf();
  EXPECT_NE(text.str().find("image: m.pgm"), std::string::npos);
  EXPECT_NE(text.str().find("resolution: 0.05"), std::string::npos);
  EXPECT_NE(text.str().find("origin: [-1, 2, 0]"), std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST(MapFile, DefaultNameHasDateAndTime) {
  EXPECT_EQ(default_map_name(0).rfind("house_", 0), 0u);
  EXPECT_EQ(default_map_name(0).size(), std::string("house_YYYYMMDD_HHMM").size());
}
