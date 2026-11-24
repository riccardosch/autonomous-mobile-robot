// Writes a map as image + metadata, the format of Nav2's map_saver, without ROS.
#pragma once

#include <ctime>
#include <string>

#include "agv_explore/grid.hpp"

namespace agv_explore {

// <path>.pgm (trinary: free 254, occupied 0, unknown 205; top row = largest y) and
// <path>.yaml (image, resolution, origin, thresholds). Creates the directory.
// Returns false if a file could not be written.
bool write_map_files(const Grid &map, const std::string &path);

// "~/maps" -> "/home/<user>/maps".
std::string expand_home(const std::string &path);

// "house_YYYYMMDD_HHMM" for the given time (local time zone).
std::string default_map_name(std::time_t t);

}  // namespace agv_explore
