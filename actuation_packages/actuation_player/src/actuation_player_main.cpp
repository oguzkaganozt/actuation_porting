// Copyright (c) 2023, Arm Limited.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
//

#include <actuation_player/actuation_player.hpp>

#include <getopt.h>
#include <unistd.h>

#include <string>

namespace
{
/// @brief Process parameters. Exit on error.
/// @param argc Number of arguments
/// @param argv Arguments
/// @param[out] path_prefix Path to the directory containing the recordings
/// @param[out] run_loop Whether the replay runs in a loop or not
/// @param[out] replay_divider Divider of the replay speed
void process_parameters(
  int argc, char * const * argv, const char * & path_prefix, bool & run_loop,
  float & replay_divider)
{
  const char optstring[] = "hld:p:";
  const struct option longopts[] = {
    {"help", no_argument, NULL, 'h'},
    {"loop", no_argument, NULL, 'l'},
    {"divider", required_argument, NULL, 'd'},
    {"path", required_argument, NULL, 'p'},
    {0, 0, 0, 0}
  };

  int c;
  while ((c = getopt_long(argc, argv, optstring, longopts, NULL)) != -1) {
    switch (c) {
      case 'h':
        std::printf(
          "Usage: %s [OPTIONS]\n"
          "    -h, --help            Print this help message and exit\n"
          "    -l, --loop            Run in a loop\n"
          "    -d DIV, --divider DIV Divide the replay speed (floating point value)\n"
          "    -p PATH, --path PATH  Path to the directory containing the recordings\n",
          argv[0]);
        std::exit(0);
        break;
      case 'l':
        run_loop = true;
        break;
      case 'd':
        replay_divider = std::atof(optarg);
        break;
      case 'p':
        path_prefix = optarg;
        break;
      default:
        std::fprintf(stderr, "Unexpected arguments\n");
        std::exit(1);
    }
  }

  if (optind > argc) {
    std::fprintf(stderr, "Unexpected arguments\n");
    std::exit(1);
  }
}

/// @brief Get the path to the expected default location for the data files. Exit on error.
/// @param[out] default_path_prefix Path to be filled
void get_default_data_path(std::string & default_path_prefix)
{
  char buf[256];
  ssize_t ret = readlink("/proc/self/exe", buf, sizeof(buf));
  if (ret == -1) {
    std::fprintf(stderr, "readlink error\n");
    std::exit(1);
  } else if (ret == sizeof(buf)) {
    std::fprintf(stderr, "readlink overflow\n");
    std::exit(1);
  }
  buf[ret] = '\0';

  default_path_prefix = buf;
  size_t bin_dir = default_path_prefix.find_last_of('/');
  const char data_path[] = "../share/actuation_player";
  default_path_prefix.replace(bin_dir + 1, default_path_prefix.size() - (bin_dir + 1), data_path);
}

/// @brief Set the CYCLONEDDS_URI environment variable to the expected default location of the XML
///        file. Don't overwrite its value if it already exists in the environment. Exit on error.
/// @param path Path to the data directory
void set_dds_config_path(std::string path)
{
  path.append("/cyclonedds.xml");
  if (setenv("CYCLONEDDS_URI", path.data(), 0) == -1) {
    std::fprintf(stderr, "setenv error\n");
    std::exit(1);
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  std::string default_path_prefix;
  get_default_data_path(default_path_prefix);

  set_dds_config_path(default_path_prefix);

  const char * path_prefix = default_path_prefix.data();
  bool run_loop = false;
  float replay_divider = 1.0;
  process_parameters(argc, argv, path_prefix, run_loop, replay_divider);

  return actuation_player(path_prefix, run_loop, replay_divider);
}
