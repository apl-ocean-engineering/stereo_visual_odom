#include "stereo_visual_odom/configuration.h"

Configuration &Conf() {
  static Configuration TheInstance;
  return TheInstance;
}

Configuration::Configuration()
    : optical_flow_win_size(25), downsample(1), ORB_edge_threshold(31),
      ORB_patch_size(31), bucket_size(10), features_per_bucket(1),
      motion_threshold(1) {}
