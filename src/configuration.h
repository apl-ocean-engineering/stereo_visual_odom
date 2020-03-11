#include <string>

#pragma once


class Configuration;

Configuration &Conf();

class Configuration {
public:
  friend Configuration &Conf();



  int optical_flow_win_size; //Square kernel, odd
  int downsample;
  int ORB_edge_threshold;
  int ORB_patch_size;
  int bucket_size;
  int features_per_bucket;



private:
  Configuration();
};
