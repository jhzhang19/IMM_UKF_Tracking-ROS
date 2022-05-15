#ifndef TRACK_PROCESS_H
#define TRACK_PROCESS_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <time.h>
#include <unordered_map>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <memory>

//Txt
#include <iomanip>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include "lonlat2utm.h"

#include "readparam.h"
#include "tracker.h"
#include "mot_tracking/detect.h"
#include "mot_tracking/object.h"

//Ros
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

class track_processor
{
public:
  track_processor();
  ~track_processor() {}

  void process(const mot_tracking::detect::ConstPtr &msg);

private:
  int frame;
  int max_marker_size_;
  Eigen::Matrix3d rotZorigion;
  Eigen::Isometry3d porigion;
  double oriheading;
  double orix;
  double oriy;

  Eigen::Matrix3d rotZpre;
  Eigen::Isometry3d ppre;
  double preheading;
  double prex;
  double prey;

  float time;
  double totaltime;

  ofstream out_txt_file;

  cv::Mat images;
  image_transport::Publisher pubimage;
  ros::Publisher publidar;
  ros::Publisher pubmarker;
  ros::Publisher pubtextmarker;
  ros::Subscriber datasub;
  Param param;
  std::shared_ptr<Tracker> tracker_;
  cv::RNG rng;
  unordered_map<int, vector<int>> idcolor;
};

#endif