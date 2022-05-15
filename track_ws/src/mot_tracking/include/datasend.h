#ifndef DATASEND_H
#define DATASEND_H

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
// #include "lonlat2utm.h"

// #include "readparam.h"
// #include "tracker.h"

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

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

struct Detect_
{
  int classname = 0;
  float score; //detection score
  float z;
  float yaw;
  Eigen::VectorXd position; //x,y
  std::vector<float> box;   //3D box in lidar h、w、l
  std::vector<float> box2D; //2D box in camera left top point and right down point
  cv::RotatedRect rotbox;
};
using namespace std;
class DataSender
{
public:
  DataSender(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
  bool Load_Sensor_Data_Path(vector<std::string> &lidarname,
                             vector<std::string> &imagename, string &path, string &data_idx);
  bool load_img_pc_data(string &pc_path, string &img_path);
  bool load_process_gps(string &path);
  int load_process_label(string &path, int lidarname_size);
  void generate_message(int frame, string &data_idx); //danz

  ~DataSender() {}

private:
  //img pointcloud
  cv::Mat img;
  image_transport::Publisher pubimage;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  double UTME, UTMN;
  vector<string> labeldata;
  vector<string> gpsdata;
  vector<string> lidarname;
  vector<string> imgname;
  string datapath; //存储数据的地址，到velodyne的上一级
  unordered_map<int, vector<Detect_>> Inputdets;
  //截断系数和遮挡难度设置
  int max_truncation;
  int max_occlusion;
  string trackclass;
  unordered_map<string, int> classname2id;
  //topic publisher
  ros::Publisher detect_pub;
};

#endif