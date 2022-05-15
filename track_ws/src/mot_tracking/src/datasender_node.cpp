#include"datasend.h"
// #include"track_process.h"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "send_data");
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
  DataSender datasender(pc);
  // track_processor tracker;
  return 0;
}