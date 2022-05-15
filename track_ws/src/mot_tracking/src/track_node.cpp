
#include"track_process.h"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "tracking");
  // //初始化跟踪器参数
  // Param param;
  // //初始化跟踪器
  // Tracker tracker_(param);
  //利用初始化的跟踪器去初始化类中的跟踪器
  track_processor tracker;
  
  return 0;
}