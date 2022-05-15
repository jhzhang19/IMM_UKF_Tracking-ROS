#include"datasend.h"
#include"mot_tracking/detect.h"
#include"mot_tracking/object.h"
using namespace std;
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;



template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

template<typename T> 
string toString(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}

		
/**
 * 坐标系转换
*/
Eigen::Vector3d camera2cloud(Eigen::Vector3d input){
	Eigen::Matrix4d RT_velo_to_cam;
	Eigen::Matrix4d R_rect;
    	RT_velo_to_cam<<7.49916597e-03, -9.99971248e-01, -8.65110297e-04, -6.71807577e-03,
    			1.18652889e-02, 9.54520517e-04, -9.99910318e-01, -7.33152811e-02,
    			9.99882833e-01, 7.49141178e-03, 1.18719929e-02, -2.78557062e-01,
			0, 0, 0, 1;
    	R_rect<<0.99992475, 0.00975976, -0.00734152, 0,
    		-0.0097913, 0.99994262, -0.00430371, 0,
    		0.00729911, 0.0043753, 0.99996319, 0,
   		0, 0, 0, 1;

	Eigen::Vector4d point;
	point<<input(0), input(1), input(2), 1;
	Eigen::Vector4d pcloud = RT_velo_to_cam.inverse()*R_rect.inverse()* point;
	Eigen::Vector3d result;
	result<<pcloud(0),pcloud(1),pcloud(2);
	return result;
}
/**
 * 读取点云和图片数据
*/
int fileNameFilter(const struct dirent *cur) {
	std::string str(cur->d_name);
	if (str.find(".bin") != std::string::npos
			|| str.find(".pcd") != std::string::npos
			|| str.find(".png") != std::string::npos
			|| str.find(".jpg") != std::string::npos
			|| str.find(".txt") != std::string::npos) {
		return 1;
	}
	return 0;
}

bool get_all_files(const std::string& dir_in,
		std::vector<std::string>& files) {

	if (dir_in.empty()) {
		return false;
	}
	struct stat s;
	stat(dir_in.c_str(), &s);
	if (!S_ISDIR(s.st_mode)) {
		return false;
	}
	DIR* open_dir = opendir(dir_in.c_str());
	if (NULL == open_dir) {
		std::exit(EXIT_FAILURE);
	}
	dirent* p = nullptr;
	while ((p = readdir(open_dir)) != nullptr) {
		struct stat st;
		if (p->d_name[0] != '.') {
			//因为是使用devC++ 获取windows下的文件，所以使用了 "\" ,linux下要换成"/"
			//cout<<std::string(p->d_name)<<endl;
			std::string name = dir_in + std::string("/")
					+ std::string(p->d_name);
			stat(name.c_str(), &st);
			if (S_ISDIR(st.st_mode)) {
				get_all_files(name, files);
			} else if (S_ISREG(st.st_mode)) {
				boost::char_separator<char> sepp { "." };
				tokenizer tokn(std::string(p->d_name), sepp);
				vector<string> filename_sep(tokn.begin(), tokn.end());
				string type_ = "." + filename_sep[1];
				break;
			}
		}
	}

	struct dirent **namelist;
	int n = scandir(dir_in.c_str(), &namelist, fileNameFilter, alphasort);
	if (n < 0) {
		return false;
	}
	for (int i = 0; i < n; ++i) {
		std::string filePath(namelist[i]->d_name);
		files.push_back(filePath);
		free(namelist[i]);
	};
	free(namelist);
	closedir(open_dir);
	return true;
}

//根据提供的地址，最后返回点云和图片文件的文件名数组
bool DataSender::Load_Sensor_Data_Path(vector<std::string> &lidarname,
  vector<std::string> &imagename, string &path, string &data_idx){
    string lidar_file_path = path + "/velodyne/" + data_idx;
	// cout<<lidar_file_path<<endl;
	if(!get_all_files(lidar_file_path, lidarname))
		return false;
	string image_file_path = path + "/image_02/" + data_idx;
	// cout<<image_file_path<<endl;
	if(!get_all_files(image_file_path, imagename))
		return false;
	if(lidarname.size()!= imagename.size())
		return false;
	return true;
}

/**
 * 根据文件路径读取单帧图像和点云数据
*/
bool DataSender::load_img_pc_data(string &pc_path, string &img_path){
  //加载存储点云数据
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->points.clear();//先清空容器
  string lidar_filename_path = pc_path;
	ifstream inputfile;
	inputfile.open(lidar_filename_path, ios::binary);
	if (!inputfile) {
		cerr << "ERROR: Cannot open file " << lidar_filename_path
					<< "! Aborting..." << endl;
		return false;
	}
  
	inputfile.seekg(0, ios::beg);
	for (int i = 0; inputfile.good() && !inputfile.eof(); i++) {
		pcl::PointXYZI data;
		inputfile.read(reinterpret_cast<char*>(&data), sizeof(data));
		pcl::PointXYZI p;
		p.x = data.x;
		p.y = data.y;
		p.z = data.z;
		p.intensity = data.intensity;
		// cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<endl;
    cloud->points.emplace_back(p);
		
	}
  cloud->height = 1;
  cloud->width = cloud->points.size();

  // cout<<"pointcloud size"<<cloud->size()<<endl;
  

  //加载图片数据
  img = cv::imread(img_path);
  // cout<<"image size"<<img.size<<endl;

	return true;
}

/**
 * 加载全部gps数据并对gps数据进行处理
*/
bool DataSender::load_process_gps(string &path){
  //read gps data get the longitude latitude
	std::string gpspath = path;
	// cout<<gpspath<<endl;
	std::ifstream gps(gpspath);
	if (gps) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << gps.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		gpsdata = lines;
    cout<<"gps data load done"<<endl;
  }
  else{
    cerr<<"gps data load error"<<endl;
  }
  return true;
}

int DataSender::load_process_label(string &path, int lidarname_size){
  boost::char_separator<char> sep { " " };
	int maxframe = 0;

	// read the label file
	std::string labelpath = path;
  // std::string labelpath = "/home/zjh/workspace/kitti_track_dataset/tracking0020/label_02/0020.txt";
	// cout<<labelpath<<endl;
	std::ifstream label(labelpath);
  
	if (label) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << label.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		labeldata = lines;
		int size = labeldata.size();
		
    //使用ｔｅｍｐ_sep来读取最后一行获取最后一帧的序号
		tokenizer tokn(labeldata[size-1], sep);
		vector<string> temp_sep(tokn.begin(), tokn.end());
		maxframe = stringToNum<int>(temp_sep[0]);
		if((maxframe+1) != lidarname_size){
      cout<<maxframe<<" "<<lidarname_size<<endl;
			cout<<"file number wrong!"<<endl;
			std::abort();
		}
    // cout<<"maxframe:"<<maxframe<<"\n"<<"lidarframe:"<<lidarname_size<<endl;
	}
  else{
    cout<<"label file path is wrong!"<<endl;
    std::abort();
  }
  //　生成检测目标
  for(int i=0; i<labeldata.size(); ++i){
		tokenizer tokn(labeldata[i], sep);
		vector<string> temp_sep(tokn.begin(), tokn.end());
		int fra        =  stringToNum<int>(temp_sep[0]);    //frame
		string type    = temp_sep[2];                       //class
		int truncation = stringToNum<int>(temp_sep[3]);     //truncation
		int occlusion  = stringToNum<int>(temp_sep[4]);     //occlusion
		float x1       = stringToNum<float>(temp_sep[6]);   //left[pixel]
		float y1       = stringToNum<float>(temp_sep[7]);   //top[pixel]
		float x2       = stringToNum<float>(temp_sep[8]);   //right[pixel]
		float y2       = stringToNum<float>(temp_sep[9]);   //bottom[pixel]
		float h        = stringToNum<float>(temp_sep[10]);  //h[m]
		float w        = stringToNum<float>(temp_sep[11]);  //w[m]
		float l        = stringToNum<float>(temp_sep[12]);  //l[m]
		float x        = stringToNum<float>(temp_sep[13]);  //x[m]
		float y        = stringToNum<float>(temp_sep[14]);  //y[m]
		float z        = stringToNum<float>(temp_sep[15]);  //z[m]
		float yaw      = stringToNum<float>(temp_sep[16]);  //yaw angle[rad]
    //按照截断系数和遮挡系数以及类别筛选标签信息
		if(truncation>max_truncation || occlusion>max_occlusion || classname2id[type] != classname2id[trackclass])
			continue;

		Eigen::Vector3d cpoint;
		cpoint<<x,y,z;
		//cout<<"came " <<cpoint<<"\n"<<endl;

    //把中心点坐标反投到激光雷达坐标系下
    //因为标注信息是在ｃａｍｅｒａ０坐标系下的
		Eigen::Vector3d ppoint = camera2cloud(cpoint);
		//cout<<"cloud: "<< ppoint<<"\n"<<endl;
		
    //把标注目标构建成检测目标
		Detect_ det;
		det.box2D.resize(4);
		det.box.resize(3);
		det.classname  = classname2id[type];
		det.box2D[0]   = x1;
		det.box2D[1]   = y1;
		det.box2D[2]   = x2;
		det.box2D[3]   = y2;
		det.box[0]     = h;//h
		det.box[1]     = w;//w
		det.box[2]     = l;//l
		det.z          = ppoint(2);
		det.yaw        = yaw;
		det.position   = Eigen::VectorXd(2);
		det.position << ppoint(0),ppoint(1);

    //检测帧哈希表
		Inputdets[fra].push_back(det);

	}
  return maxframe;
}

void DataSender::generate_message(int frame, string &data_idx){
  //１．根据加载图片数据、点云数据
  //2.处理ＧＰＳ数据，生成ＵＴＭ和heading发送出去
  //3.处理label数据，生成每一帧的目标，并赋值给ｍｅｓｓａｇｅ
  //message
  mot_tracking::detect frame_data;
  mot_tracking::object object;
  object.box2d.points.resize(2);
  
  //单帧读取点云和图片
  string imagepath = datapath + "/image_02/" + data_idx + "/" + imgname[frame];
  string lidarpath = datapath + "/velodyne/" + data_idx + "/" + lidarname[frame];
  if(load_img_pc_data(lidarpath, imagepath)){
    cout<<"load pc and image data done"<<endl;
    // cout<<"cloud width and height:"<<cloud->width<<" "<<cloud->height<<endl;
  }
  else{
    cerr<<"lidar and image data read error"<<endl;
    std::abort();
  }
  //img and pc to sensor_img
  // cout<<"img size:"<<img.size<<endl;
  // cout<<"pc cloud:"<<cloud->width<<endl;
  frame_data.image = *(cv_bridge::CvImage(std_msgs::Header(),"bgr8", img).toImageMsg());
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  frame_data.pointcloud = ros_cloud;

  //获取gps经纬度、heading
  boost::char_separator<char> sep { " " };
  tokenizer tokn(gpsdata[frame], sep);
  vector<string> temp_sep(tokn.begin(), tokn.end());
  double latitude = stringToNum<double>(temp_sep[0]);
  double longitude = stringToNum<double>(temp_sep[1]);
  double heading = stringToNum<double>(temp_sep[5]) - 90 * M_PI/180;
  frame_data.GPS.x = latitude;
  frame_data.GPS.y = longitude;
  frame_data.GPS.z = heading;
  // cout<<"Inputdets"<<Inputdets[0][0].box2D[0]<<endl;
  //object data
  for(auto ob:Inputdets[frame]){
    object.box2d.points.resize(2);
    object.header.frame_id = "global_init_frame";
    object.header.stamp = ros::Time::now();
    object.type = ob.classname;
    object.score = ob.score;
    object.z = ob.z;
    object.yaw = ob.yaw;
    object.position.x = ob.position[0];//３ｄ中心坐标
    object.position.y = ob.position[1];
    object.box3d.x = ob.box[0];//h
    object.box3d.y = ob.box[1];//w
    object.box3d.z = ob.box[2];//l
    object.box2d.points[0].x = ob.box2D[0];//x1
    object.box2d.points[0].y = ob.box2D[1];//y1
    object.box2d.points[1].x = ob.box2D[2];//x2
    object.box2d.points[1].y = ob.box2D[3];//y2 

    frame_data.objects.push_back(object);
  }
  frame_data.header.frame_id = "global_init_frame";
  frame_data.header.stamp = ros::Time::now();

  detect_pub.publish(frame_data);
  sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",img).toImageMsg(); 
  pubimage.publish(imgmsg);
  
}

DataSender::DataSender(pcl::PointCloud<pcl::PointXYZI>::Ptr pc):cloud(pc){
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  classname2id["Car"] = 1;
	classname2id["Pedestrian"] = 2;	
	classname2id["Cyclist"] = 3;
  
  //data publisher
  detect_pub = nh.advertise<mot_tracking::detect>("/objects", 1);
  trackclass = "Car";
  pubimage = it.advertise("/send_data/image", 1);
  //截断系数和遮挡难度设置
	max_truncation = 0;
	max_occlusion = 2;
  
  string data_idx;
  string label_path,gps_path;
  ros::param::get("/data_send_node/data_idx", data_idx);
  ros::param::get("/data_send_node/datapath",datapath);
  ros::param::get("/data_send_node/labelpath", label_path);
  ros::param::get("/data_send_node/gpspath", gps_path);
  // datapath = "/home/zjh/workspace/kitti_track_dataset/training";
  //load img and pc dataname
  Load_Sensor_Data_Path(lidarname, imgname, datapath, data_idx);
  // string label_path = "/home/zjh/workspace/ROS_MOT/data/result.txt";
  // string gps_path = "/home/zjh/workspace/kitti_track_dataset/training/oxts/0000.txt";
  int maxframe = load_process_label(label_path, lidarname.size());
  load_process_gps(gps_path);

  ros::Rate rate(10);
  int frame = 0;

  while(ros::ok()&&frame<maxframe){
    generate_message(frame, data_idx);
    // cout<<"send data frame:"<<frame<<endl;
    frame++;
    rate.sleep();
  }


  

}
