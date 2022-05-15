#include"track_process.h"

using namespace std;

static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	// return ms
	int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

inline cv::Point cloud2camera(Eigen::Vector3d input){
	Eigen::Matrix4d RT_velo_to_cam;
	Eigen::Matrix4d R_rect;
	Eigen::MatrixXd project_matrix(3,4);
    	RT_velo_to_cam<<7.49916597e-03, -9.99971248e-01, -8.65110297e-04, -6.71807577e-03,
    			1.18652889e-02, 9.54520517e-04, -9.99910318e-01, -7.33152811e-02,
    			9.99882833e-01, 7.49141178e-03, 1.18719929e-02, -2.78557062e-01,
			0, 0, 0, 1;
    	R_rect<<0.99992475, 0.00975976, -0.00734152, 0,
    		-0.0097913, 0.99994262, -0.00430371, 0,
    		0.00729911, 0.0043753, 0.99996319, 0,
   		0, 0, 0, 1;
    	project_matrix<<7.215377e+02,0.000000e+00,6.095593e+02,4.485728e+01,
                    0.000000e+00,7.215377e+02,1.728540e+02,2.163791e-01,
                    0.000000e+00,0.000000e+00,1.000000e+00,2.745884e-03;
    	Eigen::MatrixXd transform_matrix_ = project_matrix*R_rect*RT_velo_to_cam;

	Eigen::Vector4d point;
	point<<input(0), input(1), input(2), 1;
	Eigen::Vector3d pimage = transform_matrix_* point;
	cv::Point p2d = cv::Point(pimage(0)/pimage(2),pimage(1)/pimage(2));
	return p2d;
}

inline Eigen::Vector3d camera2cloud(Eigen::Vector3d input){
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

inline void draw3dbox(Detect &det, cv::Mat& image, vector<int>& color){
	float h = det.box[0];	
	float w = det.box[1];
	float l = det.box[2];
  float x = det.position[0];
	float y = det.position[1];
	float z = det.z;
	float yaw = -det.yaw - 90 * M_PI/180;
	double boxroation[9] = {cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1 };

	Eigen::MatrixXd BoxRotation = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(boxroation);
	double xAxisP[8] = {l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2 }; 
	double yAxisP[8] = {w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2}; 
	double zAxisP[8] = {h, h, h, h, 0, 0, 0, 0}; 
	vector<cv::Point> imagepoint;
	Eigen::Vector3d translation(x,y,z);

	for (int i = 0; i < 8; i++) {
		Eigen::Vector3d point_3d(xAxisP[i], yAxisP[i], zAxisP[i]);//以３ｄ包围框中心为原点的局部坐标系坐标
		Eigen::Vector3d rotationPoint_3d = BoxRotation * point_3d + translation;//转换为激光雷达坐标系下的坐标
		cv::Point imgpoint = cloud2camera(rotationPoint_3d);
		imagepoint.push_back(imgpoint);
	}

	int r = color[0];
	int g = color[1];
	int b = color[2];

	cv::line(image, imagepoint[0], imagepoint[1],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[2],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[2], imagepoint[3],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[3], imagepoint[0],cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[4], imagepoint[5],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[5], imagepoint[6],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[6], imagepoint[7],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[7], imagepoint[4],cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[0], imagepoint[4],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[5],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[2], imagepoint[6],cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[3], imagepoint[7],cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[0], imagepoint[5],cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[4],cv::Scalar(226, 43, 138), 2, CV_AA);

	imagepoint.clear();

}
void track_processor::process(const mot_tracking::detect::ConstPtr &msg){

  //接收消息 解析消息
  //处理ｕｔｍ坐标
  //循环处理每一个目标的坐标
  //处理完成的目标存储到目标容器中
  //整一帧的目标输送给跟踪器进行跟踪

  // cout<<"msg image picture:"<<msg->image.height<<msg->image.width<<endl;
  // cout<<"msg pointcloud size"<<msg->pointcloud.width<<endl;
  
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception &e){
    return;
  }
  cv::Mat rgbimg = cv_ptr->image;

  // cout<<"rgb images:"<<rgbimg.size<<endl;
  // pcl::PointCloud<pcl::PointXYZI> pcloud;
  // pcl::fromROSMsg(msg->pointcloud, pcloud);

  double latitude = msg->GPS.x;
  double longtitude = msg->GPS.y;
  double heading = msg->GPS.z;
  double UTME, UTMN;
  LonLat2UTM(longtitude, latitude, UTME, UTMN);


  Eigen::Isometry3d translate2origion;
  Eigen::Isometry3d origion2translate;
  // double twosub=0;
  if(frame == 0){
      //记录初始时刻的坐标、旋转平移矩阵和朝向角
			Eigen::AngleAxisd roto(heading ,Eigen::Vector3d::UnitZ());
			rotZorigion = roto.toRotationMatrix();//初始时刻旋转矩阵
			porigion = rotZorigion;//初始时刻的旋转平移矩阵的旋转部分
			// orix = UTME;//初始坐标原点为第一帧的utm坐标
			// oriy = UTMN;
			porigion.translation() = Eigen::Vector3d(UTME, UTMN, 0);//平移部分
			// oriheading = heading;//初始时刻朝向角

      //记录上一时刻的坐标、旋转平移矩阵和朝向角
			// preheading = heading;
			// prex = UTME;
			// prey = UTMN;
			// rotZpre = rotZorigion;
			// ppre = rotZpre;
			// ppre.translation() = Eigen::Vector3d(UTME, UTMN, 0);
  }
  else{
    Eigen::AngleAxisd rotnow(heading , Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotpnow = rotnow.toRotationMatrix();//当前时刻的旋转矩阵
    Eigen::Isometry3d p2;
    p2 = rotpnow;
    p2.translation() = Eigen::Vector3d(UTME, UTMN, 0);//当前时刻的旋转平移矩阵
    translate2origion = porigion.inverse() * p2;
    origion2translate = p2.inverse() * porigion;
    // 两帧方向角之间的差异
    // twosub = heading - oriheading;
  }

  int size = msg->objects.size();
  vector<Detect> inputDet;
  for(int i=0;i<size;i++){
    Detect object;
    object.box2D.resize(4);
    object.box.resize(3);
    object.position.resize(2);
    object.classname = msg->objects[i].type;
    object.box2D[0] = msg->objects[i].box2d.points[0].x;
    object.box2D[1] = msg->objects[i].box2d.points[0].y;
    object.box2D[2] = msg->objects[i].box2d.points[1].x;
    object.box2D[3] = msg->objects[i].box2d.points[1].y;
    object.box[0] = msg->objects[i].box3d.x;
    object.box[1] = msg->objects[i].box3d.y;
    object.box[2] = msg->objects[i].box3d.z;
    object.z = msg->objects[i].z;
    object.yaw = msg->objects[i].yaw;
    object.position[0] = msg->objects[i].position.x;
    object.position[1] = msg->objects[i].position.y;

    vector<int> color = {0,255,0};
    draw3dbox(object, rgbimg, color);

    Eigen::VectorXd v(2,1);
    v(1)   = object.position(0);//x in kitti lidar
    v(0)   = -object.position(1);//y in kitti lidar
    if(frame!=0){
      Eigen::Vector3d p_0(v(0), v(1), 0);
      Eigen::Vector3d p_1;
      p_1 = translate2origion * p_0;//转换到utm全局坐标系下进行跟踪
      // p_1 = p_0;
      v(0) = p_1(0);
      v(1) = p_1(1);
    }

    object.position(0) = v(0);
    object.position(1) = v(1);

    object.rotbox = cv::RotatedRect(cv::Point2f((v(0)+25)*608/50, v(1)*608/50), 
            cv::Size2f(object.box[1]*608/50, object.box[2]*608/50), object.yaw);				     

    cv::RotatedRect detshow = cv::RotatedRect(cv::Point2f((v(0)+25)*608/50, v(1)*608/50), 
            cv::Size2f(object.box[1]*608/50, object.box[2]*608/50), object.yaw);
    cv::Point2f vertices[4];
    detshow.points(vertices);
    for (int j = 0; j < 4; j++){
      cv::line(images, vertices[j], vertices[(j+1)%4], cv::Scalar(0,0,255), 1);
    }
    cout<<"processing the data!!!"<<endl;

    inputDet.emplace_back(object);
  }

  //跟踪处理
   //创建可视化array
   //进行跟踪
   //结果存储到array中
  //封装发送消息

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();
  visualization_msgs::Marker bbox_marker;
  bbox_marker.header.frame_id = "global_init_frame";
  bbox_marker.header.stamp = ros::Time::now();
  bbox_marker.ns = "";
  bbox_marker.lifetime = ros::Duration();
  bbox_marker.frame_locked = true;
  bbox_marker.type = visualization_msgs::Marker::CUBE;
  bbox_marker.action = visualization_msgs::Marker::ADD;

  visualization_msgs::MarkerArray text_marker_array;
  text_marker_array.markers.clear();
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "global_init_frame";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "";//namespace
  text_marker.lifetime = ros::Duration();//　 lifetime
  text_marker.frame_locked = true; 
  text_marker.color.r = 1.0f;
  text_marker.color.g = 0.0f;
  text_marker.color.b = 0.0f;
  text_marker.color.a = 1.0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;


  vector<Eigen::VectorXd> result;
  int64_t tm0 = gtm();
  tracker_->track(inputDet, time, result);
  int64_t tm1 = gtm();

  printf("[INFO]update cast time:%ld us\n",  tm1-tm0);
  double x = tm1-tm0;
  totaltime += x;

  int marker_id = 0;
  for(int i=0; i<result.size(); ++i){
    Eigen::VectorXd r = result[i];////id, fx, fy, angle, mx, my, yaw, l, w, h, z
    if(frame != 0){
      Eigen::Vector3d p_0(r(1), r(2), 0);//在全局坐标系下进行的跟踪结果
      Eigen::Vector3d p_1;
      p_1 = origion2translate * p_0;//转换到当前帧坐标系
      // p_1 = p_0;
      r(1) = p_1(0);
      r(2) = p_1(1);
    }

    Detect det;
    det.box2D.resize(4);
    det.box.resize(3);
    det.box[0]     = r(9);//h
    det.box[1]     = r(8);//w
    det.box[2]     = r(7);//l
    det.z          = r(10);
    det.yaw        = r(6);
    det.position   = Eigen::VectorXd(2);
    det.position << r(2),-r(1);
    cout<<"det: "<<det.position(0)<<" "<<det.position(1)<<" "<<det.z<<" "<<det.box[0]<<" "<<det.box[1]<<" "<<det.box[2]<<endl;

    //导出评估文件格式
    /*
   
    float x1,y1,x2,y2,x,y,z,yaw, score;
    x1 = float(msg->objects[i].box2d.points[0].x);
    y1 = float(msg->objects[i].box2d.points[0].y);
    x2 = float(msg->objects[i].box2d.points[1].x);
    y2 = float(msg->objects[i].box2d.points[1].y);
    x = float(msg->objects[i].position.x);
    y = float(msg->objects[i].position.y);
    z = float(msg->objects[i].z);
    yaw = float(msg->objects[i].yaw);
    score = float(msg->objects[i].score);

    [frame id class 截断系数　遮挡系数　观测角　ｘ1 y1 x2 y2 h w l x y z yaw score]
    */
    // out_txt_file<<setprecision(6)<<frame<<" "<<int(r(0))<<" "<<"Car"<<" "<<0<<" "<<0<<" "<<r(3)<<" "<<msg->objects[i].box2d.points[0].x<<" "\
    // <<msg->objects[i].box2d.points[0].y<<" "<<msg->objects[i].box2d.points[1].x<<" "<<msg->objects[i].box2d.points[1].y<<" "\
    // <<msg->objects[i].box3d.x<<" "<<msg->objects[i].box3d.y<<" "<<msg->objects[i].box3d.z<<" "<<msg->objects[i].position.x<<" "\
    // <<msg->objects[i].position.y<<" "<<msg->objects[i].z<<" "<<msg->objects[i].yaw<<" "<<msg->objects[i].score<<endl;
    
    out_txt_file<<setprecision(6)<<frame<<" "<<int(r(0))<<" "<<"Car"<<" "<<0<<" "<<0<<" "<<r(3)<<" "<<349.22<<" "\
    <<114.118<<" "<<379.46<<" "<<194.975<<" "\
    <<r(9)<<" "<<r(8)<<" "<<r(7)<<" "<<det.position(0)<<" "\
    <<det.position(1)<<" "<<det.z<<" "<<det.yaw<<endl;
    
   
    if (!idcolor.count(int(r(0)))){
                  int red = rng.uniform(0, 255);
                  int green = rng.uniform(0, 255);
                  int blue = rng.uniform(0, 255);			
      idcolor[int(r(0))] = {red,green,blue};
    }
    draw3dbox(det, rgbimg, idcolor[int(r(0))]);


    Eigen::Vector3f eulerAngle(-det.yaw, 0.0, 0.0);
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitZ()));
    const Eigen::Quaternionf bboxQ1(yawAngle * pitchAngle * rollAngle);
    Eigen::Vector4f q = bboxQ1.coeffs();

    bbox_marker.id = marker_id;
    bbox_marker.pose.position.x    = det.position(0);
    bbox_marker.pose.position.y    = det.position(1);
    bbox_marker.pose.position.z    = det.z+ det.box[0]/2;
    bbox_marker.pose.orientation.x = q[0];
    bbox_marker.pose.orientation.y = q[1];
    bbox_marker.pose.orientation.z = q[2];
    bbox_marker.pose.orientation.w = q[3];
    bbox_marker.scale.x            = det.box[1];
    bbox_marker.scale.y            = det.box[2];
    bbox_marker.scale.z            = det.box[0];
    bbox_marker.color.r            = float(idcolor[int(r(0))][0])/255;
    bbox_marker.color.g            = float(idcolor[int(r(0))][1])/255;
    bbox_marker.color.b            = float(idcolor[int(r(0))][2])/255;
    bbox_marker.color.a            = 0.8;
    marker_array.markers.push_back(bbox_marker);

    text_marker.id = marker_id;
    text_marker.pose.position.x    = det.position(0);
    text_marker.pose.position.y    = det.position(1)-1.5;
    text_marker.pose.position.z    = det.z + det.box[0]/2 + 1;
    text_marker.pose.orientation.x = 0;
    text_marker.pose.orientation.y = 0;
    text_marker.pose.orientation.z = 0;
    text_marker.pose.orientation.w = 1;
    text_marker.scale.x            = 2;
    text_marker.scale.y            = 2;
    text_marker.scale.z            = 2;
    text_marker.text = "ID: " + to_string(int(r(0)));// 目标ＩＤ
    cout<<"ID"<<" "<<int(r(0))<<endl;
    text_marker_array.markers.push_back(text_marker);
    ++marker_id;

  }

  if (marker_array.markers.size() > max_marker_size_) {
    max_marker_size_ = marker_array.markers.size();
    
  }
  cout<<"marker_id: "<<marker_id<<"max_marker_size_ "<<max_marker_size_<<endl;

  //　不够５０个要补够？？为什么
  
  for (size_t i = marker_id; i < max_marker_size_; ++i) {
    bbox_marker.id = i;
    bbox_marker.color.a = 0;
    bbox_marker.pose.position.x = 0;
    bbox_marker.pose.position.y = 0;
    bbox_marker.pose.position.z = 0;
    bbox_marker.scale.x = 0;
    bbox_marker.scale.y = 0;
    bbox_marker.scale.z = 0;
    marker_array.markers.push_back(bbox_marker);

    text_marker.id = i;
    text_marker.color.a = 0;
    text_marker.pose.position.x = 0;
    text_marker.pose.position.y = 0;
    text_marker.pose.position.z = 0;
    text_marker.scale.x = 0;
    text_marker.scale.y = 0;
    text_marker.scale.z = 0;
    text_marker_array.markers.push_back(text_marker);
    ++marker_id;
  }

  // cv::imshow("x", rgbimg);
  // cv::waitKey(1);
  sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",rgbimg).toImageMsg(); 
  pubimage.publish(imgmsg);
  sensor_msgs::PointCloud2 tem_pc = msg->pointcloud;
  tem_pc.header.frame_id = "global_init_frame";
  publidar.publish(tem_pc);
  pubtextmarker.publish(text_marker_array);
  pubmarker.publish(marker_array);

  time += 0.1;
  frame++;
  
}
  


track_processor::track_processor(){
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  rng(12345);
  time = 0.1;
  totaltime = 0.0;
  max_marker_size_ = 50;
	// oriheading=0;
	// orix =0;
	// oriy = 0;
	// preheading=0;
	// prex =0;
	// prey = 0;
  frame = 0;
  string track_out_path;
	ros::param::get("/tracking_node/track_out_path", track_out_path);
  //初始化输出文件
  // out_txt_file.open("/mnt/data/workspace/ROS_MOT/track_ws/src/mot_tracking/result/0002.txt", ios::out|ios::trunc);
  out_txt_file.open(track_out_path, ios::out|ios::trunc);
  out_txt_file<<fixed;

  images = cv::Mat::zeros(608,608,CV_8UC3);
  tracker_ = std::make_shared<Tracker>(param);
  datasub = nh.subscribe("/objects", 1, &track_processor::process, this);
  pubimage = it.advertise("/mot_tracking/image", 1);
  publidar = nh.advertise<sensor_msgs::PointCloud2>("/mot_tracking/pointcloud", 1);
  pubmarker = nh.advertise<visualization_msgs::MarkerArray>("/mot_tracking/box", 1 );
  pubtextmarker = nh.advertise<visualization_msgs::MarkerArray>("/mot_tracking/id", 1 );
  
  ros::spin();
  out_txt_file.close();
  
}