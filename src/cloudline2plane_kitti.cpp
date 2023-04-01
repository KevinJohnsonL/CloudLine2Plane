#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "plane_extraction.h"
#include "segcomp_loader.h"
#include "tictoc.h"
#include "save_result.h"

#include <chrono>
#include <thread>
#include "visualizer.h"

#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>

PlaneExtraction pe;
// PlaneNormalVisualizer vis;

ros::Publisher pubLineSegmentsMarker;
ros::Publisher pubPlaneCloudMsg;
image_transport::Publisher r_image_pub;
image_transport::Publisher vl_image_pub;
image_transport::Publisher hl_image_pub;
image_transport::Publisher pl_image_pub;

TicToc timer;
std::vector<double> times_vec;
bool SaveResult = true;
std::string config_file = "/home/lemon/plane_lcd/src/CloudLine2Plane/config/hdl64_real.yaml";
std::string output_path = "/home/lemon/plane_lcd/data/kitti/"; 
std::string prefix = "";
SaveMultiPlanes* save_results_ptr = new SaveMultiPlanes(output_path, pe.proj_params()); 
// save_results_ptr = new SaveMultiPlanes(output_path, pe.proj_params());  

int frame_cnt = 0;

CloudT rosCloudMsgToPCLCloud(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg) {
    CloudT::Ptr laserCloudInRaw(new CloudT);
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRaw);

    CloudT::Ptr laserCloudIn(new CloudT);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudInRaw, *laserCloudIn, indices);

    return *laserCloudIn;
}


void cloudMsgHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg){
    ROS_INFO(" received frame: %d", frame_cnt ++);
    CloudT cloudIn = rosCloudMsgToPCLCloud(laserCloudMsg);
    timer.Tic();
    pe.segment_unordered(cloudIn.makeShared(), false);
    double time_eclipsed = timer.Toc();
    std::cout << "segmentation cost: " << time_eclipsed << " ms" << std::endl;
    times_vec.push_back(time_eclipsed);  

    //intialize visualization images
    auto vsweep_lines = pe.getVSweepLines();
    auto hsweep_lines = pe.getHSweepLines();
    auto hl_image = pe.getHLineImage();
    auto vl_image = pe.getVLineImage();

    cv::Mat plabel_image = pe.labelImage();
    cv::Mat range_img = pe.rangeImage();
    cv::Mat x_img = pe.xImage();
    cv::Mat y_img = pe.yImage();
    cv::Mat z_img = pe.zImage();

    std::vector<PlaneParams> pl_params_vec = pe.planeParamsVec();
    int nPlanesNum = pl_params_vec.size();
    std::cout << "segmented plane num: " << nPlanesNum << std::endl;

    //publish range image
     cv::Mat range_img_c = cv::Mat::zeros(range_img.size(), CV_8UC3);  
	for(int r=0; r < range_img.rows; r++)
		for(int c=0; c < range_img.cols; c++) {
			float rd = range_img.at<float>(r,c);
			if(rd < 0.001)
				continue;

			int range = int(rd*255.0/7.0);
            cv::Vec3b range_pix =  cv::Vec3b(range, range, range);
            range_img_c.at<cv::Vec3b>(r, c) = range_pix;
		}

	cv::Mat range_img_vis;
    cv::flip(range_img_c, range_img_vis, -1);
    pubCvMatMsg(r_image_pub, range_img_vis, "velodyne", laserCloudMsg->header.stamp);    

    //publish line image
    //vertical line image
    cv::Mat lineseg_img = cv::Mat::zeros(range_img.size(), CV_8UC3);
    for(int i=0; i < vsweep_lines.size(); i++ ) {
    	std::vector<line> line_segs = vsweep_lines[i];
    	for(int j=0; j < line_segs.size(); j++) {
	        pcl::PointXYZ color;
	        color.x = RANDOM_COLORS[j % 200][0];
	        color.y = RANDOM_COLORS[j % 200][1];
	        color.z = RANDOM_COLORS[j % 200][2]; 
    		int start_idx = line_segs[j].left;
    		int stop_idx = line_segs[j].right;
    		for(int k=start_idx; k<=stop_idx; k++) {
    			lineseg_img.at<cv::Vec3b>(k, i) = cv::Vec3b(color.x, color.y, color.z);
    		}
    	}
    }
    cv::Mat lineseg_img_vis;
    cv::flip(lineseg_img, lineseg_img_vis, -1);
    pubCvMatMsg(vl_image_pub, lineseg_img_vis, "velodyne", laserCloudMsg->header.stamp);    
    
    //horizatal line image
     lineseg_img = cv::Mat::zeros(range_img.size(), CV_8UC3);
    for(int i=0; i < hsweep_lines.size(); i++) {
    	std::vector<line> line_segs = hsweep_lines[i]; 	
    	for(int j=0; j < line_segs.size(); j++) {
    		int start_idx = line_segs[j].left;
    		int stop_idx = line_segs[j].right;
	        pcl::PointXYZ color;
	        color.x = RANDOM_COLORS[j % 200][0];
	        color.y = RANDOM_COLORS[j % 200][1];
	        color.z = RANDOM_COLORS[j % 200][2];       		
    		for(int k=start_idx; k<=stop_idx; k++) {
    			lineseg_img.at<cv::Vec3b>(i, k) = cv::Vec3b(color.x, color.y, color.z);
    		}
    	}    	
    }
	cv::flip(lineseg_img, lineseg_img_vis, -1);
    pubCvMatMsg(hl_image_pub, lineseg_img_vis, "velodyne", laserCloudMsg->header.stamp);    

    // publish plane label image
    cv::Mat plane_img = cv::Mat::zeros(plabel_image.size(), CV_8UC3);
    // std::map<int,int> labels_map;
    // std::map<int, CloudT> planes_map;
    // std::map<int, std::vector<double>> colors_map;
    for(int i=0; i < plabel_image.rows; i++) {
        for(int j=0; j < plabel_image.cols; j++) {
            auto plabel = plabel_image.at<uint16_t>(i,j);
            if(plabel==0)     
                continue; 
            pcl::PointXYZ color;
            color.x = RANDOM_COLORS[plabel % 200][0];
            color.y = RANDOM_COLORS[plabel % 200][1];
            color.z = RANDOM_COLORS[plabel % 200][2];  
            plane_img.at<cv::Vec3b>(i, j) = cv::Vec3b(color.x, color.y, color.z);   
        }
    }
    cv::Mat plane_img_vis;
    cv::flip(plane_img, plane_img_vis, -1);
    pubCvMatMsg(pl_image_pub, plane_img_vis, "velodyne", laserCloudMsg->header.stamp);
    
    //publish labeled plane pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    for(int i=0; i < plabel_image.rows; i++) {
        for(int j=0; j < plabel_image.cols; j++) {
            auto plabel = plabel_image.at<uint16_t>(i,j);
            if(plabel == 0)
                continue;

            pcl::PointXYZ color;
            color.x = RANDOM_COLORS[plabel % 200][0];
            color.y = RANDOM_COLORS[plabel % 200][1];
            color.z = RANDOM_COLORS[plabel % 200][2];  
            pcl::PointXYZRGB pt;
            pt.x = x_img.at<float>(i,j);
            pt.y = y_img.at<float>(i,j);
            pt.z = z_img.at<float>(i,j);
            pt.r = color.z;
            pt.g = color.y;
            pt.b = color.x;
            color_cloud.push_back(pt);
        }
    }
    sensor_msgs::PointCloud2 cloudout_msg;
    pcl::toROSMsg(color_cloud, cloudout_msg);
    cloudout_msg.header = laserCloudMsg->header;
    pubPlaneCloudMsg.publish(cloudout_msg);

    // save label image and timing results
    if(SaveResult){
        save_results_ptr->run(
                plabel_image, 
                plane_img,
                pl_params_vec,
                frame_cnt,
                prefix,
                prefix);
    }

    // save running times
    if(SaveResult){
        std::string pathTimesFile = output_path + "timings/times.txt";
        saveTimesToFile(pathTimesFile, times_vec);

        std::vector<double> pr_times = pe.getPRTimeVec();
        std::vector<double> le_times = pe.getLETimeVec();
        std::vector<double> pe_times = pe.getPETimeVec();

        std::string pathPrTimesFile = output_path + "timings/pr_times.txt";
        std::string pathLeTimesFile = output_path + "timings/le_times.txt";
        std::string pathPeTimesFile = output_path + "timings/pe_times.txt";

        saveTimesToFile(pathPrTimesFile, pr_times);
        saveTimesToFile(pathLeTimesFile, le_times);
        saveTimesToFile(pathPeTimesFile, pe_times);
    }
}

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "cloudline2plane_kitti");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    std::string lidar_name;
    int v_min_line_points, v_seed_line_points;
    double v_least_thresh, v_min_line_length, v_predict_distance;
    int h_min_line_points, h_seed_line_points;
    double h_least_thresh, h_min_line_length, h_predict_distance;
    float line_plane_distThreshold;

    //params useless
    int v_pts_missing_tolerance, h_pts_missing_tolerance;
    float v_max_pts_gap, h_max_pts_gap;

     nh_local.param<std::string>("lidar_name", lidar_name, "HDL-64");

    //param used by linefeature algorithm
    nh_local.param<double>("v_least_thresh", v_least_thresh, 0.075);
    nh_local.param<double>("v_min_line_length", v_min_line_length, 0.10);
    nh_local.param<double>("v_predict_distance", v_predict_distance, 0.12);
    nh_local.param<int>("v_seed_line_points", v_seed_line_points, 2);
    nh_local.param<int>("v_min_line_points", v_min_line_points, 4);
    nh_local.param<int>("v_pts_missing_tolerance", v_pts_missing_tolerance, 2);
    nh_local.param<float>("v_max_pts_gap", v_max_pts_gap, 1.0);

    nh_local.param<double>("h_least_thresh", h_least_thresh, 0.075);
    nh_local.param<double>("h_min_line_length", h_min_line_length, 0.08);
    nh_local.param<double>("h_predict_distance", h_predict_distance, 0.12);
    nh_local.param<int>("h_seed_line_points", h_seed_line_points, 2);
    nh_local.param<int>("h_min_line_points", h_min_line_points, 5);
    nh_local.param<int>("h_pts_missing_tolerance", h_pts_missing_tolerance, 2);
    nh_local.param<float>("h_max_pts_gap", h_max_pts_gap, 1.0);

    nh_local.param<float>("line_plane_distThreshold", line_plane_distThreshold, 0.085);

   // boost::thread vis_thread(boost::bind(&PlaneNormalVisualizer::Spin, &vis));
    pe.loadParams(config_file);

    ROWS = pe.proj_params().rows();
    COLS = pe.proj_params().cols();

    // initialize color table.
    std::vector<cv::Vec3b> colors;
    for(int i=0; i<COLOR_TABLE_SIZE; ++i) {
        colors.push_back(cv::Vec3b(default_colors[i]));
    }

    ros::Subscriber subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 
    2, &cloudMsgHandler);

    pubPlaneCloudMsg = nh.advertise<sensor_msgs::PointCloud2>("/plane_cloud", 1);
    pubLineSegmentsMarker = nh.advertise<visualization_msgs::Marker>("/lines", 1);
    image_transport::ImageTransport it_(nh);
    r_image_pub = it_.advertise("/range_img", 1);
    vl_image_pub = it_.advertise("/vline_img", 1);
    hl_image_pub = it_.advertise("/hline_img", 1);
    pl_image_pub = it_.advertise("/plane_img", 1);

    ros::Rate r(100);
    while(ros::ok()) {
    	ros::spinOnce();
    	r.sleep();
    }

    return 0;
}