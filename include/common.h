#ifndef CLOUDLINE2PLANE_COMMON_H
#define CLOUDLINE2PLANE_COMMON_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv/cv.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <limits>

#include "fstruct.h"



using PointT=pcl::PointXYZ;
using CloudT=pcl::PointCloud<PointT>;

const int COLOR_TABLE_SIZE = 82;
static const unsigned char default_colors[COLOR_TABLE_SIZE][3] =
{
    {255, 0, 0}, 
    {255, 255, 0}, 
    {100, 20, 50}, 
    {0, 30, 255}, 
    {10, 255, 60}, 
    {80, 10, 100}, 
    {0, 255, 200}, 
    {10, 60, 60}, 
    {255, 0, 128}, 
    {60, 128, 128}, 
    {15, 99, 98}, 
    {102, 159, 227}, 
    {245, 180, 173}, 
    {37, 229, 49}, 
    {110, 81, 166}, 
    {238, 29, 94}, 
    {232, 57, 157}, 
    {17, 209, 187}, 
    {117, 165, 112}, 
    {38, 25, 29}, 
    {9, 201, 164}, 
    {149, 110, 55}, 
    {238, 108, 3}, 
    {113, 69, 235}, 
    {47, 52, 61},     
    {217, 129, 225}, 
    {19, 58, 205}, 
    {46, 234, 98}, 
    {206, 249, 79}, 
    {134, 45, 252}, 
    {96, 61, 59}, 
    {149, 203, 31}, 
    {119, 178, 190}, 
    {92, 168, 242}, 
    {137, 78, 217}, 
    {186, 167, 91}, 
    {28, 186, 21}, 
    {105, 233, 1}, 
    {203, 56, 122}, 
    {27, 62, 167}, 
    {25, 158, 100}, 
    {211, 180, 176}, 
    {114, 44, 99}, 
    {49, 8, 139}, 
    {164, 145, 217}, 
    {126, 76, 130}, 
    {89, 231, 188}, 
    {85, 162, 91}, 
    {180, 140, 20}, 
    {209, 157, 229}, 
    {141, 146, 11}, 
    {126, 57, 163}, 
    {27, 66, 110}, 
    {191, 87, 92}, 
    {246, 71, 73}, 
    {122, 56, 14}, 
    {102, 230, 66}, 
    {35, 61, 101}, 
    {126, 113, 241}, 
    {18, 67, 15}, 
    {247, 80, 161}, 
    {131, 206, 218}, 
    {39, 105, 157}, 
    {21, 42, 244}, 
    {114, 160, 187}, 
    {59, 27, 244}, 
    {73, 129, 219}, 
    {12, 37, 152}, 
    {113, 35, 10}, 
    {226, 54, 204}, 
    {113, 173, 157}, 
    {146, 49, 235}, 
    {237, 89, 86}, 
    {139, 110, 128}, 
    {0, 96, 160}, 
    {188, 155, 188}, 
    {177, 101, 189}, 
    {13, 240, 226}, 
    {166, 225, 7}, 
    {48, 68, 61}, 
    {253, 181, 106}, 
    {155, 72, 156}
};

constexpr std::array<std::array<int, 3>, 200> RANDOM_COLORS = {{
  {{104, 109, 253}}, {{125, 232, 153}}, {{158, 221, 134}},
  {{228, 109, 215}}, {{249, 135, 210}}, {{255, 207, 237}},
  {{151, 120, 235}}, {{145, 123, 213}}, {{172, 243, 184}},
  {{105, 131, 110}}, {{217, 253, 154}}, {{250, 102, 109}},
  {{116, 179, 127}}, {{200, 251, 206}}, {{117, 146, 240}},
  {{234, 162, 176}}, {{160, 172, 171}}, {{205, 129, 168}},
  {{197, 167, 238}}, {{234, 248, 101}}, {{226, 240, 119}},
  {{189, 211, 231}}, {{226, 170, 216}}, {{109, 180, 162}},
  {{115, 167, 221}}, {{162, 134, 131}}, {{203, 169, 114}},
  {{221, 138, 114}}, {{246, 146, 237}}, {{200, 167, 244}},
  {{198, 150, 236}}, {{237, 235, 191}}, {{132, 137, 171}},
  {{136, 219, 103}}, {{229, 210, 135}}, {{133, 188, 111}},
  {{142, 144, 142}}, {{122, 189, 120}}, {{127, 142, 229}},
  {{249, 147, 235}}, {{255, 195, 148}}, {{202, 126, 227}},
  {{135, 195, 159}}, {{139, 173, 142}}, {{123, 118, 246}},
  {{254, 186, 204}}, {{184, 138, 221}}, {{112, 160, 229}},
  {{243, 165, 249}}, {{200, 194, 254}}, {{172, 205, 151}},
  {{196, 132, 119}}, {{240, 251, 116}}, {{186, 189, 147}},
  {{154, 162, 144}}, {{178, 103, 147}}, {{139, 188, 175}},
  {{156, 163, 178}}, {{225, 244, 174}}, {{118, 227, 101}},
  {{176, 178, 120}}, {{113, 105, 164}}, {{137, 105, 123}},
  {{144, 114, 196}}, {{163, 115, 216}}, {{143, 128, 133}},
  {{221, 225, 169}}, {{165, 152, 214}}, {{133, 163, 101}},
  {{212, 202, 171}}, {{134, 255, 128}}, {{217, 201, 143}},
  {{213, 175, 151}}, {{149, 234, 191}}, {{242, 127, 242}},
  {{152, 189, 230}}, {{152, 121, 249}}, {{234, 253, 138}},
  {{152, 234, 147}}, {{171, 195, 244}}, {{254, 178, 194}},
  {{205, 105, 153}}, {{226, 234, 202}}, {{153, 136, 236}},
  {{248, 242, 137}}, {{162, 251, 207}}, {{152, 126, 144}},
  {{180, 213, 122}}, {{230, 185, 113}}, {{118, 148, 223}},
  {{162, 124, 183}}, {{180, 247, 119}}, {{120, 223, 121}},
  {{252, 124, 181}}, {{254, 174, 165}}, {{188, 186, 210}},
  {{254, 137, 161}}, {{216, 222, 120}}, {{215, 247, 128}},
  {{121, 240, 179}}, {{135, 122, 215}}, {{255, 131, 237}},
  {{224, 112, 171}}, {{167, 223, 219}}, {{103, 200, 161}},
  {{112, 154, 156}}, {{170, 127, 228}}, {{133, 145, 244}},
  {{244, 100, 101}}, {{254, 199, 148}}, {{120, 165, 205}},
  {{112, 121, 141}}, {{175, 135, 134}}, {{221, 250, 137}},
  {{247, 245, 231}}, {{236, 109, 115}}, {{169, 198, 194}},
  {{196, 195, 136}}, {{138, 255, 145}}, {{239, 141, 147}},
  {{194, 220, 253}}, {{149, 209, 204}}, {{241, 127, 132}},
  {{226, 184, 108}}, {{222, 108, 147}}, {{109, 166, 185}},
  {{152, 107, 167}}, {{153, 117, 222}}, {{165, 171, 214}},
  {{189, 196, 243}}, {{248, 235, 129}}, {{120, 198, 202}},
  {{223, 206, 134}}, {{175, 114, 214}}, {{115, 196, 189}},
  {{157, 141, 112}}, {{111, 161, 201}}, {{207, 183, 214}},
  {{201, 164, 235}}, {{168, 187, 154}}, {{114, 176, 229}},
  {{151, 163, 221}}, {{134, 160, 173}}, {{103, 112, 168}},
  {{209, 169, 218}}, {{137, 220, 119}}, {{168, 220, 210}},
  {{182, 192, 194}}, {{233, 187, 120}}, {{223, 185, 160}},
  {{120, 232, 147}}, {{165, 169, 124}}, {{251, 159, 129}},
  {{182, 114, 178}}, {{159, 116, 158}}, {{217, 121, 122}},
  {{106, 229, 235}}, {{164, 208, 214}}, {{180, 178, 142}},
  {{110, 206, 136}}, {{238, 152, 205}}, {{109, 245, 253}},
  {{213, 232, 131}}, {{215, 134, 100}}, {{163, 140, 135}},
  {{233, 198, 143}}, {{221, 129, 224}}, {{150, 179, 137}},
  {{171, 128, 119}}, {{210, 245, 246}}, {{209, 111, 161}},
  {{237, 133, 194}}, {{166, 157, 255}}, {{191, 206, 225}},
  {{125, 135, 110}}, {{199, 188, 196}}, {{196, 101, 202}},
  {{237, 211, 167}}, {{134, 118, 177}}, {{110, 179, 126}},
  {{196, 182, 196}}, {{150, 211, 218}}, {{162, 118, 228}},
  {{150, 209, 185}}, {{219, 151, 148}}, {{201, 168, 104}},
  {{237, 146, 123}}, {{234, 163, 146}}, {{213, 251, 127}},
  {{227, 152, 214}}, {{230, 195, 100}}, {{136, 117, 222}},
  {{180, 132, 173}}, {{112, 226, 113}}, {{198, 155, 126}},
  {{149, 255, 152}}, {{223, 124, 170}}, {{104, 146, 255}},
  {{113, 205, 183}}, {{100, 156, 216}},
}};

inline void pubCvMatMsg(const image_transport::Publisher& img_pub,
                const cv::Mat& image, std::string frameId, ros::Time timestamp) {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = timestamp;
    cv_ptr->header.frame_id = frameId;  
    cv_ptr->image = image;  
    img_pub.publish(cv_ptr->toImageMsg());
}

inline CloudT CloudFromCoords(float *P[3], int rows, int cols) {
    CloudT segcomp_cloud;
    segcomp_cloud.width = cols;
    segcomp_cloud.height = rows;
    segcomp_cloud.is_dense = false;
    segcomp_cloud.points.resize(cols*rows);
    for (int r=0; r < rows; r++) {
        for (int c=0; c <  cols; c++) {
            PointT pt;
            // pt.x = 0.01*P[2][r*cols+c];
            // pt.y = 0.01*P[1][r*cols+c];
            // pt.z = 0.01*P[0][r*cols+c];
            pt.x = P[2][c*rows+r];
            pt.y = P[0][c*rows+r];
            pt.z = P[1][c*rows+r];
            segcomp_cloud.at(c, r) = pt;
        }
    }  
    return segcomp_cloud;  
}

inline CloudT loadPTXCloud(std::string file_name) {
    CloudT cloud_tmp;

    std::ifstream input(file_name.c_str(), std::ios::in);
    if(!input.good()){
        std::cerr << "Could not read file: " << file_name << std::endl;
    }

    int i=0;
    std::string line;
    while(getline(input, line)) {
        std::stringstream ss(line);
        // float px, py, pz;
        // ss >> px >> py >> pz;
        // if(i==0) {
        //     std::cout << px << ", " << py << ", " << pz << std::endl;
        // }
        std::string spx, spy, spz;
        std::getline(ss, spx, ' ');
        std::getline(ss, spy, ' ');
        std::getline(ss, spz, ' ');
        float px, py, pz;
        if(spx=="NaN" || spy=="NaN" || spz=="NaN") {
            px = py = pz = std::numeric_limits<float>::quiet_NaN();
        } else {
            px = std::stof(spx);
            py = std::stof(spy);
            pz = std::stof(spz);
        }

        PointT pt;   
        pt.x = px;
        pt.y = py;
        pt.z = pz;  
        cloud_tmp.points.push_back(pt);
    }

    return cloud_tmp;
}


inline CloudT CloudFromCoordsVLP(float *P[3], int rows, int cols) {
    CloudT segcomp_cloud;
    segcomp_cloud.width = cols;
    segcomp_cloud.height = rows;
    segcomp_cloud.is_dense = false;
    segcomp_cloud.points.resize(cols*rows);
    for (int r=0; r < rows; r++) {
        for (int c=0; c <  cols; c++) {
            PointT pt;
            // pt.x = 0.01*P[2][r*cols+c];
            // pt.y = 0.01*P[1][r*cols+c];
            // pt.z = 0.01*P[0][r*cols+c];
            // pt.x = P[0][c*rows+r];
            // pt.y = P[1][c*rows+r];
            // pt.z = P[2][c*rows+r];
            pt.x = P[0][r*cols+c];
            pt.y = P[1][r*cols+c];
            pt.z = P[2][r*cols+c];            
            segcomp_cloud.at(c, r) = pt;
            // if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
            //     std::cout << "invalid point at: (" << c << ", " << r << ")" << std::endl;
            // }
        }
    }  
    return segcomp_cloud;  
}

inline void saveTimesToFile(std::string out_file, std::vector<double> times_vec) {
    // std::cout << "saving timings to file : " << out_file  << " ...." << std::endl;
    FILE* fptw = NULL;
    fptw = fopen(out_file.c_str(), "w");
    if(fptw == NULL) {
        std::cout << "cannot open " << out_file << std::endl;
        return;
    }

    for(int i=0; i < times_vec.size(); i++) {
        fprintf(fptw, "%lf\n", times_vec[i]);
    }
    fclose(fptw);
    // std::cout << "timing file saved ..." << std::endl;    
}

#endif