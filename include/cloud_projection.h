#ifndef CLOUD_PROJECTION_H_
#define CLOUD_PROJECTION_H_

#include "projection_params.h"
#include "common.h"
#include "tictoc.h"

// store (z,rxy) (x,y).
class CloudProjection {
public:
  CloudProjection() {}

  CloudProjection(const ProjectionParams& params): m_params(params) {
  	initializeOutputImages();
  }
  
  void setProjectionParams(const ProjectionParams& params) {
  	m_params = params;
  	initializeOutputImages();
  }

  void initializeOutputImages() {
  	m_range_image = 500.0*cv::Mat::ones(m_params.rows(), 
                                        m_params.cols(), 
                                        cv::DataType<float>::type);
    m_rxy_image = 500.0*cv::Mat::ones(m_params.rows(), 
                                        m_params.cols(), 
                                        cv::DataType<float>::type);
    m_x_image = 500.0*cv::Mat::ones(m_params.rows(), 
                                        m_params.cols(), 
                                        cv::DataType<float>::type);
    m_y_image = 500.0*cv::Mat::ones(m_params.rows(), 
                                        m_params.cols(), 
                                        cv::DataType<float>::type);
    m_z_image = 500.0*cv::Mat::ones(m_params.rows(), 
                                        m_params.cols(), 
                                        cv::DataType<float>::type);  	

    m_valid_image = cv::Mat::zeros(m_params.rows(), 
                                        m_params.cols(), 
                                        cv::DataType<uint16_t>::type);  	
    m_index_image = -cv::Mat::ones(m_params.rows(),
                                        m_params.cols(),
                                        cv::DataType<int>::type);
    m_ground_image = cv::Mat::zeros(m_params.rows(),
                                        m_params.cols(),
                                        cv::DataType<uint16_t>::type);
    m_mask_image = 255*cv::Mat::ones(m_params.rows(),
                                        m_params.cols(),
                                        cv::DataType<uint8_t>::type);
  }

  void InitFromPoints(const CloudT& points) {
  	for(int i=0; i < points.size(); i++) {
  		const auto& pt = points[i];
  		float rxy = sqrt(pt.x*pt.x + pt.y*pt.y);
  		float range = sqrt(rxy*rxy + pt.z*pt.z);
  		float vangle = asin(pt.z / range);
  		float hangle = atan2(pt.y, pt.x);
  		size_t bin_row = m_params.RowFromAngle(vangle);
  		size_t bin_col = m_params.ColFromAngle(hangle);
  		auto& current_written_range = m_range_image.at<float>(bin_row, bin_col);
  		if(current_written_range > range) { //  && range>3.0
  			current_written_range = range;
  			m_rxy_image.at<float>(bin_row, bin_col) = rxy;
  			m_x_image.at<float>(bin_row, bin_col) = pt.x;
  			m_y_image.at<float>(bin_row, bin_col) = pt.y;
  			m_z_image.at<float>(bin_row, bin_col) = pt.z;
  			m_valid_image.at<uint16_t>(bin_row, bin_col) = 1;
  			m_index_image.at<int>(bin_row, bin_col) = i;
        m_mask_image.at<uint8_t>(bin_row, bin_col) = 0;
  		}
  	}
    //cv Mat to InputArray
    // range image completion(worked but not applied)
    // TicToc timer;
    // timer.Tic();
    // cv::InputArray range_image = m_range_image;
    // cv::inpaint(range_image, m_mask_image, m_range_image, 3, cv::INPAINT_NS);
    // double time = timer.Toc();
    // std::cout << "inpaint time: " << time << std::endl;

  }

  // void LabelGroundPoints(){ 
  //   int groundInd = m_params.rows() * 0.8;
  //   for(size_t j=0; j < m_params.cols(); j++) {
  //     for(size_t i=0; i < groundInd; i++) 
  //     {
  //         float diffRXY, diffZ, angle;
  //         size_t k = i+1;
  //         if(m_valid_image.at<uint16_t>(i, j) == 0) continue;
  //         for(; k < groundInd; k++) { 
  //           if(m_valid_image.at<uint16_t>(k, j) == 1) break;
  //         }
  //         // fprintf(stderr, "i: %d, k: %d, \n", i, k);
  //         // diffRXY = m_rxy_image.at<float>(i, j) - m_rxy_image.at<float>(k, j);
  //         diffRXY = sqrt(pow(m_x_image.at<float>(i, j) - m_x_image.at<float>(k, j), 2) + 
  //                        pow(m_y_image.at<float>(i, j) - m_y_image.at<float>(k, j), 2));
  //         diffZ = m_z_image.at<float>(i, j) - m_z_image.at<float>(k, j);
  //         angle = atan2(diffZ, diffRXY) * 180 / M_PI;
  //         fprintf(stderr, "diffRXY: %f, diffZ:%f, \n", diffRXY, diffZ);
  //         fprintf(stderr, "angle: %f, \n", angle);
  //         // fprintf(stderr, "angle: %f, \n", angle);
  //         if(fabsf(angle) < 5 || fabsf(angle) > 175)
  //           m_ground_image.at<uint16_t>(i, j) = 1;
  //           m_ground_image.at<uint16_t>(k, j) = 1;
  //         // float lidar_mount_height = 1.6;
  //         // if(m_z_image.at<float>(i, j) < -lidar_mount_height) 
  //         // // if(true)
  //         //   m_ground_image.at<uint16_t>(i, j) = 1;
  //     }
  //   }
  // }

  // void InitFromOrganizedPoints(const CloudT& points, bool colwise=true) {
  // 	// fprintf(stderr, " Cloud size: %d, \n", points.size());
  // 	for(int i=0; i < points.size(); i++) {

  // 		// float vangle = asin(pt.z / range);
  // 		// float hangle = atan2(pt.y, pt.x);
  // 		size_t bin_row, bin_col; 
  //       if(colwise) {
  //           bin_row = i%m_params.rows();
  // 		    bin_row = i/m_params.rows();
  //       } else {
  // 		    bin_row = i/m_params.cols();
  // 		    bin_col = i%m_params.cols();  
  //       }

  // 		const auto& pt = points[i];
  //       if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
  //           m_range_image.at<float>(bin_row, bin_col) = std::numeric_limits<float>::quiet_NaN(); 
  // 			m_rxy_image.at<float>(bin_row, bin_col) = std::numeric_limits<float>::quiet_NaN();
  // 			m_valid_image.at<uint16_t>(bin_row, bin_col) = 0;            
  //           continue;
  //       }

  // 		float rxy = sqrt(pt.x*pt.x + pt.y*pt.y);
  // 		float range = sqrt(rxy*rxy + pt.z*pt.z);

  // 		auto& current_written_range = m_range_image.at<float>(bin_row, bin_col);
  // 		if(range > 0.001) {
  // 			current_written_range = range;
  // 			m_rxy_image.at<float>(bin_row, bin_col) = rxy;
  // 			m_x_image.at<float>(bin_row, bin_col) = pt.x;
  // 			m_y_image.at<float>(bin_row, bin_col) = pt.y;
  // 			m_z_image.at<float>(bin_row, bin_col) = pt.z;
  // 			m_valid_image.at<uint16_t>(bin_row, bin_col) = 1;
  // 			m_index_image.at<int>(bin_row, bin_col) = i;  		
  // 		}
  // 	}
  // }


  const cv::Mat& depth_image() const {
  	return m_range_image;
  }
  cv::Mat& depth_image() {
  	return m_range_image;
  }
  inline void CloneDepthImage(const cv::Mat& image) {
    m_range_image = image.clone();
  }

  const cv::Mat& rxy_image() const {
  	return m_rxy_image;
  }
  cv::Mat& rxy_image() {
  	return m_rxy_image;
  }  
  const cv::Mat& x_image() const {
  	return m_x_image;
  }
  cv::Mat& x_image() {
  	return m_x_image;
  }
  const cv::Mat& y_image() const {
  	return m_y_image;
  }
  cv::Mat& y_image() {
  	return m_y_image;
  }
  const cv::Mat& z_image() const {
  	return m_z_image;
  }
  cv::Mat& z_image() {
  	return m_z_image;
  }

  const cv::Mat& valid_image() const {
  	return m_valid_image;
  }

  cv::Mat& valid_image() {
  	return m_valid_image;
  }

  const cv::Mat& index_image() const {
  	return m_index_image;
  }

  cv::Mat& index_image() {
  	return m_index_image;
  }
  const cv::Mat& ground_image() const {
  	return m_ground_image;
  }

  cv::Mat& ground_image() {
  	return m_ground_image;
  }


  inline size_t rows() const { return m_params.rows(); }
  inline size_t cols() const { return m_params.cols(); }
  inline size_t size() const { return m_params.size(); }
  inline const ProjectionParams& params() const { return m_params; }

private:

  ProjectionParams m_params;

  // output.
  cv::Mat m_range_image;	// five channels:  x, y, z, rxy, r.
  cv::Mat m_rxy_image;
  cv::Mat m_x_image;
  cv::Mat m_y_image;
  cv::Mat m_z_image;

  cv::Mat m_valid_image;
  cv::Mat m_index_image;
  cv::Mat m_ground_image;
  cv::Mat m_mask_image;
};




#endif