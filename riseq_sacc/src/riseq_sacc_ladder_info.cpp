#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <riseq_sacc/RiseSaccHelix.h>
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>

ros::Publisher ladder_info_pub;
riseq_sacc::RiseSaccHelix ladder_info;

float abs_alt, home_alt ,rel_alt;
int center_x, center_y, left, top, width, height;

void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg){
  home_alt = msg->geo.altitude;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  abs_alt = msg->altitude;
}

void DepthCallback(const sensor_msgs::Image::ConstPtr& msg) {
  /*convert opencv Mat*/
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat depth_img = cv_ptr->image;
  
  /*create threshold image (grayscale)*/
  float threshold_meter = 8; 
  cv::Mat thr_img;
  cv::threshold(depth_img, thr_img, threshold_meter, 255, cv::THRESH_BINARY_INV);
  thr_img.convertTo(thr_img, CV_8UC1);

  /*delete ground*/
  rel_alt = abs_alt - home_alt;

  cv::Rect rect(0,360,1280,360);

  if(rel_alt <= 10){
    thr_img(rect) = 0;
  }

  /*create display image*/
  cv::Mat display_img = thr_img;
  
  /*labeling*/
  cv::Mat labels, stats, centroids;
  int n = cv::connectedComponentsWithStats(thr_img, labels, stats, centroids, 8);
  int height_max = 0;
  int i_max = -1;

  for (int i = 0; i<n; i++){
      if (i < 1) continue;
      int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
      //float ratio = stats.at<int>(i, cv::CC_STAT_HEIGHT) / stats.at<int>(i, cv::CC_STAT_WIDTH);
      int min_height = (rel_alt <= 10) ? 90 : 180;
      if (height_max < height && height >= min_height){
          height_max = height;
          i_max = i;
      }
  } 

  float count = 0;
  float sum_dist = 0.0;
  float avgdist; 
  /* if found ladder*/
  if(i_max != -1){
    center_x = centroids.at<double>(i_max,0);
    center_y = centroids.at<double>(i_max,1);
    left = stats.at<int>(i_max,cv::CC_STAT_LEFT);
    top = stats.at<int>(i_max,cv::CC_STAT_TOP);
    width = stats.at<int>(i_max,cv::CC_STAT_WIDTH);
    height = stats.at<int>(i_max,cv::CC_STAT_HEIGHT);
    cv::rectangle(display_img, cv::Point(left,top), cv::Point(left+width,top+height),100,3);
    
    /* calculate average depth */  
    for(int y = top; y <= top + height; y++){
      float* row_pointer = depth_img.ptr<float>(y);
      for(int x = left; x <= left + width; x++){
        if((row_pointer[x] < threshold_meter) && std::isfinite(row_pointer[x])){
           count = count + 1;
           sum_dist += row_pointer[x]; 
        }
      }
    }
    avgdist = sum_dist / count;

    ladder_info.header.stamp = ros::Time::now();
    ladder_info.x = left+width/2;  // bbox x-center
    ladder_info.y = top+height/2;  // bbox y-center
    ladder_info.width = depth_img.cols;  //image width
    ladder_info.height = depth_img.rows;  //image height
    ladder_info.depth = avgdist;
    ladder_info_pub.publish(ladder_info);



  }
  /* if not found ladder */ 
  else{
    avgdist = -1;
  }

  // Output the measure 
  //cv::imshow("depth", display_img);
  //cv::waitKey(1);

}

int main(int argc, char **argv){
  ros::init(argc, argv, "riseq_sacc_helix");
  ros::NodeHandle n1, n2, n3, n4;

  image_transport::ImageTransport it(n1);
  image_transport::Subscriber Depth  = it.subscribe("/zed/zed_node/depth/depth_registered", 1, DepthCallback);
  ros::Subscriber HomePosition_sub = n2.subscribe("/mavros/home_position/home",1, HomePositionCallback);
  ros::Subscriber gps_sub = n3.subscribe("/mavros/global_position/global",1, gpsCallback);

  ladder_info_pub = n4.advertise<riseq_sacc::RiseSaccHelix>("/riseq/sacc/ladder_info",1);

  if (!ros::ok()){
      cv::destroyAllWindows();
      ros::shutdown();
  }
  ros::spin();
  return 0;

}
