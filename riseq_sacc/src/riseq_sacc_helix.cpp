#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>

float lat, lon, alt, heading;

void DepthCallback(const sensor_msgs::Image::ConstPtr& msg) {
   cv_bridge::CvImagePtr cv_ptr;
   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
   cv::Mat img = cv_ptr->image;
   cv::Mat thr_img;

   cv::threshold(img, thr_img, 0.8, 255, cv::THRESH_BINARY_INV);

   thr_img.convertTo(thr_img, CV_8UC1);

   /* test depth distance 
   std::cout<<thr_img.at<uchar>(640,360)<<std::endl;
   std::cout<<img.at<float>(640,360)<<std::endl;
   cv::circle(thr_img, cv::Point(640,360), 3, cv::Scalar(0,0,255), 1);
   */

   cv::Mat labels, stats, centroids;
   int n = cv::connectedComponentsWithStats(thr_img, labels, stats, centroids, 8);

   int height_max = 0;
   int i_max = -1;
   for (int i = 0; i<n; i++){
       if (i < 1) continue;
       int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
       float ratio = stats.at<int>(i, cv::CC_STAT_HEIGHT) / stats.at<int>(i, cv::CC_STAT_WIDTH);
       if (height_max < height && height >= 450 && ratio > 1){
           height_max = height;
           i_max = i;
       }
   }
   
   if(i_max != -1){
      int center_x = centroids.at<double>(i_max,0);
      int center_y = centroids.at<double>(i_max,1);
      int left = stats.at<int>(i_max,cv::CC_STAT_LEFT);
      int top = stats.at<int>(i_max,cv::CC_STAT_TOP);
      int width = stats.at<int>(i_max,cv::CC_STAT_WIDTH);
      int height = stats.at<int>(i_max,cv::CC_STAT_HEIGHT);
      cv::rectangle(thr_img, cv::Point(left,top), cv::Point(left+width,top+height),cv::Scalar(66,255,5),3);
   }

   cv::imshow("depth", thr_img);

   cv::waitKey(1);

}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
   lat = msg->latitude;
   lon = msg->longitude;
   alt = msg->altitude;
}

void headingCallback(const std_msgs::Float64::ConstPtr& msg){
   heading = msg->data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rise_sacc_helix");
    ros::NodeHandle n1, n2, n3;
    image_transport::ImageTransport it(n1);
    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

    image_transport::Subscriber Depth  = it.subscribe("/zed/zed_node/depth/depth_registered", 1, DepthCallback);
    ros::Subscriber gps_sub = n2.subscribe("/mavros/global_position/global",1, gpsCallback);
    ros::Subscriber heading_sub = n3.subscribe("/mavros/global_position/compass_hdg",1, headingCallback);

    if (!ros::ok()){
        cv::destroyAllWindows();
        ros::shutdown();
    }
    ros::spin();
    return 0;

}