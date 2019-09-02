#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <math.h>

ros::Publisher helix_point_pub;
mavros_msgs::GlobalPositionTarget point;

float lat, lon, alt, heading;


float getAverageDistance(const sensor_msgs::Image::ConstPtr& msg, int& left, int& top, int& width, int& height ){
    
    
    float avgdist = 0;
    float count = 0;
    /**
    for(int row = 0; row < srcimg.rows; row++) { 
        for(int col = 0; col < srcimg.cols; col++) { 
            //uchar pixel = imgMask.at<uchar>(row, col); 
            if( (col > left) && (col < (left + width)) && (row >= top) && (row <= (row + height)) ) { 
                //imgSrc.at<uchar>(row, col) = pixel; 
                avgdist = avgdist + (float)srcimg.at<uchar>(row, col);
                count = count + 1 ;
            } 
        } 
    }
    avgdist = avgdist /count;
    
    return avgdist;

    */

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    // Image coordinates of the center pixel
    int u = msg->width / 2;
    int v = msg->height / 2;

    // Linear index of the center pixel
    int centerIdx = u + msg->width * v;

    // Output the measure
    ROS_INFO("Center distance : %g m", depths[centerIdx]);


    for (int row = top; row <= top + height; row++){
        for (int col = left; col <= left + width; col++){
            int index = col + msg->width * row;
            avgdist = avgdist + depths[index];
            count = count + 1.;
        }
    }

    //avgdist = avgdist / count;
    ROS_INFO_STREAM("Avg Dist:" << avgdist);;
}

void DepthCallback(const sensor_msgs::Image::ConstPtr& msg) {
   cv_bridge::CvImagePtr cv_ptr;
   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
   cv::Mat depth_img = cv_ptr->image;
   cv::Mat thr_img;

   cv::threshold(depth_img, thr_img, 0.8, 255, cv::THRESH_BINARY_INV);

   thr_img.convertTo(thr_img, CV_8UC1);

   cv::resize(thr_img, thr_img, cv::Size(640,360), 0, 0, CV_INTER_NN);
   cv::Mat display_img = thr_img;

   /* test depth distance 
   std::cout<<thr_img.at<uchar>(640,360)<<std::endl;
   std::cout<<img.at<float>(640,360)<<std::endl;
   cv::circle(thr_img, cv::Point(640,360), 3, 100, 1);
   */

   cv::Mat labels, stats, centroids;
   int n = cv::connectedComponentsWithStats(thr_img, labels, stats, centroids, 8);

   int height_max = 0;
   int i_max = -1;
   for (int i = 0; i<n; i++){
       if (i < 1) continue;
       int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
       //float ratio = stats.at<int>(i, cv::CC_STAT_HEIGHT) / stats.at<int>(i, cv::CC_STAT_WIDTH);
       if (height_max < height && height >= 180){
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
      cv::rectangle(display_img, cv::Point(left,top), cv::Point(left+width,top+height),100,3);
   }

   cv::imshow("depth", display_img);
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
    ros::NodeHandle n1, n2, n3, n4;
    image_transport::ImageTransport it(n1);

    image_transport::Subscriber Depth  = it.subscribe("/zed/zed_node/depth/depth_registered", 1, DepthCallback);
    ros::Subscriber gps_sub = n2.subscribe("/mavros/global_position/global",1, gpsCallback);
    ros::Subscriber heading_sub = n3.subscribe("/mavros/global_position/compass_hdg",1, headingCallback);
    helix_point_pub = n4.advertise<mavros_msgs::GlobalPositionTarget>("/setpoint_helix",1);

    if (!ros::ok()){
        cv::destroyAllWindows();
        ros::shutdown();
    }
    ros::spin();
    return 0;

}
