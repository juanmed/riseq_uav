#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Int8.h>
#include <vector>

int num;
std::vector<std::string> Class(2);
std::vector<int> xmin(2);
std::vector<int> ymin(2);
std::vector<int> xmax(2); 
std::vector<int> ymax(2);
int detection_number[10] = {0,};
int j = 0;
std::vector<std::string> order_number(10); 

/* screen capture part */
cv::VideoWriter writer("/home/nvidia/Desktop/number_detection.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30.0, cv::Size(640,360));

void numCallback(const std_msgs::Int8::ConstPtr& msg){
    num = msg->data;  
}

void boundingCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
  if(num != 0){
    for(int i=0; i < num; i++){
      /*we should send hovering signal to pixhawk*/
      /*msg data to box structure*/
      Class[i] = msg->bounding_boxes[i].Class;
      xmin[i] = (int)msg->bounding_boxes[i].xmin/2;
      ymin[i] = (int)msg->bounding_boxes[i].ymin/2;
      xmax[i] = (int)msg->bounding_boxes[i].xmax/2;
      ymax[i] = (int)msg->bounding_boxes[i].ymax/2;
  
      /*String to int*/
      int Class_int;
      std::stringstream s(Class[i]);
      s >> Class_int;

      /*Count detected number*/
      if(detection_number[Class_int] == -1){
        return;
      }
 
      detection_number[Class_int]++;
      if(detection_number[Class_int] >= 20){
        detection_number[Class_int] = -1;
        order_number[j] = Class[i];
        j++;
      }
    }   
  }
}

void draw_bound_Callback(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr->image;
  cv::resize(img, img, cv::Size(640,360), 0, 0, CV_INTER_NN);
  
  if(num != 0){
    for(int i=0; i <num; i++){
      cv::rectangle(img, cv::Point(xmin[i]-2, ymin[i] - 30), cv::Point(xmin[i] + 20, ymin[i]), cv::Scalar(0,0,255), -1);
      cv::rectangle(img, cv::Point(xmin[i], ymin[i]), cv::Point(xmax[i], ymax[i]), cv::Scalar(0,0,255), 2);
      cv::putText(img, Class[i], cv::Point(xmin[i], ymin[i] - 3 ), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1);
    }
  }
  for(int i = 0; i < j; i++){
    cv::putText(img, order_number[i], cv::Point(10 + i*20, 355), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1);
  }

  /* screen capture part */
  writer.write(img);
  
  /* image view */
  cv::imshow("Number", img);
  cv::waitKey(1);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "riseq_sacc_number");
  ros::NodeHandle n1, n2, n3;
  image_transport::ImageTransport it(n3);
  cv::namedWindow("Number", CV_WINDOW_AUTOSIZE);
  
  ros::Subscriber num_sub = n1.subscribe("/darknet_ros/found_object",1, numCallback);

  ros::Subscriber number_boundingbox_sub = n2.subscribe("/darknet_ros/bounding_boxes",1, boundingCallback);
  
  image_transport::Subscriber Number_zed_image_sub  = it.subscribe("/zed/zed_node/rgb/image_rect_color", 1, draw_bound_Callback);

  if (!ros::ok()){
    cv::destroyAllWindows();
    ros::shutdown();
    writer.release();	
  }

  ros::spin();
  return 0;
}
