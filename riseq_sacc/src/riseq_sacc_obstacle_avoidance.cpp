#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <math.h>

#define PI 3.141592

ros::Publisher avoidance_point_pub;
mavros_msgs::GlobalPositionTarget point;

double cur_lat, cur_lon;
double target_lat = 37.564700, target_lon=126.627628;
//double target_lat = 37.17357, target_lon=126.58394;
double past_lat, past_lon;

float side_thr = 255/5;
float updown_thr = 255/2;
float front_thr = 255/15;

int width = 640;
int height = 360;
int width_13 = 213;
int width_23 = 427;
int height_13 = 120;
int height_23 = 240;

int front_count = 0;
int right_count = 0;

<<<<<<< HEAD


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

=======
>>>>>>> b5f87cbf810c4a32fd539f71ab9114d400aae90e
cv::Rect left_box(0, height_13, width_13, 120);
cv::Rect right_box(width_23, height_13, 640 - width_23 , 120);
cv::Rect up_box(0, 0, width, 120);
cv::Rect down_box(0, height_23, width, 120);
cv::Rect front_box(width_13, height_13, width_23 - width_13, height_23 - height_13);


void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  cur_lat = msg->latitude;
  cur_lon = msg->longitude;
}


void DepthCallback(const sensor_msgs::Image::ConstPtr& msg) {
  bool sw = false;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat img = cv_ptr->image;
  cv::Mat thr_img;

  cv::threshold(img, thr_img, 8, 255, cv::THRESH_BINARY_INV);

  thr_img.convertTo(thr_img, CV_8UC1);

  cv::resize(thr_img, thr_img, cv::Size(640,360), 0, 0, CV_INTER_NN);
  cv::Mat display_img = thr_img;
  cv::cvtColor(display_img, display_img, cv::COLOR_GRAY2BGR);
   
  cv::Scalar left_scalar = mean(thr_img(left_box));
  cv::Scalar right_scalar = mean(thr_img(right_box));
  cv::Scalar up_scalar = mean(thr_img(up_box));
  cv::Scalar down_scalar = mean(thr_img(down_box));
  cv::Scalar front_scalar = mean(thr_img(front_box));
   
  float left = left_scalar.val[0];
  float right = right_scalar.val[0];
  float up = up_scalar.val[0];
  float down = down_scalar.val[0];
  float front = front_scalar.val[0];

  bool left_obstacle; 
  bool right_obstacle;
  bool up_obstacle;
  bool down_obstacle;
  bool front_obstacle;
   
  left_obstacle = (left < side_thr) ? false : true;
  right_obstacle = (right < side_thr) ? false : true; 
  up_obstacle = (up < updown_thr) ? false : true; 
  down_obstacle = (down < updown_thr) ? false : true; 
  front_obstacle = (front < front_thr) ? false : true;

  if(front_count < 32 ){   
    if((left_obstacle || right_obstacle || front_obstacle) == true){      
      // stop, right
      ROS_INFO("stop");
      if(pow((cur_lat-target_lat)/0.00001129413,2)+pow((cur_lon-target_lon)/0.00000895247,2) < 1){
        past_lon = target_lon;
        past_lat = target_lat;
        target_lat = past_lat - (8.155*2.5/1000000);
        target_lon = past_lon + (4.812*2.5/1000000);

        sw = true;
        right_count++;
      }
      cv::arrowedLine(display_img, cv::Point(width_13, int(height/2)),cv::Point(width_23,int(height/2)), cv::Scalar(0,0,255), 3);
    }
    else if((left_obstacle || right_obstacle || front_obstacle) == false){
      //go
      ROS_INFO("go");
      if(pow((cur_lat-target_lat)/0.00001129413,2)+pow((cur_lon-target_lon)/0.00000895247,2) < 1){
        past_lon = target_lon;
        past_lat = target_lat;
        target_lat = past_lat + (4.579*2.5/1000000);
        target_lon = past_lon + (9.812*2.5/1000000);

        sw = true;
        front_count++;
      } 
      cv::circle(display_img, cv::Point(int(width/2), int(height/2)), 10, cv::Scalar(0,255,0),-1);                                  
    }     
   
  }
  else{
    sw = true;
    target_lat = 37.565111;
    target_lon = 126.628503;
    if(pow((cur_lat-target_lat)/0.00001129413,2)+pow((cur_lon-target_lon)/0.00000895247,2) < 1){
      ros::shutdown();
    }
  } 


  point.header.stamp = ros::Time::now();
  point.longitude = target_lon;
  point.latitude = target_lat;
  point.altitude = 25.065700531;
  point.yaw = 0.534942408493;
  avoidance_point_pub.publish(point);

  ROS_INFO("frond_count: %d", front_count);

  cv::line(display_img, cv::Point(0,height_13),cv::Point(width, height_13), cv::Scalar(0,0,255),1);
  cv::line(display_img, cv::Point(0,height_23),cv::Point(width, height_23), cv::Scalar(0,0,255),1);
  cv::line(display_img, cv::Point(width_13,height_13),cv::Point(width_13,height_23), cv::Scalar(0,0,255),1);
  cv::line(display_img, cv::Point(width_23,height_13),cv::Point(width_23,height_23), cv::Scalar(0,0,255),1) ;  
  cv::imshow("depth", display_img);

  cv::waitKey(1);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "riseq_sacc_obstacle_avoidance");
    ros::NodeHandle n1, n2, n3;
    image_transport::ImageTransport it(n2);

    ros::Subscriber gps_sub = n1.subscribe("/mavros/global_position/global",1, gpsCallback);
    image_transport::Subscriber Depth  = it.subscribe("/zed/zed_node/depth/depth_registered", 1, DepthCallback);
    avoidance_point_pub = n3.advertise<mavros_msgs::GlobalPositionTarget>("/setpoint_avoidance",1);

    if (!ros::ok()){
        cv::destroyAllWindows();
        ros::shutdown();
    }
    ros::spin();
    return 0;

}