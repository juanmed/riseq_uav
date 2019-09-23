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
#include <mavros_msgs/HomePosition.h>

ros::Publisher target_contol_pub;
ros::Publisher gimbal_pub;

mavros_msgs::GlobalPositionTarget target_pub;
std_msgs::Float64 gimbal;

int xmin, ymin, xmax, ymax;
float tilt = 240; //Center: 303, -32.1 deg: 240 
float cur_lat, cur_lon, cur_alt;
float target_lat, target_lon, target_alt, target_yaw;
float home_alt;

int tilt_count = 0, tilt_end = 0;
int yaw_direction;
int yaw_stability = 0;
int yaw_count = 0;
int step = 0;
int target_detection = 0;

/*first target color*/
int lower_H = 115, upper_H = 125;

cv::Scalar lower_color(lower_H, 130, 130);
cv::Scalar upper_color(upper_H, 255, 255);
int color_count = 0;

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  cur_lat = msg->latitude;
  cur_lon = msg->longitude;
  cur_alt = msg->altitude;
}

void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg){
  home_alt = msg->geo.altitude;
}

void position_publish(float target_lat, float target_lon, float target_alt, float target_yaw){
    target_pub.header.stamp = ros::Time::now();
    target_pub.latitude = target_lat;
    target_pub.longitude = target_lon;
    target_pub.altitude = target_alt;
    target_pub.yaw = target_yaw;
    target_contol_pub.publish(target_pub);  
}

int sw = 0;
ros::Time time_init;
ros::Duration five_seconds(5.0);

void setPoint(float cur_lat, float cur_lon, float cur_alt){
  float yaw_rate = 0.003573 * 2.68; //{2.86 deg/s} / 14Hz
  float rel_alt = cur_alt - home_alt;
  /*step 0*/
  if(step == 0){
    //wp2
    target_lat = 37.565350;
    target_lon = 126.626778;
    target_alt = 100;
    target_yaw += yaw_direction * yaw_rate;

    if((pow((cur_lat-target_lat)/0.00001129413,2) + pow((cur_lon-target_lon)/0.00000895247,2) + pow((rel_alt - target_alt),2)) < 1){
      if(yaw_stability == 1){
        step = 1;
      }
      else{
        if(target_detection == 0){
          lower_color.val[0]--; 
          upper_color.val[0]++;
          color_count++;
          if(color_count == 5){
            color_count = 0;
            lower_color.val[0] = lower_H; 
            upper_color.val[0] = upper_H;
            lower_color.val[1] = lower_color.val[1] - 5;
            lower_color.val[2] = lower_color.val[2] - 5;
          }
        }
      }
      if((lower_color.val[1] <= 110) && (color_count == 4)){
        if(sw == 0){
          sw = 1;
          time_init = ros::Time::now();
        }
        ros::Time time_fin = ros::Time::now();
        if((time_fin - time_init) > five_seconds){
          step = 1;
        }
      }
    }
    
    position_publish(target_lat, target_lon, target_alt, target_yaw);
  }

  /*step 1*/
  if(step == 1){
    target_yaw += yaw_direction * yaw_rate;

    if((tilt_end == 0) && (pow((cur_lat-target_lat)/0.00001129413,2) + pow((cur_lon-target_lon)/0.00000895247,2) + pow((rel_alt - target_alt),2)) < 1){
      target_lat = 37.565350;
      target_lon = 126.626778;
      target_alt -= 5;
    }
    else if(tilt_end == 1){
      step = 2;

      target_lat = 37.565350;
      target_lon = 126.626778;
      target_alt = rel_alt;
    }

    else if((target_detection == 0) && (rel_alt == 20)){
      step = 2;
      target_lat = 37.565350;
      target_lon = 126.626778;
      target_alt = 20;
    }

    position_publish(target_lat, target_lon, target_alt, target_yaw);
  }

  /*step 2*/
  if(step == 2){
    //wp3
    target_lat = 37.564700;
    target_lon = 126.627628;
    target_yaw += yaw_direction * yaw_rate;
    if((pow((cur_lat-target_lat)/0.00001129413,2) + pow((cur_lon-target_lon)/0.00000895247,2) + pow((rel_alt - target_alt),2)) < 1){
      step = 3;
    }

    position_publish(target_lat, target_lon, target_alt, target_yaw);
  }

  /*step 3*/
  if(step == 3){
    //wp3
    target_lat = 37.564700;
    target_lon = 126.627628;
    target_alt = 2;

    position_publish(target_lat, target_lon, target_alt, target_yaw);
  }
  std::cout<<"step: "<<step<<std::endl;
  std::cout<<"target_lat: "<<target_lat<<std::endl;
  std::cout<<"target_lon: "<<target_lon<<std::endl;
  std::cout<<"target_alt: "<<target_alt<<std::endl;
  std::cout<<"target_yaw: "<<target_yaw<<std::endl;
  std::cout<<"HSV: "<<lower_color<<"\n"<<upper_color<<std::endl;
  std::cout<<"rotation: "<<yaw_stability<<std::endl;
  std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
}

void setGimbal(float tilt){
  if(step == 0){
    gimbal.data = 240;
    gimbal_pub.publish(gimbal);
  }
  else if(step == 3){
    gimbal.data = 303;
    gimbal_pub.publish(gimbal);
  }
  else{
    gimbal.data = tilt;
    gimbal_pub.publish(gimbal);
  }
}

void TargetCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr->image;
  cv::Mat filtered_img;
  int center_x, center_y, left, top, width, height;
     
  cv::cvtColor(img, filtered_img, cv::COLOR_BGR2HSV);
    
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
  
  cv::inRange(filtered_img, lower_color, upper_color, filtered_img);


  cv::erode(filtered_img, filtered_img, kernel, cv::Point(-1,-1), 1);
  cv::dilate(filtered_img, filtered_img, kernel, cv::Point(-1,-1), 5);

  cv::Mat labels, stats, centroids;
  int n = cv::connectedComponentsWithStats(filtered_img, labels, stats, centroids, 8);

  int area_max = 0;
  int i_max = 0;
  for (int i = 1; i<n; i++){
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if ((area_max < area) && (area > 49)){
      area_max = area;
      i_max = i;
    }
  }
  std::cout<<area_max<<std::endl;
  center_x = centroids.at<double>(i_max,0);
  center_y = centroids.at<double>(i_max,1);
  left = stats.at<int>(i_max,cv::CC_STAT_LEFT);
  top = stats.at<int>(i_max,cv::CC_STAT_TOP);
  width = stats.at<int>(i_max,cv::CC_STAT_WIDTH);
  height = stats.at<int>(i_max,cv::CC_STAT_HEIGHT);

  /*target detection*/
  if((width == 1280) && (height == 720)){
    target_detection = 0;
  }
  else{
    target_detection = 1;
  }
  std::cout<<"target_detection: "<<target_detection<<std::endl;
  
  /*camera control*/
  // mininum 4096 * 5% = 204.8
  // maximum 4096 * 10% = 409.6
  float P_tilt = 0.005;

  /* tilt calculation */
  float d_tilt = P_tilt * (360 - center_y);

  if((360 - center_y) > 54 || (360 - center_y) < -54){
    tilt = tilt + d_tilt;
  }

  if(tilt <= 210){
    tilt = 210;
  }

  if(tilt >= 303){
    tilt = 303;
    if(step == 1){
      tilt_count++;
    }
    if(tilt_count >= 30){
      tilt_end = 1;
    }
  }

  else{
    tilt_count = 0;
  }

  std::cout<<"tilt_end: "<<tilt_end<<std::endl;
   
  /* yaw direction and rotation */
  if((target_detection == 1) && ((640 - center_x) > 96)){
    //rotate left
    yaw_direction = 1;
    yaw_stability = 0;
    yaw_count = 0;
  }
  else if((target_detection == 1) && ((640 - center_x) < -96)){
    //rotate right
    yaw_direction = -1;
    yaw_stability = 0;
    yaw_count = 0;
  }
  else if((target_detection == 1) && ((-96 <= (640 - center_x)) && ((640 - center_x) <= 96))){
    //no rotation
    yaw_direction = 0;
    yaw_count++;

    if(yaw_count > 30){
      yaw_stability = 1;
    }
  }
  else if(target_detection == 0){
    yaw_direction = 0;
    yaw_stability = 0;
    yaw_count = 0;
  }
  
  /*set point*/
  setGimbal(tilt);
  setPoint(cur_lat, cur_lon, cur_alt);

  if(target_detection == 1){ 
    cv::rectangle(img, cv::Point(left,top), cv::Point(left+width,top+height), cv::Scalar(66,255,5),3);
  }
  cv::imshow("img", img);
  cv::waitKey(1);
}

int main(int argc, char **argv){
  cv::namedWindow("img");
  cv::moveWindow("img", 20,20);

  ros::init(argc, argv, "riseq_sacc_target_detection");
  ros::NodeHandle n1, n2, n3, n4, n5;
  
  /*first position*/
  target_lat = 37.565350;
  target_lon = 126.626778;
  target_alt = 100;
  target_yaw = 5.85367;  
  position_publish(target_lat, target_lon, target_alt, target_yaw);

  gimbal_pub = n1.advertise<std_msgs::Float64>("/gimbal_control", 1);
  target_contol_pub = n2.advertise<mavros_msgs::GlobalPositionTarget>("/setpoint_target",1);

  ros::Subscriber gps_sub = n3.subscribe("/mavros/global_position/global",1, gpsCallback);
  ros::Subscriber HomePosition_sub = n4.subscribe("/mavros/home_position/home",1, HomePositionCallback);  

  image_transport::ImageTransport it(n5);    
  image_transport::Subscriber target_zed_image_sub = it.subscribe("/zed/zed_node/rgb/image_rect_color", 1, TargetCallback);

  if (!ros::ok()){
      std_msgs::Float64 gimbal;
      gimbal.data = 303;
      gimbal_pub.publish(gimbal);

      cv::destroyAllWindows();
      ros::shutdown();
  }
  ros::spin();
  return 0;
}
