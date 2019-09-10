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
#include <math.h>

float lat, lon, alt, heading;

float side_thr = 255/5;
float updown_thr = 255/2;
float front_thr = 255/15;

int width = 640;
int height = 360;
int width_13 = 213;
int width_23 = 427;
int height_13 = 120;
int height_23 = 240;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

cv::Rect left_box(0, height_13, width_13, 120);
cv::Rect right_box(width_23, height_13, 640 - width_23 , 120);
cv::Rect up_box(0, 0, width, 120);
cv::Rect down_box(0, height_23, width, 120);
cv::Rect front_box(width_13, height_13, width_23 - width_13, height_23 - height_13);

geometry_msgs::PoseStamped move;

void DepthCallback(const sensor_msgs::Image::ConstPtr& msg) {
   cv_bridge::CvImagePtr cv_ptr;
   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
   cv::Mat img = cv_ptr->image;
   cv::Mat thr_img;

   cv::threshold(img, thr_img, 100, 255, cv::THRESH_BINARY_INV);

   thr_img.convertTo(thr_img, CV_8UC1);

   cv::resize(thr_img, thr_img, cv::Size(640,360), 0, 0, CV_INTER_NN);
   cv::Mat display_img = thr_img;
   cv::cvtColor(display_img, display_img, cv::COLOR_GRAY2BGR);

   /* test depth distance
   std::cout<<thr_img.at<uchar>(640,360)<<std::endl;
   std::cout<<img.at<float>(640,360)<<std::endl;
   cv::circle(thr_img, cv::Point(640,360), 3, cv::Scalar(0,0,255), 1);
   */

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

   move.pose.position.x = 0;
   move.pose.position.y = 0;
   move.pose.position.z = 0;

   if(front_obstacle == true){
      cv::circle(display_img, cv::Point(int(width/2), int(height/2)), 10, cv::Scalar(0,0,255),-1);
      if (left_obstacle && right_obstacle && up_obstacle && down_obstacle){
         // stop, alt up
         cv::arrowedLine(display_img, cv::Point(int(width/2), height-10),cv::Point(int(width/2),height_23), cv::Scalar(0,0,255), 3);
      }
      else{
         if ((left_obstacle == false ) && (right_obstacle == false)){
            if (left > right){
               // stop, right
               cv::arrowedLine(display_img, cv::Point(10, int(height/2)),cv::Point(width_13,int(height/2)), cv::Scalar(0,0,255), 3);
               move.pose.position.y -= 0.1;
            }
            else{
               // stop, left
               cv::arrowedLine(display_img, cv::Point(width-10, int(height/2)),cv::Point(width_23,int(height/2)), cv::Scalar(0,0,255), 3);
               move.pose.position.y += 0.1;
            }
         }
         if ((left_obstacle == true) || (right_obstacle == true)){
            if((left_obstacle == true) && (right_obstacle == true)){
            }
            else if (left > right){
               //stop, right
               cv::arrowedLine(display_img, cv::Point(10, int(height/2)),cv::Point(width_13,int(height/2)), cv::Scalar(0,0,255), 3);
               move.pose.position.y -= 0.1;
            }
            else{
               //stop, left
               cv::arrowedLine(display_img, cv::Point(width-10, int(height/2)),cv::Point(width_23,int(height/2)), cv::Scalar(0,0,255), 3);
               move.pose.position.y += 0.1;
            }
         }
         if ((up_obstacle == false) && (down_obstacle == false)){
            if (down > up){
               //stop, up
               cv::arrowedLine(display_img, cv::Point(int(width/2), height-10),cv::Point(int(width/2),height_23), cv::Scalar(0,0,255), 3);
               move.pose.position.z += 0.1;
            }
            else{
               //stop, down
               cv::arrowedLine(display_img, cv::Point(int(width/2), 10),cv::Point(int(width/2),height_13), cv::Scalar(0,0,255), 3);
               move.pose.position.z -= 0.1;
            }
         }
         if ((up_obstacle == true) || (down_obstacle == true)){
            if ((up_obstacle == true) && (down_obstacle == true)){
            }
            else if (up > down){
               //stop,down
               cv::arrowedLine(display_img, cv::Point(int(width/2), 10),cv::Point(int(width/2),height_13), cv::Scalar(0,0,255), 3);
               move.pose.position.z -= 0.1;
            }
            else{
               //stop,up
               cv::arrowedLine(display_img, cv::Point(int(width/2), height-10),cv::Point(int(width/2),height_23), cv::Scalar(0,0,255), 3);
               move.pose.position.z += 0.1;
            }
         }
      }
   }
   else if((left_obstacle || right_obstacle || up_obstacle || down_obstacle || front_obstacle) == false){
      //wp4
      cv::circle(display_img, cv::Point(int(width/2), int(height/2)), 10, cv::Scalar(0,255,0),-1);
   }
   else{
      cv::circle(display_img, cv::Point(int(width/2), int(height/2)), 10, cv::Scalar(0,225,0),-1);

      if ((left_obstacle && right_obstacle && up_obstacle && down_obstacle)){
         //go
         cv::arrowedLine(display_img, cv::Point(int(width/2), height-75),cv::Point(int(width/2),height_23), cv::Scalar(0,255,0), 3);
         move.pose.position.x += 0.1;
      }
      else{
         if ((left_obstacle == 1) || (right_obstacle == 1)){
            if((left_obstacle == true) && (right_obstacle == true)){
            }
            else if (left > right){
               // go, right
               cv::arrowedLine(display_img, cv::Point(75, int(height/2)),cv::Point(width_13,int(height/2)), cv::Scalar(0,255,0), 3);
               move.pose.position.x += 0.1;
               move.pose.position.y -= 0.1;
            }
            else{
               // go, left
               cv::arrowedLine(display_img, cv::Point(width-75, int(height/2)),cv::Point(width_23,int(height/2)), cv::Scalar(0,255,0), 3);
               move.pose.position.x += 0.1;
               move.pose.position.y += 0.1;
            }
         }
         if ((up_obstacle == 1) || (down_obstacle == 1)){
            if ((up_obstacle == true) && (down_obstacle == true)){
            }
            else if (up > down){
               //go, down
               cv::arrowedLine(display_img, cv::Point(int(width/2), 75),cv::Point(int(width/2),height_13), cv::Scalar(0,255,0), 3);
               move.pose.position.x += 0.1;
               move.pose.position.z -= 0.1;
            }
            else{
               //go, up
               cv::arrowedLine(display_img, cv::Point(int(width/2), height-75),cv::Point(int(width/2),height_23), cv::Scalar(0,255,0), 3);
               move.pose.position.x += 0.1;
               move.pose.position.z += 0.1;
            }
         }
      }
   }

   cv::line(display_img, cv::Point(0,height_13),cv::Point(width, height_13), cv::Scalar(0,0,255),1);
   cv::line(display_img, cv::Point(0,height_23),cv::Point(width, height_23), cv::Scalar(0,0,255),1);
   cv::line(display_img, cv::Point(width_13,height_13),cv::Point(width_13,height_23), cv::Scalar(0,0,255),1);
   cv::line(display_img, cv::Point(width_23,height_13),cv::Point(width_23,height_23), cv::Scalar(0,0,255),1) ;
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
    ros::init(argc, argv, "riseq_sacc_obstacle_avoidance");

    // make ros handle for off board
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // make ros handle for openCV
    ros::NodeHandle n1, n2, n3, n4;
    image_transport::ImageTransport it(n1);
    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

    image_transport::Subscriber Depth  = it.subscribe("/camera/depth/image_raw", 1, DepthCallback);
    ros::Subscriber gps_sub = n2.subscribe("/mavros/global_position/global",1, gpsCallback);
    ros::Subscriber heading_sub = n3.subscribe("/mavros/global_position/compass_hdg",1, headingCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        pose.pose.position.x += move.pose.position.x * 0.1;
        pose.pose.position.y += move.pose.position.y * 0.1;
        pose.pose.position.z += move.pose.position.z * 0.1;

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }



    if (!ros::ok()){
        cv::destroyAllWindows();
        ros::shutdown();
    }
    ros::spin();
    return 0;

}