/**
@author  Eugene Auh
@version 0.1.0
@brief This node is for apart and cluster several obstacles(e.g. poles, trees) at AI R&D Grand Challenge.
       Using PCL library, process Euclidean clustering and plane segmentation so that can detect obstacles from stereo depth.

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include <ros/ros.h>
// PCL specific includes. PCL <-> PointCloud2(ROS message)
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/// Including PCL library to work with PointCloud
/// http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

typedef pcl::PointXYZ PointT;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);

  // Data containers used
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud.makeShared());
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.01); // Setting the tolerance to cluster. unit: m
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
 
  std::cout << "Number of clusters is equal to " << cluster_indices.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZI> TotalCloud; 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end(); ++pit)
    {
        pcl::PointXYZ pt = cloud_filtered->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j + 1);

            TotalCloud.push_back(pt2);
    }
    j++;
  }

  // Convert to ROS data type 
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);

  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "map";   
  pub.publish(output); 

  ROS_INFO("published it."); 
}


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "riseq_perception");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("pclplaneoutput", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/riseq/perception/cluster", 10);

  while (ros::ok())
  {
    // Spin
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
