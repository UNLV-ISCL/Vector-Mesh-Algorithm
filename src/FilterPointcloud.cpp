#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl_ros/filters/filter.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <limits>

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_listener.h>
#include <math.h>


using namespace std;

class FilterPointcloud
{

public:
    FilterPointcloud()
    {
    
     sub_kinect = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("camera/depth/points", 1, &FilterPointcloud::callback,this);
     point_pub= nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("FilteredPoints",1);


    }

    void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloud );

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_kinect;
    ros::Publisher point_pub;
};


void FilterPointcloud::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloud )
{

   //convert PointCLoud2 to PoiintXYZ and remove NAN data
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*inputCloud,*tempCloud1, indices);

   // reduce data amout
    pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    int counter = 0;
    for( it= tempCloud1->begin(); it!= tempCloud1->end(); it++)
   {
      if(counter%25==0)
      simpleCloud->push_back (pcl::PointXYZ (it->x, it->y, it->z)); 
      counter ++;
   }

    // transform obstacle XYZ from Kinect frame into base_link frame
    tf::Transform transform1;
    transform1.setOrigin( tf::Vector3(0.05,-0.02,0.2) );
    tf::Quaternion quat;
    quat.setRPY(-1.5707963, 0.0, -1.5707963);//roll,pitch,yaw angle 
    transform1.setRotation(quat);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*simpleCloud,*tempCloud2, transform1);


   //transform obstacle XYZ from base_link into world
    tf::TransformListener listener;
    tf::StampedTransform transform2;
    tf::Transform transform3;
    tf::Quaternion Q;
    tf::Vector3 V;

    listener.waitForTransform("world", "base_link", ros::Time(0),ros::Duration(3.0));
    listener.lookupTransform("world", "base_link", ros::Time(0), transform2);
    Q=transform2.getRotation();
    V.setX(transform2.getOrigin().x());
    V.setY(transform2.getOrigin().y());
    V.setZ(transform2.getOrigin().z());
    transform3.setOrigin(V);
    transform3.setRotation(Q);


    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*tempCloud2,*tempCloud3, transform3);

    //publish filtered pointcloud
   
    pcl::PointCloud<pcl::PointXYZ> outputCloud;
    for( it= tempCloud3->begin(); it!= tempCloud3->end(); it++)
   {
      outputCloud.points.push_back (pcl::PointXYZ (it->x, it->y, it->z)); 
   }
    point_pub.publish(outputCloud);



    //ROS_INFO("aa: %.2f",aa);
}



int main(int argc, char **argv)
{
     ros::init(argc, argv, "filter_pointcloud_node");

     FilterPointcloud  Filterproject;

     ros::spin();
   

     return 0;
}












