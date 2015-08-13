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

#include <map>
#include <string>
#include "hector_navigation/Map.h"
#include "hector_navigation/Coordinate.h"
#include "hector_navigation/StructKeyMap.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
//using namespace std;

class LocalMap
{

public:
    LocalMap()
    {
    
     sub_filter = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("FilteredPoints", 1, &LocalMap::callback,this);
     sub_map = nh.subscribe<hector_navigation::Map>("Local_Map", 1, &LocalMap::callback_map,this);
     map_pub = nh.advertise<hector_navigation::Map>("Local_Map", 1);
     //pub_CubeList = nh.advertise<visualization_msgs::Marker>("CubeList", 1);
     //pub_MapOutline = nh.advertise<visualization_msgs::Marker>("MapOutline", 1);
   }

    void callback_map(const hector_navigation::Map::ConstPtr& inputMap);
    void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloud );

    hector_navigation::Map local_map; 
    typedef std::map<position,int> map_;

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_filter;
    ros::Subscriber sub_map;
    ros::Publisher map_pub;
    //ros::Publisher pub_CubeList;
    //ros::Publisher pub_MapOutline;
};


void LocalMap::callback_map(const hector_navigation::Map::ConstPtr& inputMap)
{

   local_map.points=inputMap->points;
   local_map.cv=inputMap ->cv;

}

void LocalMap::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloud )
{
    tf::TransformListener listener;
    tf::StampedTransform transform2;
    listener.waitForTransform("world", "base_link", ros::Time(0),ros::Duration(3.0));
    listener.lookupTransform("world", "base_link", ros::Time(0), transform2);
    double x= round(transform2.getOrigin().x()/0.1);//UAV location in gloabl 3D grid frame
    double y= round(transform2.getOrigin().y()/0.1);
    double z= round(transform2.getOrigin().z()/0.1); 

    /*visualization_msgs::Marker  line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "Outline";
    line_list.id = 1;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.scale.x = 0.005;
    line_list.color.r = 1.0f;
    line_list.color.g = 0.5f;
    line_list.color.a = 1.0;

    geometry_msgs::Point vertex;
    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);//12

    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);//23

    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);//34

    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);//41

    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);//48

    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);//87

    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);//73

    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);//58

    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);//56

    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05+2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);//67

    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05+2.9;
    line_list.points.push_back(vertex);//51

    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05+2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);
    vertex.x=x*0.1-0.05-2.9;
    vertex.y=y*0.1-0.05-2.9;
    vertex.z=z*0.1-0.05-2.9;
    line_list.points.push_back(vertex);//62

    pub_MapOutline.publish(line_list);*/

//Delete far away COR in local_map (std::map structure) project
    position P1;
    position P2;
    position P3;
    map_ tep_map;
    int map_size = local_map.cv.size();
    for(int i=0; i<map_size; i++)
   {  
         P1.x=local_map.points[i].x;
         P1.y=local_map.points[i].y;
         P1.z=local_map.points[i].z;
         if ((P1.x >=(x-29) && P1.x<=(x+29)) && (P1.y >=(y-29) && P1.y<=(y+29)) && (P1.z >=(z-29) && P1.z<=(z+29)))
	   {
	    tep_map[P1]=local_map.cv[i];
            //ROS_INFO_STREAM("!!!!!!!"); 
	   }    
   }

// add new COR into local_map (std::map structure) project 

     int cloudsize = (inputCloud -> width) * (inputCloud -> height);
     double x_;
     double y_;
     double z_;
     for( int j=0; j<cloudsize; j++)
     {  

        x_= round((inputCloud ->points[j].x)/0.1);
        y_= round((inputCloud ->points[j].y)/0.1);
        z_= round((inputCloud ->points[j].z)/0.1);

        if ((x_ >=(x-29) && x_<=(x+29)) && (y_ >=(y-29) && y_<=(y+29)) && (z_ >=(z-29) && z_<=(z+29)))
	   {
	    P2.x=x_;
	    P2.y=y_;
	    P2.z=z_;

            int value =tep_map[P2];
            if (value <20)
		{
		  value++;
		}
            tep_map[P2]=value;

	   }
     }

  // convert local_map project into (std::vector structure) to publish
     /*visualization_msgs::Marker cube_list;
     cube_list.header.frame_id = "world";
     cube_list.header.stamp = ros::Time::now();
     cube_list.ns = "Cubes";
     cube_list.id = 2;
     cube_list.type = visualization_msgs::Marker::CUBE_LIST;
     cube_list.action = visualization_msgs::Marker::ADD;
     cube_list.pose.orientation.w = 1.0;
     cube_list.scale.x = 0.1f;
     cube_list.scale.y = 0.1f;
     cube_list.scale.z = 0.1f;
     cube_list.color.r = 1.0f;
     cube_list.color.g = 0.5f;
     cube_list.color.a = 1.0;
     cube_list.lifetime = ros::Duration();*/

      hector_navigation::Coordinate coord;
      map_::iterator iter;
      for (iter = tep_map.begin(); iter != tep_map.end();++iter)
      {
        P3=  iter->first;
        coord.x=P3.x;
        coord.y=P3.y;
        coord.z=P3.z;
        if ((iter->second)==20)
        {
            geometry_msgs::Point temp;
            temp.x = coord.x*0.1-0.05;
            temp.y = coord.y*0.1-0.05;
            temp.z = coord.z*0.1-0.05;
            //cube_list.points.push_back(temp);
	}

        local_map.points.push_back(coord);
        local_map.cv.push_back(iter->second);
      }

      //pub_CubeList.publish(cube_list);
      local_map.header.stamp=ros::Time::now();
      map_pub.publish(local_map);


}



int main(int argc, char **argv)
{
     ros::init(argc, argv, "local_map_node");

     LocalMap  localproject;

     ros::spin();
   

     return 0;
}



