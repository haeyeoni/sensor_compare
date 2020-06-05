//
// Created by haeyeon on 20. 6. 4..
//

#include "count_point.h"
#include <ros/ros.h>
#include <sstream>


int distance = 3;
double length = 1;
double width = 1;
double height = -0.5;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

class CountPoints
{
public:
    CountPoints() {
        this->subscriber = this->nh.subscribe("/velodyne_points", 10, &CountPoints::PointsInBounary, this);
        this->publisher = this->nh.advertise<sensor_msgs::PointCloud2> ("points_inside", 10);
    };

    void PointsInBounary(const sensor_msgs::PointCloud2 &cloud_msg) {
        sensor_msgs::PointCloud2 points_out;
        pcl::PointXYZ origin(distance, 0, 0);

        PointCloudPtr cloud(new PointCloud); 
        PointCloud2Ptr cloud2(new PointCloud2);
        PointCloud2Ptr cloud_out(new PointCloud2);
        
        PointCloud result;
        pcl_conversions::toPCL(cloud_msg, *cloud2); // save cloud message to cloud2
        pcl::fromPCLPointCloud2(*cloud2, *cloud); 
        pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree_->setInputCloud(cloud);

        std::vector<int> nn_indices(5000);
        std::vector<float> nn_dists(5000);

        tree_->nearestKSearch(origin, 5000, nn_indices, nn_dists);
        for (int i = 0; i < nn_indices.size(); i++) {
            bool inside_x = cloud->points[nn_indices[i]].x < distance + length/2 && cloud->points[nn_indices[i]].x > distance - length/2;
            bool inside_y = cloud->points[nn_indices[i]].y < width/2 && cloud->points[nn_indices[i]].y > -width/2;
            bool inside_z = cloud->points[nn_indices[i]].z > height;
            if(inside_x && inside_y && inside_z) {
                result.push_back(cloud->points[nn_indices[i]]);
            }
            //result.push_back(cloud->points[nn_indices[i]]);
        }
        pcl::toROSMsg(result, points_out);
        points_out.header.frame_id = "velodyne";
        this->publisher.publish(points_out);
    }
    ~CountPoints(){
    }
    private:
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    CountPoints CountPoints;
    while(1) {
        ros::spinOnce();
    }
}
