//
// Created by haeyeon on 20. 6. 4..
//

#include "count_point.h"


int distance;
double length;
double width1;
double width2;
double height;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

class CountPoints
{
public:
    CountPoints() {
        this->subscriber = this->nh.subscribe("/scan_rp", 10, &CountPoints::PointsInBounary, this);
        this->publisher = this->nh.advertise<sensor_msgs::PointCloud2> ("points_inside", 10);
        this->publisher2 = this->nh.advertise<std_msgs::Int32> ("num_points", 10);
    };

    void PointsInBounary(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
        sensor_msgs::PointCloud2 cloud_msg2;
        
        pcl::PointXYZ origin(distance, 0, 0);
        sensor_msgs::PointCloud2 points_out;
        std_msgs::Int32 num_points;

        PointCloudPtr cloud(new PointCloud); 
        PointCloud2Ptr cloud2(new PointCloud2);
        PointCloud2Ptr cloud_out(new PointCloud2);
        
        PointCloud result;
        projector_.projectLaser(*scan_in, cloud_msg2);
        pcl_conversions::toPCL(cloud_msg2, *cloud2); // save cloud message to cloud2
        pcl::fromPCLPointCloud2(*cloud2, *cloud); 
        pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree_->setInputCloud(cloud);

        std::vector<int> nn_indices(5000);
        std::vector<float> nn_dists(5000);

        tree_->nearestKSearch(origin, 5000, nn_indices, nn_dists);
        for (int i = 0; i < nn_indices.size(); i++) {
            bool inside_x = cloud->points[nn_indices[i]].x < distance + length/2 && cloud->points[nn_indices[i]].x > distance - length/2;
            bool inside_y = cloud->points[nn_indices[i]].y < width1 && cloud->points[nn_indices[i]].y > width2;
            bool inside_z = cloud->points[nn_indices[i]].z > height;
            if(inside_x && inside_y && inside_z) {
                result.push_back(cloud->points[nn_indices[i]]);
            }
            //result.push_back(cloud->points[nn_indices[i]]);
        }
        pcl::toROSMsg(result, points_out);
        points_out.header.frame_id = "laser";
        num_points.data = result.size();
        //cout<< result.size();
        this->publisher.publish(points_out);
        this->publisher2.publish(num_points);
    }
    ~CountPoints(){
    }
    private:
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
        ros::Publisher publisher2;
        laser_geometry::LaserProjection projector_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    distance = atoi(argv[1]);
    length = atof(argv[2]);
    width1 = atof(argv[3]);
    width2 = atof(argv[4]);
    height = atof(argv[5]);
    CountPoints CountPoints;
    while(ros::ok()) {
         ros::spinOnce();
    } 
    return 0;
}