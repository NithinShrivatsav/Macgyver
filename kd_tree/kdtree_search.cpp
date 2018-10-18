#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>

using namespace std;

void cloud_segment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2){
    Eigen::Vector3f color_temp;
    // Pick one of the two colors in original point cloud
    color_temp[0] = cloud_color->points[0].r;
    color_temp[1] = cloud_color->points[0].g;
    color_temp[2] = cloud_color->points[0].b;

    vector<pcl::PointXYZRGBA> segment_1;
    vector<pcl::PointXYZRGBA> segment_2;

    for (int i = 0; i < cloud_color->points.size(); i++){
        pcl::PointXYZRGBA point = cloud_color->points[i];
        Eigen::Vector3f color_check;
        color_check[0] = point.r;
        color_check[1] = point.g;
        color_check[2] = point.b;

        if (color_temp == color_check){
            segment_1.push_back(point);
        }
        else{
            segment_2.push_back(point);
        }
    }

    cloud1->width = segment_1.size();
    cloud1->height = 1;
    cloud1->points.resize(cloud1->width * cloud1->height);

    cloud2->width = segment_2.size();
    cloud2->height = 1;
    cloud2->points.resize(cloud2->width * cloud2->height);

    for (int i = 0; i < cloud1->points.size(); i++){
        cloud1->points[i] = segment_1[i];
    }

    for (int i = 0; i < cloud2->points.size(); i++){
        cloud2->points[i] = segment_2[i];
    } 
}

int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_color (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2_color (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

    string filename = "XYZ.ply";

    if (pcl::io::loadPLYFile(filename.c_str(), *cloud_color) == -1){
        PCL_ERROR("Couldn't load file \n");
        return 0;
    }

    // Segment the input point cloud into parts based on color
    cloud_segment(cloud_color, cloud1_color, cloud2_color);

    copyPointCloud(*cloud1_color, *cloud1);
    copyPointCloud(*cloud2_color, *cloud2);

    // Perform kd tree search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2); // Set cloud 2 as input
    pcl::PointXYZ searchpoint;

    // Set search point as centroid of cloud 1 to compute nearest points in cloud 2
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud1, centroid);
    searchpoint.x = centroid[0];
    searchpoint.y = centroid[1];
    searchpoint.z = centroid[2];

    // Compute closest 20 points in cloud 2 and store them in point_set
    int K = 20;
    vector<int> pointIdxNKNSearch(K);
    vector<float> pointNKNSquaredDistance(K);

    vector<pcl::PointXYZ> point_set;

    if ( kdtree.nearestKSearch (searchpoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
            point_set.push_back(cloud2->points[pointIdxNKNSearch[i]]);
        }
    }
    else{
        cout << "No points found" << endl;
    }

    // Compute the centroid of intersection 
    pcl::CentroidPoint<pcl::PointXYZ> kd_centroid;
    for (int i = 0; i < point_set.size(); i++){
        kd_centroid.add(point_set[i]);
    }

    pcl::PointXYZ intersection;
    kd_centroid.get(intersection);

    cout << intersection.x << ", " << intersection.y << ", " << intersection.z << endl;
    return 0;

}