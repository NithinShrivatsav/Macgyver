#include<iostream>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/features/shot.h>
#include<pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/keypoints/sift_keypoint.h>
#include<pcl/common/centroid.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/common/transforms.h>
#include<pcl/common/pca.h>
#include <pcl/common/common.h>
#include<fstream>
#include<string.h>
#include<stdlib.h>
#include<dirent.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<sys/types.h>


using namespace std;

int main(int argc, char** argv)
{
    DIR *dp;
    struct dirent *dirp;
    struct stat filestat;
    string filepath, folder, stringsub;
    string main_folder, subfolder;
    size_t pos_1 = 0;
    size_t pos_2 = 0;
    size_t pos_3 = 0;
    size_t pos_4 = 0;
    size_t pos_5 = 0;
    size_t pos_6 = 0;
    size_t pos_7 = 0;
    int j = 1;

    ofstream shotc_file;
    shotc_file.open("shotc_descriptors_final.csv");
    shotc_file<<"Function"<<","<<"S.No"<<","<<"Object Type"<<","<<"Names"<<","<<"Label"<<",";
    for(int a = 1; a<=352; a++)
        {
            shotc_file<<"Feature"<<a<<",";
        }
    shotc_file<<"\n";


    main_folder = "/home/nithin/Desktop/ThreeDCV/Dataset";
    string type_tool[6] = {"CONTAIN", "CUT", "FLIP", "HIT", "POKE", "SCOOP"};

    for(int k=0; k<6; k++)
    {
        subfolder = type_tool[k];
        folder = main_folder + "/" + subfolder;

        dp = opendir(folder.c_str());
        if(dp == NULL)
            {
                return -1;
            }
        while(dirp = readdir(dp))
        {
            int label_handle = 0;
            int label_2 = 2;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
            pcl::PointXYZ c;
            pcl::CentroidPoint<pcl::PointXYZ> centroid;

            filepath = folder + "/" + dirp->d_name;
            if (stat( filepath.c_str(), &filestat )) continue;
            if (S_ISDIR( filestat.st_mode ))         continue;
            pos_1 = filepath.find("contain");
            pos_2 = filepath.find("handle");
            pos_3 = filepath.find("blade");
            pos_4 = filepath.find("spatula.ply");
            pos_5 = filepath.find("head");
            pos_6 = filepath.find("tip");
            pos_7 = filepath.find("scoop");
            if(pos_1!=-1)
            {
                stringsub = filepath.substr(pos_1);
                label_2 = label_2 * (k+1);
            }
            else if(pos_2!=-1)
            {
                stringsub = filepath.substr(pos_2);
                label_handle = 2*(k+1) - 1;
            }
            else if(pos_3!=-1)
            {
                stringsub = filepath.substr(pos_3);
                label_2 = label_2 * (k+1);
            }
            else if(pos_4!=-1)
            {
                stringsub = filepath.substr(pos_4);
                label_2 = label_2 * (k+1);
            }
            else if(pos_5!=-1)
            {
                stringsub = filepath.substr(pos_5);
                label_2 = label_2 * (k+1);
            }
            else if(pos_6!=-1)
            {
                stringsub = filepath.substr(pos_6);
                label_2 = label_2 * (k+1);
            }
            else if(pos_7!=-1)
            {
                stringsub = filepath.substr(pos_7);
                label_2 = label_2 * (k+1);
            }
            cout<<stringsub<<endl;
            shotc_file<<subfolder<<","<<j<<","<<stringsub<<",";
            j++;
            shotc_file<<dirp->d_name<<",";
            if(label_handle!=0)
            {
                shotc_file<<int(label_handle)<<",";
            }
            else
            {
                shotc_file<<int(label_2)<<",";
            }
            if (pcl::io::loadPLYFile<pcl::PointXYZ> (filepath, *cloud)==-1)
            {
                PCL_ERROR("couldn't read file\n");
                return(-1);
            }
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cloud, pcaCentroid);
            Eigen::Matrix3f covariance;
            computeCovarianceMatrixNormalized(*cloud,pcaCentroid,covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
            eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
            projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
            // Get the minimum and maximum points of the transformed cloud.
            pcl::PointXYZ minPoint;
            pcl::PointXYZ maxPoint;
            pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
            const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

            float x, y, z;

            x = maxPoint.x - minPoint.x;
            y = maxPoint.y - minPoint.y;
            z = maxPoint.z - minPoint.z;

            float radius = 0;
            if(x>y)
            {
                radius = x;
            }
            else
            {
                radius = y;
            }
            if(z>radius)
            {
                radius = z;
            }

            cout<<endl<<"x= "<<x<<" y= "<<y<<" z= "<<z<<endl;
            cout<<"Display Max Radius"<<radius;
            cout<<endl<<"x= "<<x<<" y= "<<y<<" z= "<<z<<endl;
            cout<<"Display Max Radius"<<radius;



            for(size_t i=0;i<cloud->points.size();i++)
            {
                centroid.add(cloud->points[i]);
            }
            centroid.get(c);
            cout<<endl<<"Centroid"<<c<<endl;



            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
            normalEstimation.setInputCloud(cloud);
            normalEstimation.setRadiusSearch(radius);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
            normalEstimation.setSearchMethod(kdtree);
            normalEstimation.compute(*normals);

            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            new_cloud->push_back(c);
            pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
            shot.setSearchMethod(kdtree);
            shot.setSearchSurface(cloud);
            shot.setInputCloud(new_cloud);
            shot.setInputNormals(normals);
            shot.setRadiusSearch(radius);
            shot.compute(*descriptors);

            cout<<"Global SHOT output points.size "<<descriptors->points.size()<<endl;
            cout<<"Original cloud points.size "<<cloud->points.size()<<endl;
            //global shot descriptor for first point
            cout<<"Descriptor Points "<<descriptors->points[0]<<endl;




            for(int m=0;m<352;m++)
            {
                shotc_file<<descriptors->points[0].descriptor[m]<<",";
            }
            shotc_file<<"\n";
        }
            closedir(dp);
    }

        shotc_file.close();
        return 0;
}


    /*
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
pcl::PointXYZ c;
pcl::CentroidPoint<pcl::PointXYZ> centroid;

ofstream shotc_file;
shotc_file.open("shotc_descriptors_plate_2_3dwh1_contain.csv");

if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/nithin/Desktop/ThreeDCV/Dataset/CONTAIN/plate_2_3dwh1_contain.ply", *cloud)==-1)
{
    PCL_ERROR("couldn't read file\n");
    return(-1);
}

// Calculation of radius of object
Eigen::Vector4f pcaCentroid;
pcl::compute3DCentroid(*cloud, pcaCentroid);
Eigen::Matrix3f covariance;
computeCovarianceMatrixNormalized(*cloud,pcaCentroid,covariance);
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
// Get the minimum and maximum points of the transformed cloud.
pcl::PointXYZ minPoint;
pcl::PointXYZ maxPoint;
pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

float x, y, z;

x = maxPoint.x - minPoint.x;
y = maxPoint.y - minPoint.y;
z = maxPoint.z - minPoint.z;

float radius = 0;
if(x>y)
{
    radius = x;
}
else
{
    radius = y;
}
if(z>radius)
{
    radius = z;
}

cout<<endl<<"x= "<<x<<" y= "<<y<<" z= "<<z<<endl;
cout<<"Display Max Radius"<<radius;



for(size_t i=0;i<cloud->points.size();i++)
{
    centroid.add(cloud->points[i]);
}
centroid.get(c);
cout<<endl<<"Centroid"<<c<<endl;



pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
normalEstimation.setInputCloud(cloud);
normalEstimation.setRadiusSearch(radius);
pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
normalEstimation.setSearchMethod(kdtree);
normalEstimation.compute(*normals);

pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
new_cloud->push_back(c);
pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
shot.setSearchMethod(kdtree);
shot.setSearchSurface(cloud);
shot.setInputCloud(new_cloud);
shot.setInputNormals(normals);
shot.setRadiusSearch(radius);
shot.compute(*descriptors);

cout<<"Global SHOT output points.size "<<descriptors->points.size()<<endl;
cout<<"Original cloud points.size "<<cloud->points.size()<<endl;
//global shot descriptor for first point
cout<<"Descriptor Points "<<descriptors->points[0]<<endl;




for(int m=0;m<352;m++)
{
    shotc_file<<descriptors->points[0].descriptor[m]<<",";
}
shotc_file<<"\n";

shotc_file.close();
return 0;

}
*/

