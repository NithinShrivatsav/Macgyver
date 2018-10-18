#include<iostream>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/features/shot.h>
#include<pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_plotter.h>
#include <pcl/keypoints/sift_keypoint.h>
#include<pcl/common/centroid.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<fstream>
#include<dirent.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
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
    ofstream shota_file;
    shota_file.open("shota_descriptors_final.csv");
    shota_file<<"Function"<<","<<"S.No"<<","<<"Object Type"<<","<<"Names"<<","<<"Label"<<",";
    for(int a = 1; a<=352; a++)
        {
            shota_file<<"Feature"<<a<<",";
        }
    shota_file<<"\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
    //pcl::PointCloud<pcl::SHOT352>::Ptr new_cloud(new pcl::PointCloud<pcl::SHOT352>());
    //pcl::CentroidPoint<pcl::SHOT352> average;
    //pcl::CentroidPoint<pcl::SHOT352> average;
    //pcl::PointXYZ c;
    //pcl::PointCloud<pcl::SHOT352>::Ptr a(new pcl::PointCloud<pcl::SHOT352>());
    float f[352] = {0.0};
    int j = 1;
    string type_tool[6] = {"CONTAIN", "CUT", "FLIP", "HIT", "POKE", "SCOOP"};
    main_folder = "/home/nithin/Desktop/ThreeDCV/Dataset";

    for(int k=0; k<6; k++)
    {
        subfolder = type_tool[k];
        folder = main_folder + "/" + subfolder;
        dp = opendir(folder.c_str());
        int label_handle = 0;
        int label_2 = 2;
        if(dp == NULL)
            {
                return -1;
            }
        while(dirp = readdir(dp))
        {
            int label_handle = 0;
            int label_2 = 2;
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
            shota_file<<subfolder<<","<<j<<","<<stringsub<<",";
            j++;
            shota_file<<dirp->d_name<<",";
            if(label_handle!=0)
            {
                shota_file<<int(label_handle)<<",";
            }
            else
            {
                shota_file<<int(label_2)<<",";
            }
            if (pcl::io::loadPLYFile<pcl::PointXYZ> (filepath, *cloud)==-1)
            {
                PCL_ERROR("couldn't read file\n");
                return(-1);
            }


            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
            normalEstimation.setInputCloud(cloud);
            normalEstimation.setRadiusSearch(0.03);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
            normalEstimation.setSearchMethod(kdtree);
            normalEstimation.compute(*normals);

            pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
            shot.setInputCloud(cloud);
            shot.setInputNormals(normals);
            shot.setRadiusSearch(0.02);
            shot.compute(*descriptors);

            for(size_t i=0;i<descriptors->points.size();++i)
            {

                for(int j = 0;j<352;++j)
                {
                     f[j] = f[j] + descriptors->points[i].descriptor[j];
                }
            }

            for(int k=0;k<352;k++)
            {
                f[k] = f[k]/(descriptors->points.size());
                if(pcl_isnan(f[k])>0)
                {
                    f[k] = 0.0;
                }
            }
            cout<<descriptors->points[0];
            for(int l = 0;l<352;++l)
            {
                shota_file<<f[l]<<",";
            }
            shota_file<<"\n";
        }
        closedir(dp);

    }

    shota_file.close();
    return 0;
}

/*

//single object

ofstream shota_file;
shota_file.open("shota_descriptors_spatula_2_3dwh4_spatula.csv");
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
float f[352] = {0.0};
if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/nithin/Desktop/ThreeDCV/Dataset/FLIP/spatula_2_3dwh4_spatula.ply", *cloud)==-1)
{
PCL_ERROR("couldn't read file bottle_10_3dwh.ply\n");
return(-1);
}

pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
normalEstimation.setInputCloud(cloud);
normalEstimation.setRadiusSearch(0.03);
pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
normalEstimation.setSearchMethod(kdtree);
normalEstimation.compute(*normals);

pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
shot.setInputCloud(cloud);
shot.setInputNormals(normals);
shot.setRadiusSearch(0.02);
shot.compute(*descriptors);

for(size_t i=0;i<descriptors->points.size();++i)
{

    for(int j = 0;j<352;++j)
    {
         f[j] = f[j] + descriptors->points[i].descriptor[j];
    }
}

for(int k=0;k<352;k++)
{
    f[k] = f[k]/(descriptors->points.size());
    if(pcl_isnan(f[k])>0)
    {
        f[k] = 0.0;
    }
}

cout<<f[0]<<endl;
cout<<"Original cloud points.size "<<cloud->points.size()<<endl;

for(int l = 0;l<352;++l)
{
    shota_file<<f[l]<<",";
}

shota_file.close();
return 0;

}
*/

