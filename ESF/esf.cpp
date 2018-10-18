#include<iostream>
#include<vector>
#include<utility>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/features/esf.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<fstream>
#include<string.h>
#include<stdlib.h>
#include<dirent.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<sys/types.h>


using namespace std;

// the main can take in command line arguments
int main(int argc, char** argv)
{

    ofstream fi;
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

    fi.open("esf_descriptor_final_new.csv");
    fi<<"Function"<<","<<"S.No"<<","<<"Object Type"<<","<<"Names"<<","<<"Label"<<",";
    for(int a = 1; a<=640; a++)
    {
        fi<<"Feature"<<a<<",";
    }
    fi<<"\n";
    pcl::visualization::PCLPlotter plotter ;
    // create a pointer for pointcloud data called cloud and initialize it
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    int j = 1;
    string type_tool[6] = {"CONTAIN", "CUT", "FLIP", "HIT", "POKE", "SCOOP"};
    main_folder = "/home/nithins/Desktop/ThreeDCV/Dataset";

    for(int k=1;k<=6;k++)
    {

        if(k==1)
        {
            subfolder = "CONTAIN";
            folder = main_folder + "/" + subfolder;
        }
        else if(k==2)
        {
            subfolder = "CUT";
            folder = main_folder + "/" + subfolder;
        }
        else if(k==3)
        {
            subfolder = "FLIP";
            folder = main_folder + "/" + subfolder;
        }
        else if(k==4)
        {
            subfolder = "HIT";
            folder = main_folder + "/" + subfolder;
        }
        else if(k==5)
        {
            subfolder = "POKE";
            folder = main_folder + "/" + subfolder;
        }
        else if(k==6)
        {
            subfolder = "SCOOP";
            folder = main_folder + "/" + subfolder;
        }




    dp = opendir(folder.c_str());
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
            label_2 = label_2 * k;
        }
        else if(pos_2!=-1)
        {
            stringsub = filepath.substr(pos_2);
            label_handle = 2*k - 1;
        }
        else if(pos_3!=-1)
        {
            stringsub = filepath.substr(pos_3);
            label_2 = label_2 * k;
        }
        else if(pos_4!=-1)
        {
            stringsub = filepath.substr(pos_4);
            label_2 = label_2 * k;
        }
        else if(pos_5!=-1)
        {
            stringsub = filepath.substr(pos_5);
            label_2 = label_2 * k;
        }
        else if(pos_6!=-1)
        {
            stringsub = filepath.substr(pos_6);
            label_2 = label_2 * k;
        }
        else if(pos_7!=-1)
        {
            stringsub = filepath.substr(pos_7);
            label_2 = label_2 * k;
        }
        cout<<stringsub<<endl;
        fi<<subfolder<<","<<j<<","<<stringsub<<",";
        j++;
        fi<<dirp->d_name<<",";
        if(label_handle!=0)
        {
            fi<<int(label_handle)<<",";
        }
        else
        {
            fi<<int(label_2)<<",";
        }
        if (pcl::io::loadPLYFile<pcl::PointXYZ> (filepath, *cloud)==-1)
        {
            PCL_ERROR("couldn't read file\n");
            return(-1);
        }
        // Create the ESF estimation class and pass the input dataset + normals to it
        pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
        esf.setInputCloud (cloud);


        // output dataset
        pcl::PointCloud<pcl::ESFSignature640>::Ptr esfSignature (new pcl::PointCloud<pcl::ESFSignature640>);

        // compute features
        esf.compute(*esfSignature);
        for(int i=0;i<640;i++)
        {
            fi<<float(esfSignature->points[0].histogram[i])<<",";
        }
        fi<<"\n";

    }
    closedir(dp);
    }

/*if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/nithin/Desktop/ThreeDCV/ToolWeb/fyingpan_4_3dwh.ply", *cloud)==-1)
{
    PCL_ERROR("couldn't read file\n");
    return(-1);
}*/

// Create the ESF estimation class and pass the input dataset + normals to it
/*pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
esf.setInputCloud (cloud);


// output dataset
pcl::PointCloud<pcl::ESFSignature640>::Ptr esfSignature (new pcl::PointCloud<pcl::ESFSignature640>);

// compute features
esf.compute(*esfSignature);

//esf.setNormalizeBins(false);
cout<<"ESF output points.size "<<esfSignature->points.size()<<endl;
cout<<"Original cloud points.size "<<cloud->points.size()<<endl;
//esf descriptor for first point
cout<<esfSignature->points[0];
plotter.addFeatureHistogram(*esfSignature,640);
plotter.plot();

esf_file<<"Function"<<",";
esf_file<<"Part"<<",";
esf_file<<"Names"<<",";
esf_file<<"Features"<<"\n";
esf_file<<"CONTAIN"<<","<<"contain"<<",";
*/

    /*for(int i=0;i<640;i++)
{
    esf_file<<esfSignature->points[0].histogram[i]<<",";
}
//esf_file<<esfSignature->points[0];
*/
fi.close();
return 0;
}
