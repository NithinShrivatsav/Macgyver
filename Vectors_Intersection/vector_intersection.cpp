#include<iostream>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/point_types.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_cloud.h>
#include <vector>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/common/centroid.h>
#include<pcl/common/transforms.h>
#include<pcl/common/pca.h>
#include <pcl/common/common.h>
#include<pcl/visualization/cloud_viewer.h>
#include<math.h>
#include<fstream>
#include<string.h>
#include<stdlib.h>
#include<dirent.h>
#include<stdio.h>
#include<unistd.h>
#include<sys/stat.h>
#include<sys/types.h>

using namespace std;

// Segment the point clouds into part and handle
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

// Transform the point clouds into eigen vector axis
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tf(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud)
{	
	// Compute the centroid (c0,c1,c2)
	// Compute the normalized covariance
	// compute the eigen vectors (e0,e1,e2) such that e2 = e0*e1 as this takes care of the proper sign of e2
	// move the points in that reference frame (note: the transformation given by rotation matrix (e0,e1,e0*e1) & (c0,c1,c2) must be inverted)
	pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*incloud,centroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*incloud,centroid,covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
	eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
	
	// Transform the point cloud to the new reference frame
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block<3,3>(0,0) = eigen_vectors.transpose();
	transform.block<3,1>(0,3) = -1.f * (transform.block<3,3>(0,0) * centroid.head<3>());
	pcl::transformPointCloud(*incloud,*transform_cloud,transform);
	
	/*Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*transform_cloud,minPoint,maxPoint);
	translate(1,3) = -1*minPoint.y;
	transform(2,3) = -1*minPoint.z;
	pcl::transformPointCloud(*transform_cloud,*transform_cloud_1,translate);*/
	
	return transform_cloud;
}

// calculate the norm of the vector
float create_norm(pcl::PointXYZ c, pcl::PointXYZ p)
{
	float norm = sqrt(pow((p.x-c.x),2) + pow((p.y-c.y),2) + pow((p.z-c.z),2));
	return norm;
}

// create the unit vector from centroid to intersection point
pcl::PointXYZ create_vector(pcl::PointXYZ c, pcl::PointXYZ p)
{
	pcl::PointXYZ unit_vector;
	float norm = create_norm(c,p);
	unit_vector.x = ((p.x-c.x)/norm);
	unit_vector.y = ((p.y-c.y)/norm);
	unit_vector.z = ((p.z-c.z)/norm);
	return unit_vector;
}


// check the number of significant eigen vectors
int rotation_number(pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud)
{
	Eigen::Vector4f centroid;
	int count = 0;
	pcl::compute3DCentroid(*main_cloud,centroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*main_cloud,centroid,covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
	//cout<<eigen_solver.eigenvalues()[0]<<"Eigen Values"<<endl;
	float lambda1 = eigen_solver.eigenvalues()[0]/(eigen_solver.eigenvalues()[0]+eigen_solver.eigenvalues()[1]+eigen_solver.eigenvalues()[2]);
	float lambda2 = eigen_solver.eigenvalues()[1]/(eigen_solver.eigenvalues()[0]+eigen_solver.eigenvalues()[1]+eigen_solver.eigenvalues()[2]);
	float lambda3 = eigen_solver.eigenvalues()[2]/(eigen_solver.eigenvalues()[0]+eigen_solver.eigenvalues()[1]+eigen_solver.eigenvalues()[2]);
	cout<<"lambda 1:\t"<<lambda1<<" lambda2:\t"<<lambda2<<" lambda3:\t"<<lambda3<<endl;
	//cout<<"Covariance:\n"<<covariance<<endl;
	if(lambda1>0.4)
	{
		count++;
	}
	if(lambda2>0.4)
	{
		count++;
	}
	if(lambda3>0.4)
	{
		count++;
	}
	return count;
}

// rotate unit vector by 180 degrees to create symmetric attachment point
pcl::PointXYZ rotate_x(pcl::PointXYZ UnitVector,int axes)
{
	// Rotation Matrix to transform the vector
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_1->push_back(UnitVector);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector3f unitvector;
	unitvector(0) = UnitVector.x;
	unitvector(1) = UnitVector.y;
	unitvector(2) = UnitVector.z;	
	pcl::PointXYZ transformed_unit_vector;
	Eigen::Matrix3f rotation_about_x = Eigen::Matrix3f::Identity();
	float theta;
	if(axes==1)
	{
		theta = M_PI;
	}
	else if(axes==2)
	{
		theta = M_PI/2;
	}
	else if(axes==3)
	{
		theta = 3*M_PI/2;
	}
	rotation_about_x(0,0) = 1;
	rotation_about_x(0,1) = 0;
	rotation_about_x(0,2) = 0;
	rotation_about_x(1,0) = 0;
	rotation_about_x(1,1) = cos(theta); 
	rotation_about_x(1,2) = -sin(theta);
	rotation_about_x(2,0) = 0;
	rotation_about_x(2,1) = sin(theta);
	rotation_about_x(2,2) = cos(theta); 
	Eigen::Vector3f transformedunitvector = rotation_about_x*unitvector;
	transformed_unit_vector.x = transformedunitvector(0);
	transformed_unit_vector.y = transformedunitvector(1);
	transformed_unit_vector.z = transformedunitvector(2);
	return transformed_unit_vector;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color_transform (new pcl::PointCloud<pcl::PointXYZRGBA>);
    	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_color (new pcl::PointCloud<pcl::PointXYZRGBA>);
    	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2_color (new pcl::PointCloud<pcl::PointXYZRGBA>);
    	pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_transform(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_transform(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_transform_points(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_transform_points(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	int order_store;
	ofstream feature_pairs;
	DIR *dp;
	struct dirent *dirp;
    	struct stat filestat;
    	string folder = "/home/nithins/Desktop/ThreeDCV/Tools_Dataset/SCOOP";
	string filepath;
	feature_pairs.open("Feature_Pairs_SCOOP_New.csv");
	feature_pairs<<"Object Name"<<","<<"Label"<<",";
	for(int k=1;k<=6;k++)
	{
		feature_pairs<<"Feature"<<" "<<k<<",";
	}
	feature_pairs<<"\n";
	
	dp = opendir(folder.c_str());
	if(dp == NULL)
        {
            return -1;
        }
	while(dirp = readdir(dp))
	{
		filepath = folder + "/" + dirp->d_name;
		if (stat( filepath.c_str(), &filestat )) continue;
		if (S_ISDIR( filestat.st_mode ))         continue;
		int label = 1;
		if (pcl::io::loadPLYFile(filepath, *cloud_color) == -1)
		{
			PCL_ERROR("Couldn't load file \n");
			return 0;
	    	}
		cout<<"Name of the object:\t"<<dirp->d_name<<endl;

		// segment into two parts
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
		intersection_cloud->push_back(point_set[i]);
		}

		pcl::PointXYZ intersection;
		pcl::PointXYZ intersection_cloud1;
		pcl::PointXYZ intersection_cloud2;
		kd_centroid.get(intersection);
		cout << intersection.x << ", " << intersection.y << ", " << intersection.z << endl;


		// Push back the intersection point to cloud
		cloud1->push_back(intersection);
		size_t cloud1_length = cloud1->points.size()-1;
		cout<<"The actual intersection point:\t"<<intersection.x<<","<<intersection.y<<","<<intersection.z<<endl;
		cout<<"The intersection point at the end of cloud1:\t"<<cloud1->points[cloud1_length].x<<","<<cloud1->points[cloud1_length].y<<","<<cloud1->points[cloud1_length].z<<endl;
		

		// Transform cloud 1 and find it's centroid		
		cloud1_transform = pcl_tf(cloud1);
		pcl::CentroidPoint<pcl::PointXYZ> centroid_1;
		pcl::PointXYZ c_1;
		size_t k=0;
		for(;k<cloud1_transform->points.size();k++)
		    {
		        centroid_1.add(cloud1_transform->points[k]);
		    }
		centroid_1.get(c_1);
		intersection_cloud1.x = cloud1_transform->points[k-1].x;
		intersection_cloud1.y = cloud1_transform->points[k-1].y;
		intersection_cloud1.z = cloud1_transform->points[k-1].z;
		cloud1_transform_points->push_back(intersection_cloud1);
		cloud1_transform_points->push_back(c_1);
		cout<<"Centroid of Cloud 1:\t"<<c_1.x<<" "<<c_1.y<<" "<<c_1.z<<endl;
		cout<<"Intersection point of Cloud 1:\t"<<intersection_cloud1.x<<" "<<intersection_cloud1.y<<" "<<intersection_cloud1.z<<endl;

		// Push back the intersection point to cloud2, Transform cloud2 and Compute it's centroid
		cloud2->push_back(intersection);
		size_t cloud2_length = cloud2->points.size()-1;
		cout<<"The actual intersection point:\t"<<intersection.x<<","<<intersection.y<<","<<intersection.z<<endl;
		cout<<"The intersection point at the end of cloud2:\t"<<cloud2->points[cloud2_length].x<<","<<cloud2->points[cloud2_length].y<<","<<cloud2->points[cloud2_length].z<<endl;
		cloud2_transform = pcl_tf(cloud2);
		pcl::CentroidPoint<pcl::PointXYZ> centroid_2;
		pcl::PointXYZ c_2;
		size_t l=0;
		for(;l<cloud2_transform->points.size();l++)
		    {
		        centroid_2.add(cloud2_transform->points[l]);
		    }
		centroid_2.get(c_2);
		intersection_cloud2.x = cloud2_transform->points[l-1].x;
		intersection_cloud2.y = cloud2_transform->points[l-1].y;
		intersection_cloud2.z = cloud2_transform->points[l-1].z;
		cloud2_transform_points->push_back(intersection_cloud2);
		cloud2_transform_points->push_back(c_2);
		cout<<"Centroid of Cloud 2:\t"<<c_2.x<<" "<<c_2.y<<" "<<c_2.z<<endl;
		cout<<"Intersection of Cloud2:\t"<<intersection_cloud2.x<<" "<<intersection_cloud2.y<<" "<<intersection_cloud2.z<<endl;
	
		// Get the number of principle axes
		int count_1 = rotation_number(cloud1_transform);
		cout<<"Number of significant directions of Cloud 1:\t"<<count_1<<endl;
		int count_2 = rotation_number(cloud2_transform);
		cout<<"Number of significant directions of Cloud 2:\t"<<count_2<<endl;
		
		// Declaration of all the unit vectors
		pcl::PointXYZ vec_1;
		pcl::PointXYZ vec_2;
		pcl::PointXYZ vec_3;
		pcl::PointXYZ vec_4;
		pcl::PointXYZ vec_5;
		pcl::PointXYZ vec_6;
		pcl::PointXYZ vec_7;
		pcl::PointXYZ vec_8;

		if(count_1==1)
		{
			vec_1 = create_vector(c_1,intersection_cloud1);
			vec_2 = rotate_x(vec_1,1);
			float norm_vector_1 = create_norm(c_1,intersection_cloud1);
			pcl::PointXYZ point_1;
			point_1.x = (norm_vector_1*vec_2.x)+c_1.x;	
			point_1.y = (norm_vector_1*vec_2.y)+c_1.y;
			point_1.z = (norm_vector_1*vec_2.z)+c_1.z;
			cloud1_transform_points->push_back(point_1);
			cout<<"Vector 1 "<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<endl;
			cout<<"Vector 2 "<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<endl;
		}
		else if(count_1==2)
		{
			
			vec_1 = create_vector(c_1,intersection_cloud1);
			
			vec_2 = rotate_x(vec_1,1);
			vec_3 = rotate_x(vec_1,2);
			vec_4 = rotate_x(vec_1,3);
			
			float norm_vector_1 = create_norm(c_1,intersection_cloud1);
			pcl::PointXYZ point_1;
			point_1.x = (norm_vector_1*vec_2. x)+c_1.x;	
			point_1.y = (norm_vector_1*vec_2.y)+c_1.y;
			point_1.z = (norm_vector_1*vec_2.z)+c_1.z;
			cloud1_transform_points->push_back(point_1);
			
			pcl::PointXYZ point_2;
			point_2.x = c_1.x+(norm_vector_1*vec_3.x);	
			point_2.y = c_1.y+(norm_vector_1*vec_3.y);
			point_2.z = c_1.z+(norm_vector_1*vec_3.z);
			cloud1_transform_points->push_back(point_2);
			
			pcl::PointXYZ point_3;
			point_3.x = c_1.x+(norm_vector_1*vec_4.x);	
			point_3.y = c_1.y+(norm_vector_1*vec_4.y);
			point_3.z = c_1.z+(norm_vector_1*vec_4.z);
			cloud1_transform_points->push_back(point_3);
			cout<<"Vector 1 "<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<endl;
			cout<<"Vector 2 "<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<endl;
			cout<<"Vector 3 "<<vec_3.x<<","<<vec_3.y<<","<<vec_3.z<<endl;
			cout<<"Vector 4 "<<vec_4.x<<","<<vec_4.y<<","<<vec_4.z<<endl;
		}

		if(count_2==1)
		{
			
			vec_5 = create_vector(c_2,intersection_cloud2);
			
			vec_6 = rotate_x(vec_5,1);
			float norm_vector_2 = create_norm(c_2,intersection_cloud2);
			pcl::PointXYZ point_4;
			point_4.x = c_2.x+(norm_vector_2*vec_6.x);	
			point_4.y = c_2.y+(norm_vector_2*vec_6.y);
			point_4.z = c_2.z+(norm_vector_2*vec_6.z);
			cloud2_transform_points->push_back(point_4);
			cout<<"Vector 5 "<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<endl;
			cout<<"Vector 6 "<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<endl;
		}
		else if(count_2==2)
		{
			
			vec_5 = create_vector(c_2,intersection_cloud2);

			vec_6 = rotate_x(vec_5,1);
			vec_7 = rotate_x(vec_5,2);
			vec_8 = rotate_x(vec_5,3);
			float norm_vector_2 = create_norm(c_2,intersection_cloud2);

			pcl::PointXYZ point_4;
			point_4.x = c_2.x+(norm_vector_2*vec_6.x);	
			point_4.y = c_2.y+(norm_vector_2*vec_6.y);
			point_4.z = c_2.z+(norm_vector_2*vec_6.z);
			cloud2_transform_points->push_back(point_4);
			
			pcl::PointXYZ point_5;
			point_5.x = c_2.x+(norm_vector_2*vec_7.x);	
			point_5.y = c_2.y+(norm_vector_2*vec_7.y);
			point_5.z = c_2.z+(norm_vector_2*vec_7.z);
			cloud2_transform_points->push_back(point_5);
			
			pcl::PointXYZ point_6;
			point_6.x = c_2.x+(norm_vector_2*vec_8.x);	
			point_6.y = c_2.y+(norm_vector_2*vec_8.y);
			point_6.z = c_2.z+(norm_vector_2*vec_8.z);
			cloud2_transform_points->push_back(point_6);
			
			cout<<"Vector 5 "<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<endl;
			cout<<"Vector 6 "<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<endl;
			cout<<"Vector 7 "<<vec_7.x<<","<<vec_7.y<<","<<vec_7.z<<endl;
			cout<<"Vector 8 "<<vec_8.x<<","<<vec_8.y<<","<<vec_8.z<<endl;
		}




		/*pcl::PointXYZ vec_1;
		pcl::PointXYZ vec_3;
		pcl::PointXYZ vec_4;
		pcl::PointXYZ point_1;
		pcl::PointXYZ point_2;
		vec_1 = create_vector(c_1,intersection_cloud1);
		vec_4 = rotate_x_90(vec_1);	
		vec_3 = rotate_x_180(vec_1);
		float norm1 = create_norm(c_1,intersection_cloud1);
		point_1.x = c_1.x-(norm1*vec_3.x);	
		point_1.y = c_1.y-(norm1*vec_3.y);
		point_1.z = c_1.z-(norm1*vec_3.z);
		
		point_2.x = c_1.x-(norm1*vec_4.x);
		point_2.y = c_1.y-(norm1*vec_4.y);
		point_2.z = c_1.z-(norm1*vec_4.z);

		cloud1_transform_points->push_back(point_1);
		cloud1_transform_points->push_back(point_2);
		cout<<"vec_1:\t"<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<","<<endl;
		cout<<"vec_3:\t"<<vec_3.x<<","<<vec_3.y<<","<<vec_3.z<<","<<endl;
		cout<<"vec_4:\t"<<vec_4.x<<","<<vec_4.y<<","<<vec_4.z<<","<<endl;

		pcl::PointXYZ vec_2;
		vec_2 = create_vector(c_2,intersection_cloud2);
		cout<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<","<<endl;*/

		pcl::visualization::PCLVisualizer viewer("Transform");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler_1 (cloud1_transform, 255, 255, 255); // White
		viewer.addPointCloud (cloud1_transform, source_cloud_color_handler_1, "Transformed Cloud 1");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler_2 (cloud2_transform, 230, 20, 20); // Red
	  	viewer.addPointCloud (cloud2_transform, source_cloud_color_handler_2, "Transformed Cloud 2");

		/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_points_handler_1 (cloud1_transform_points, 20, 230, 20); // Green
	  	viewer.addPointCloud (cloud1_transform_points, source_cloud_color_points_handler_1, "Transformed Cloud Points 1");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_points_handler_2 (cloud2_transform_points, 20, 20, 230); // Blue
	  	viewer.addPointCloud (cloud2_transform_points, source_cloud_color_points_handler_2, "Transformed Cloud Points 2");*/

		//viewer.addCoordinateSystem (1.0, "cloud", 0);
	  	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Transformed Cloud 1");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Transformed Cloud 2");
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Transformed Cloud Points 1");
		//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Transformed Cloud Points 2");
	  	while (!viewer.wasStopped ()) 
		{ 
	    		viewer.spinOnce ();
	  	}	
		cout<<"Enter the order of storing: "<<endl;
		cin>>order_store;
		if(order_store==1)
		{
			if(count_1==1 && count_2==1)
			{
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<"\n";
					
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";				
				feature_pairs<<label<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<"\n";

				cout<<"Vector 1 "<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<endl;
				cout<<"Vector 2 "<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<endl;
				cout<<"Vector 5 "<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<endl;
				cout<<"Vector 6 "<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<endl;
			}
			
			else if(count_1==2 && count_2==1)
			{
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";				
				feature_pairs<<label<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_3.x<<","<<vec_3.y<<","<<vec_3.z<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_3.x<<","<<vec_3.y<<","<<vec_3.z<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";				
				feature_pairs<<label<<","<<vec_4.x<<","<<vec_4.y<<","<<vec_4.z<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_4.x<<","<<vec_4.y<<","<<vec_4.z<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<"\n";

				
			}
			
			else if(count_1==1 && count_2==2)
			{
				cout<<"peace"<<endl;
			}
			
			else if(count_1==2 && count_2==2)
			{	
				cout<<"peace"<<endl;	
			}
		}
		else if(order_store==2)	
		{
			if(count_1==1 && count_2==1)
			{
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";				
				feature_pairs<<label<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<"\n";
			}
			
			else if(count_1==1 && count_2==2)
			{
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_5.x<<","<<vec_5.y<<","<<vec_5.z<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";				
				feature_pairs<<label<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_6.x<<","<<vec_6.y<<","<<vec_6.z<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_7.x<<","<<vec_7.y<<","<<vec_7.z<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_7.x<<","<<vec_7.y<<","<<vec_7.z<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<"\n";
				
				feature_pairs<<dirp->d_name<<",";				
				feature_pairs<<label<<","<<vec_8.x<<","<<vec_8.y<<","<<vec_8.z<<","<<vec_1.x<<","<<vec_1.y<<","<<vec_1.z<<"\n";

				feature_pairs<<dirp->d_name<<",";
				feature_pairs<<label<<","<<vec_8.x<<","<<vec_8.y<<","<<vec_8.z<<","<<vec_2.x<<","<<vec_2.y<<","<<vec_2.z<<"\n";
			}

			else if(count_1==2 && count_2==1)
			{
				cout<<"peace"<<endl;
			}
			
			
			else if(count_1==2 && count_2==2)
			{	
				cout<<"peace"<<endl;	
			}
		}
		else if(order_store==3)
		{
			continue;
		}
			
			
	}
	
	
        	
	return 0;
}

// For all bottles only bottom part is needed
// For all bowls there is just one color and hence on cloud - modify the code to directly ave those files after transformation and calculatin vectors
