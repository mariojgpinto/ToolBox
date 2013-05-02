#include "ToolBoxPCL.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace ToolBoxPCL{
	bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, int point_radius){
		//if(!points) return false;

		std::string* pcl_file = new std::string("file.pcd");

		//FILE* fp = NULL;
		//fopen_s(&fp, pcl_file->data(),"w+");

		//if(!fp) return false;

		//fprintf(fp,"# .PCD v0.7 - Point Cloud Data file format\n");
		//fprintf(fp,"VERSION 0.7\n");
		//fprintf(fp,"FIELDS x y z\n");
		//fprintf(fp,"SIZE 4 4 4\n");
		//fprintf(fp,"TYPE F F F\n");
		//fprintf(fp,"COUNT 1 1 1\n");
		//fprintf(fp,"WIDTH %d\n", points->size() * point_radius * point_radius);
		//fprintf(fp,"HEIGHT 1\n");
		//fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
		//fprintf(fp,"POINTS %d\n", points->size() * point_radius * point_radius);
		//fprintf(fp,"DATA ascii\n");

		////TODO Change writting to file to cloud input
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	

		//cv::Point pt1;
		//cv::Point3f pf1;
		//for(int i = -point_radius ; i < point_radius ; i++){
		//	for(int j = -5 ; j < 5 ; j++){
		//		for(int k = 0 ; k < (int)points->size() ; k++){
		//			;
		//			if(points->at(i)->x == 0 && points->at(i)->y == 0 && points->at(i)->z == 0) fprintf(fp,"nan nan nan\n");
		//			else{
		//				fprintf(fp,"%f %f %f\n", points->at(i)->x, points->at(i)->y, points->at(i)->z);
		//				//cloud->push_back(pcl::PointXYZ(pf1.x,pf1.y,pf1.z));
		//				//cloud->points.push_back(pcl::PointXYZ(pf1.x,pf1.y,pf1.z));
		//			}
		//		}
		//	}
		//}

	
		//		

		//fclose(fp);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcl_file->data(), *cloud) == -1) //* load the file
		{
			//PCL_ERROR ("Couldn't read file floor.pcd \n");
			return NULL;
		}

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.02);

		seg.setInputCloud (cloud->makeShared ());
		seg.segment (*inliers, *coefficients);

		coefficients->values.size();

		*a = coefficients->values[0];
		*b = coefficients->values[1];
		*c = coefficients->values[2];
		*d = coefficients->values[3];
	}
}