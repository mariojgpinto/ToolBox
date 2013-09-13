#include "ToolBoxPCL.h"

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//-----------------------------------------------------------------------------
// TIMER
//-----------------------------------------------------------------------------
//namespace ToolBoxPCL{
//	//-------------------------------------------------------------------------
//	// CONTRUCTORS
//	//-------------------------------------------------------------------------
//	Timer::Timer(){
//
//	}
//
//	Timer::~Timer(){
//
//	}
//	
//	void Timer::start_timer(){
//
//	}
//}

//-----------------------------------------------------------------------------
// other functions
//-----------------------------------------------------------------------------
namespace ToolBoxPCL{
	//bool in_range(cv::Point* point, int width, int height){
	//	if(!point) return false;
	//	if(point->x < 0 || point->y < 0 || point->x >= width || point->y >= height) return false;
	//	return true;
	//}
	bool write_to_pcd_file(std::vector<cv::Point3f*>* points, char* file_name){
		if(!points || !points || points->size() < 3) return false;

		FILE* fp = NULL;
		fopen_s(&fp, file_name,"w+");

		if(!fp) return false;

		fprintf(fp,"# .PCD v0.7 - Point Cloud Data file format\n");
		fprintf(fp,"VERSION 0.7\n");
		fprintf(fp,"FIELDS x y z\n");
		fprintf(fp,"SIZE 4 4 4\n");
		fprintf(fp,"TYPE F F F\n");
		fprintf(fp,"COUNT 1 1 1\n");
		fprintf(fp,"WIDTH %d\n", points->size());
		fprintf(fp,"HEIGHT 1\n");
		fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
		fprintf(fp,"POINTS %d\n", points->size());
		fprintf(fp,"DATA ascii\n");

		for(int k = 0 ; k < (int)points->size() ; k++){
			fprintf(fp,"%f %f %f\n", points->at(k)->x, points->at(k)->y,points->at(k)->z);
		}

		fclose(fp);

		return true;
	}

	bool write_to_pcd_file(pcl::PointCloud<pcl::PointXYZ>& points, char* file_name){
		if(points.size() < 3) return false;

		FILE* fp = NULL;
		fopen_s(&fp, file_name,"w+");

		if(!fp) return false;

		fprintf(fp,"# .PCD v0.7 - Point Cloud Data file format\n");
		fprintf(fp,"VERSION 0.7\n");
		fprintf(fp,"FIELDS x y z\n");
		fprintf(fp,"SIZE 4 4 4\n");
		fprintf(fp,"TYPE F F F\n");
		fprintf(fp,"COUNT 1 1 1\n");
		fprintf(fp,"WIDTH %d\n", points.size());
		fprintf(fp,"HEIGHT 1\n");
		fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
		fprintf(fp,"POINTS %d\n", points.size());
		fprintf(fp,"DATA ascii\n");

		for(int k = 0 ; k < (int)points.size() ; k++){
			fprintf(fp,"%f %f %f\n", points.at(k).x, points.at(k).y,points.at(k).z);
		}

		fclose(fp);

		return true;
	}

	bool write_to_pcd_file(pcl::PointCloud<pcl::PointXYZRGB>& points, char* file_name){
		if(points.size() < 3) return false;

		FILE* fp = NULL;
		fopen_s(&fp, file_name,"w+");

		if(!fp) return false;

		fprintf(fp,"# .PCD v0.7 - Point Cloud Data file format\n");
		fprintf(fp,"VERSION 0.7\n");
		fprintf(fp,"FIELDS x y z r g b\n");
		fprintf(fp,"SIZE 4 4 4 1 1 1\n");
		fprintf(fp,"TYPE F F F I I I\n");
		fprintf(fp,"COUNT 1 1 1 1 1 1\n");
		fprintf(fp,"WIDTH %d\n", points.size());
		fprintf(fp,"HEIGHT 1\n");
		fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
		fprintf(fp,"POINTS %d\n", points.size());
		fprintf(fp,"DATA ascii\n");

		for(int k = 0 ; k < (int)points.size() ; k++){
			fprintf(fp,"%f %f %f %d %d %d\n",	points.at(k).x, points.at(k).y, points.at(k).z,
												points.at(k).r, points.at(k).g, points.at(k).b);
		}

		fclose(fp);

		return true;
	}

	bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, double dist_threshold){
		if(!points || points->size() < 3) return false;

		pcl::PointCloud<pcl::PointXYZ> cloud;
		//Fill in the cloud data
		cloud.width  = points->size();
		cloud.height = 1;
		cloud.points.resize (cloud.width * cloud.height);

		for(int i = 0 ; i < points->size() ; ++i){
			cloud[i] = pcl::PointXYZ(points->at(i)->x,points->at(i)->y,points->at(i)->z);
		}

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_PROSAC);
		seg.setDistanceThreshold (dist_threshold);

		seg.setInputCloud (cloud.makeShared ());
		seg.segment (*inliers, *coefficients);

		int s = coefficients->values.size();

		if(!s) return false;
	
		*a = coefficients->values[0];
		*b = coefficients->values[1];
		*c = coefficients->values[2];
		*d = coefficients->values[3];
	
		return true;
	}

	bool convert_points_to_mesh(int counter, XnPoint3D* points_in, pcl::PointCloud<pcl::PointXYZ>& cloud){
		if(counter <= 0) return true;

		if(!points_in) return false;

		cloud.clear();
		for(int i = 0 ; i < counter ; i++){
			if(points_in[i].Z > 0 )
				cloud.push_back(pcl::PointXYZ(points_in[i].X,points_in[i].Y,points_in[i].Z));
		}

		return true;
	}
}