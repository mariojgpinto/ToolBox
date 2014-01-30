#ifndef _TOOLBOX_PCL
#define _TOOLBOX_PCL

#pragma warning(disable: 4251) //Disable dll interface warning for std::vector

#ifdef TOOLBOX_DLL_EXPORT
	#define TOOLBOX_DLL __declspec(dllexport)
#else
	#define TOOLBOX_DLL __declspec(dllimport)
#endif

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
//#include <XnTypes.h>

//typedef struct XnPoint3D;

namespace ToolBoxPCL{
	TOOLBOX_DLL bool write_to_pcd_file(std::vector<cv::Point3f*>* points, char* file_name);
	TOOLBOX_DLL bool write_to_pcd_file(pcl::PointCloud<pcl::PointXYZ>& points, char* file_name);
	TOOLBOX_DLL bool write_to_pcd_file(pcl::PointCloud<pcl::PointXYZRGB>& points, char* file_name);

	TOOLBOX_DLL bool write_to_obj_file(std::vector<cv::Point3f*>* points, char* file_name);
	TOOLBOX_DLL bool write_to_obj_file(pcl::PointCloud<pcl::PointXYZ>& points, char* file_name);
	TOOLBOX_DLL bool write_to_obj_file(pcl::PointCloud<pcl::PointXYZRGB>& points, char* file_name);

	TOOLBOX_DLL bool write_to_ply_file(std::vector<cv::Point3f*>* points, char* file_name);
	TOOLBOX_DLL bool write_to_ply_file(pcl::PointCloud<pcl::PointXYZ>& points, char* file_name);
	TOOLBOX_DLL bool write_to_ply_file(pcl::PointCloud<pcl::PointXYZRGB>& points, char* file_name);

	TOOLBOX_DLL bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, double dist_threshold = 0.01);

	//TOOLBOX_DLL bool convert_points_to_mesh(int counter, XnPoint3D* points_in, pcl::PointCloud<pcl::PointXYZ>& cloud);

}

#endif