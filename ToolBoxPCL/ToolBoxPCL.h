#ifndef _TOOLBOX_PCL
#define _TOOLBOX_PCL

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <XnTypes.h>

//typedef struct XnPoint3D;

namespace ToolBoxPCL{
	__declspec(dllexport) bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, int point_radius = 5);

	__declspec(dllexport) bool convert_points_to_mesh(int counter, XnPoint3D* points_in, pcl::PointCloud<pcl::PointXYZ>& cloud);

}

#endif