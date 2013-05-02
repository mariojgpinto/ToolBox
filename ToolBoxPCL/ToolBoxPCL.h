#ifndef _TOOLBOX_PCL
#define _TOOLBOX_PCL

#include <opencv2\opencv.hpp>

namespace ToolBoxPLC{
	bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, int point_radius = 5);
}

#endif