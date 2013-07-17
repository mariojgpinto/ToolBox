#include "ToolBoxPCL.h"

#include <pcl/point_types.h>

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
	bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, int point_radius){
		if(!points) return false;

		return false;
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