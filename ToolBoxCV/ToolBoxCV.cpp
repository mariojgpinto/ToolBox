#include "ToolBoxCV.h"

namespace ToolBoxCV{
	bool in_range(cv::Point* point, int width, int height){
		if(!point) return false;
		if(point->x < 0 || point->y < 0 || point->x >= width || point->y >= height) return false;
		return true;
	}
}