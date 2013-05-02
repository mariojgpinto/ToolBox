#ifndef _TOOLBOX_CV
#define _TOOLBOX_CV

#include <opencv2\opencv.hpp>

namespace ToolBoxCV{
	bool in_range(cv::Point* point, int width = 640, int height = 480);
}

#endif