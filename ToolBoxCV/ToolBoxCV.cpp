#include "ToolBoxCV.h"

//-----------------------------------------------------------------------------
// TIMER
//-----------------------------------------------------------------------------
namespace ToolBoxCV{
	//-------------------------------------------------------------------------
	// CONTRUCTORS
	//-------------------------------------------------------------------------
	Timer::Timer(){

	}

	Timer::~Timer(){

	}
	
	void Timer::start_timer(){

	}
}

//-----------------------------------------------------------------------------
// other functions
//-----------------------------------------------------------------------------
namespace ToolBoxCV{
	/**
	 *
	 */
	bool in_range(cv::Point* point, int width, int height){
		if(!point) return false;
		if(point->x < 0 || point->y < 0 || point->x >= width || point->y >= height) return false;
		return true;
	}

	/** 
	 * @brief	
	 * @details	
	 *
	 * @param[in]	orig
	 *				.
	 * @param[out]	out
	 *				.
	 * @param[in]	out_width
	 *				.
	 * @param[in]	out_height
	 *				.
	 * @param[out]	roi_out
	 *				.
	 *
	 * @return	
	 *			
	 */
	double fitt_image(cv::Mat& orig, cv::Mat& out, int out_width, int out_height, cv::Rect* roi_out){
		//Test Input
		if(orig.cols <= 0 || orig.rows <= 0) return -1;

		//Create Output
		out = cv::Mat::zeros(cv::Size(out_width,out_height),orig.type());

		//Calc ROI and Resize Image
		if(!roi_out) roi_out = new cv::Rect();
		cv::Mat temp;
		int w,h;
		double output_ratio = (double)out.cols / (double)out.rows;
		double input_ratio = (double)orig.cols / (double)orig.rows;

		double ratio = 1.0;

		if(input_ratio < output_ratio){
			ratio = (float)out.rows/(float)orig.rows;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = (out.cols - w) / 2.0;
			roi_out->y = 0;
			roi_out->width = w;
			roi_out->height = h;
		}
		else{
			ratio = (float)out.cols/(float)orig.cols;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = 0;
			roi_out->y = (out.rows - h) / 2.0;
			roi_out->width = w;
			roi_out->height = h;
		}

		//Copy to Destination
		temp.copyTo(out(*roi_out));

		return ratio;
	}

	/**
	 * @brief	
	 * @details	
	 *
	 * @param[in]	point
	 *				
	 * @param[in]	points
	 *
	 *
	 * @return
	 * 
	 */
	bool inside(cv::Point* point, std::vector<cv::Point*>* points){
		if(!point || !points || !points->size()) return false;

		int max_x = -INT_MAX;
		int max_y = -INT_MAX;
		int min_x = INT_MAX;
		int min_y = INT_MAX;

		for(unsigned int i = 0 ; i < points->size() ; ++i){
			if(points->at(i)->x > max_x)
				max_x = points->at(i)->x;
			if(points->at(i)->x < min_x)
				min_x = points->at(i)->x;
			if(points->at(i)->y > max_y)
				max_y = points->at(i)->y;
			if(points->at(i)->y < min_y)
				min_y = points->at(i)->y;
		}

		if(point->x > max_x || point->x < min_x || point->y > max_y || point->y < min_y) return false;
		return true;
	}
}