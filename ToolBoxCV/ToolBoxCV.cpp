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
	 * @param[out]	orig
	 *				.
	 * @param[in]	orig
	 *				.
	 * @param[in]	orig
	 *				.
	 * @param[out]	orig
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
}