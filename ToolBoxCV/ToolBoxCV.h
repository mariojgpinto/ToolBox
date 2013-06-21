#ifndef _TOOLBOX_CV
#define _TOOLBOX_CV

#include <opencv2\opencv.hpp>

namespace ToolBoxCV{
	class __declspec(dllexport) Timer{
		public:
			Timer();
			~Timer();

			void start_timer();

			double pause_timer();
			double stop_timer();
			double restart_timer();

			double time_elapsed_seconds();
			double time_elapsed_milliseconds();

			std::string* time_elapsed_seconds_str();
			std::string* time_elapsed_milliseconds_str();

		private:
			double _first_tick;
	};

	__declspec(dllexport) bool in_range(cv::Point* point, int width = 640, int height = 480);
	__declspec(dllexport) double fitt_image(cv::Mat& orig, cv::Mat& out, int out_width = 640, int out_height = 480, cv::Rect* roi_out = 0);
}

#endif//_TOOLBOX_CV