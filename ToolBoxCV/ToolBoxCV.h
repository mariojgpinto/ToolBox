#ifndef _TOOLBOX_CV
#define _TOOLBOX_CV

#include <opencv2\opencv.hpp>

#ifdef TOOLBOX_DLL_EXPORT
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllexport)
	#endif
#else
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllimport)
	#endif
#endif

namespace ToolBoxCV{
	class TOOLBOX_DLL Timer{
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

	TOOLBOX_DLL bool in_range(cv::Point* point, int width = 640, int height = 480);
	TOOLBOX_DLL double fitt_image(cv::Mat& orig, cv::Mat& out, int out_width = 640, int out_height = 480, cv::Rect* roi_out = 0);

	TOOLBOX_DLL bool inside(cv::Point* point, std::vector<cv::Point*>* points);
}

#endif//_TOOLBOX_CV