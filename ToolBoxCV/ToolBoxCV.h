#ifndef _TOOLBOX_CV
#define _TOOLBOX_CV

#include <opencv2\opencv.hpp>

namespace ToolBoxCV{
	class Timer{
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
			int asd;
	};

	bool in_range(cv::Point* point, int width = 640, int height = 480);
}

#endif