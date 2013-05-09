#ifndef _TOOLBOX
#define _TOOLBOX

#include <string>

namespace ToolBox{
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

	void coiso();
}

#endif