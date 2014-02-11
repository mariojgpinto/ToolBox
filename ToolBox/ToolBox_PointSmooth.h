/** 
 * @file	ToolBox.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	February, 2014
 * @brief	Declaration of ToolBox namespace.
 */
#ifndef _TOOLBOX_POINT_SMOOTH
#define _TOOLBOX_POINT_SMOOTH

#pragma warning(disable: 4251) //Disable dll interface warning for std::vector

#include <vector>

#ifdef TOOLBOX_DLL_EXPORT
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllexport)
	#endif
#else
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllimport)
	#endif
#endif

namespace ToolBox{

	class TOOLBOX_DLL Point{
		public:
			Point(){x = 0;y = 0;z = 0;};
			~Point(){};

			double x;
			double y;
			double z;
	};

	class TOOLBOX_DLL PointSmooth{
		public:
			enum METHOD{
				LAST_ENTRY,
				METHOD1,
				METHOD2
			};

		public:
			PointSmooth(PointSmooth::METHOD method = LAST_ENTRY, int buffer_size = 1);
			~PointSmooth();

			void set_method(PointSmooth::METHOD method, int buffer_size = 10);
			void set_buffer_size(int size);
			void set_pos_height(int pos, double height);

			void add_new_point(double xx, double yy = 0., double zz = 0.);

			void get_first_point(int *xx, int *yy = new int, int *zz = new int);
			void get_first_point(double *xx, double *yy = new double, double *zz = new double);
			void get_last_point(int *xx, int *yy = new int, int *zz = new int);
			void get_last_point(double *xx, double *yy = new double, double *zz = new double);
			void get_processed_position(int *xx, int *yy = new int, int *zz = new int);
			void get_processed_position(float *xx, float *yy = new float, float *zz = new float);
			void get_processed_position(double *xx, double *yy = new double, double *zz = new double);

		private:
			void init_variables();

			void update_indexes();

			void process_last_entry(double *xx, double *yy, double *zz);
			void process_method_1(double *xx, double *yy, double *zz);
			void process_method_2(double *xx, double *yy, double *zz);

		public:
			int						_buffer_size;
			std::vector<Point>		_buffer;
			std::vector<int>		_buffer_idx;
			std::vector<double>		_buffer_height;

			int						_last_position_idx;
			int						_first_position_idx;

			METHOD					_method;
	};
}

#endif//_TOOLBOX_POINT_SMOOTH