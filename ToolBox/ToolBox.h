/** 
 * @file	ToolBox.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	Somewhere, 2013
 * @brief	Declaration of ToolBox namespace.
 */
#ifndef _TOOLBOX
#define _TOOLBOX

#pragma warning(disable: 4251) //Disable dll interface warning for std::vector

#include <vector>

#ifdef TOOLBOX_DLL_EXPORT
	#define TOOLBOX_DLL __declspec(dllexport)
#else
	#define TOOLBOX_DLL __declspec(dllimport)
#endif

namespace ToolBox{
	class TOOLBOX_DLL Color{
		public:
			Color();
			Color(int rr, int gg, int bb){
				this->r = rr; this->g = gg; this->b = bb;
			}
			~Color();

			int r,g,b;
	};

	class TOOLBOX_DLL Plane{
		public:	
			Plane(double a = 0, double b = 0, double c = 0, double d = 0);
			Plane(double n_x, double n_y, double n_z, double p_x, double p_y, double p_z);
			~Plane();

			inline void set(double a, double b, double c, double d);
			void set_normalized(double a, double b, double c, double d);
			double distance_to_plane(double x, double y, double z);
			inline void get_normal(double *x, double *y, double *z);

		public:
			double _a, _b, _c, _d;
	};

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

	__int64 TOOLBOX_DLL currentTimeMillis();
}

#endif