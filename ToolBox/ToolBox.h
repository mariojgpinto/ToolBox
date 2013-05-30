#ifndef _TOOLBOX
#define _TOOLBOX

namespace ToolBox{
	class __declspec(dllexport) Plane{
		public:	
			Plane(double a = 0, double b = 0, double c = 0, double d = 0);
			Plane(double n_x, double n_y, double n_z, double p_x, double p_y, double p_z);
			~Plane();

			void set(double a, double b, double c, double d);
			double distance_to_plane(double x, double y, double z);
			inline void get_normal(double *x, double *y, double *z);

		public:
			double _a, _b, _c, _d;
	};
}

#endif