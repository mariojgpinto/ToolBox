/** 
 * @file	ToolBox.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	Somewhere, 2013
 * @brief	Declaration of ToolBox namespace.
 */
#ifndef _TOOLBOX
#define _TOOLBOX

#include "ToolBox_PointSmooth.h"
#include "ToolBox_StateMachine.h"

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

	__int64 TOOLBOX_DLL currentTimeMillis();
}

#endif