#include "ToolBox.h"

#include <math.h>
#include <windows.h>
#include <cstring>

//-----------------------------------------------------------------------------
// OTHER FUNCTION
//-----------------------------------------------------------------------------
namespace ToolBox {
	//-------------------------------------------------------------------------
	// PLANE
	//-------------------------------------------------------------------------
	Plane::Plane(double a, double b, double c, double d){
		this->_a = a; this->_b = b; this->_c = c; this->_d = d;
	}

	Plane::Plane(double n_x, double n_y, double n_z, double p_x, double p_y, double p_z){
		double lenght = sqrt(n_x*n_x + n_y*n_y + n_z*n_z);
		_a = n_x / lenght;
		_b = n_y / lenght;
		_c = n_z / lenght;
		_d = -(_a*p_x + _b*p_y + _c*p_z);
	}

	Plane::~Plane(){

	}
	
	void Plane::set_normalized(double a, double b, double c, double d){
		double v = sqrt(a * a + b * b + c * c);
		this->_a = a/v;
		this->_b = b/v;
		this->_c = c/v;
		this->_d = d;
	}

	void Plane::set(double a, double b, double c, double d){
		this->_a = a;
		this->_b = b;
		this->_c = c;
		this->_d = d;
	}

	double Plane::distance_to_plane(double x, double y, double z){
		double v = _a*x + _b*y + _c*z + _d;
		v /= sqrt(_a*_a + _b*_b + _c*_c);
		return abs(v);
	}

	void Plane::get_normal(double *x, double *y, double *z){
		*x = _a; *y = _b; *z = _c;
	}





	
	__int64 currentTimeMillis()
	{
		static const __int64 magic = 116444736000000000; // 1970/1/1
		SYSTEMTIME st;
		 GetSystemTime(&st);
		 FILETIME   ft;
		SystemTimeToFileTime(&st,&ft); // in 100-nanosecs...
		 __int64 t;
		  memcpy(&t,&ft,sizeof t);
	  return (t - magic)/10000; // scale to millis.
	}
}