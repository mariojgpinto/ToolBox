#include "ToolBox.h"

#include <math.h>

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
}