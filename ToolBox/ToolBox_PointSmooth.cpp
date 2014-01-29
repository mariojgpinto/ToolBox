/** 
 * @file	ToolBox_PointSmooth.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	January, 2014
 * @brief	Implementation of the PointSmooth class.
 */
#include "ToolBox.h"

namespace ToolBox {

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
PointSmooth::PointSmooth(PointSmooth::METHOD method, int buffer_size){
	this->_last_position_idx = 0;
	this->_first_position_idx = 0;

	this->set_method(method);

	this->set_buffer_size(buffer_size);
}

/**
 * @brief	.
 * @details	.
 */
PointSmooth::~PointSmooth(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void PointSmooth::set_method(PointSmooth::METHOD method, int buffer_size){
	if(this->_method == method)
		return;

	this->_method = method;

	switch(this->_method){
		case	PointSmooth::LAST_ENTRY:

			break;
		case	PointSmooth::METHOD1:
			this->set_buffer_size(buffer_size);
			break;
		default: break;
	}
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::set_buffer_size(int size){
	if(size < 1) 
		return;

	this->_buffer_size = size;

	//this->_buffer.resize(this->_buffer_size);
	this->_buffer_idx.resize(this->_buffer_size);
	this->_buffer_height.resize(this->_buffer_size);

	//TODO Re-addapt indexes
	//TODO Check resize to lower value
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::set_pos_height(int pos, double height){
	if(pos < this->_buffer_size){
		this->_buffer_height[pos] = height;
	}
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void PointSmooth::add_new_point(double xx, double yy, double zz){
	this->update_indexes();
	
	this->_buffer[this->_buffer_idx[this->_last_position_idx]].x = xx;
	this->_buffer[this->_buffer_idx[this->_last_position_idx]].y = yy;
	this->_buffer[this->_buffer_idx[this->_last_position_idx]].z = zz;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::update_indexes(){
	if((int)this->_buffer.size() < this->_buffer_size){
		this->_buffer.push_back(Point());
		this->_buffer_idx[this->_last_position_idx] = this->_last_position_idx;
		this->_last_position_idx = (this->_last_position_idx + 1) % this->_buffer_size;;
	}
	else{	
		for(int i = 0 ; i < this->_buffer_size ; ++i){
			this->_buffer_idx[i] = (this->_first_position_idx + i) % this->_buffer_size;
		}

		this->_last_position_idx  = 0;//(this->_last_position_idx  + 1) % this->_buffer_size;
		this->_first_position_idx = (this->_first_position_idx + 1) % this->_buffer_size;
	}
}


//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void PointSmooth::process_last_entry(double *xx, double *yy, double *zz){
	*xx = this->_buffer[this->_buffer_idx[this->_last_position_idx]].x;
	*yy = this->_buffer[this->_buffer_idx[this->_last_position_idx]].y;
	*zz = this->_buffer[this->_buffer_idx[this->_last_position_idx]].z;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::process_method_1(double *xx, double *yy, double *zz){
	Point point;
	int size = this->_buffer.size();
	for(int i = 0 ; i < size ; ++i){
		point.x += this->_buffer[this->_buffer_idx[i]].x;
		point.y += this->_buffer[this->_buffer_idx[i]].y;
		point.z += this->_buffer[this->_buffer_idx[i]].z;
	}

	*xx = point.x / size;
	*yy = point.y / size;
	*zz = point.z / size;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::process_method_2(double *xx, double *yy, double *zz){

}


//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::get_last_point(double *xx, double *yy, double *zz){
	this->process_last_entry(xx,yy,zz);
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::get_last_point(int *xx, int *yy, int *zz){
	double _xx,_yy,_zz;

	this->get_last_point(&_xx, &_yy, &_zz);

	*xx = (int)_xx;
	*yy = (int)_yy;
	*zz = (int)_zz;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::get_first_point(double *xx, double *yy, double *zz){
	*xx = this->_buffer[this->_buffer_idx[this->_first_position_idx]].x;
	*yy = this->_buffer[this->_buffer_idx[this->_first_position_idx]].y;
	*zz = this->_buffer[this->_buffer_idx[this->_first_position_idx]].z;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::get_first_point(int *xx, int *yy, int *zz){
	double _xx,_yy,_zz;

	this->get_first_point(&_xx, &_yy, &_zz);

	*xx = (int)_xx;
	*yy = (int)_yy;
	*zz = (int)_zz;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::get_processed_position(int *xx, int *yy, int *zz){
	double _xx,_yy,_zz;

	this->get_processed_position(&_xx, &_yy, &_zz);

	*xx = (int)_xx;
	*yy = (int)_yy;
	*zz = (int)_zz;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::get_processed_position(float *xx, float *yy, float *zz){
	double _xx,_yy,_zz;

	this->get_processed_position(&_xx, &_yy, &_zz);

	*xx = (float)_xx;
	*yy = (float)_yy;
	*zz = (float)_zz;
}

/**
 * @brief	.
 * @details	.
 */
void PointSmooth::get_processed_position(double *xx, double *yy, double *zz){
	if(!this->_buffer.size()){
		*xx = 0.;
		*yy = 0.;
		*zz = 0.;
		return;
	}

	switch(this->_method){
		case LAST_ENTRY:
			this->process_last_entry(xx,yy,zz);
			break;
		case METHOD1:
			this->process_method_1(xx,yy,zz);
			break;
		default: 
			this->process_last_entry(xx,yy,zz); 
			break;
	}
}

} //namespace ToolBox 