/** 
 * @file	StateMachine.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Implementation of the StateMachine class.
 */
#include "ToolBox_StateMachine.h"

namespace ToolBox{

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	StateMachine Constructor.
 * @details	StateMachine(int entries). 
 *	
 */
StateMachine::StateMachine(int entries, int states){
	this->set_entries(entries);
	this->set_states(states);
}

/**
 * @brief	StateMachine Destructor.
 * @details	~StateMachine().
 */
StateMachine::~StateMachine(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	set_n_states(int states).
 * @details	set_n_states(int states).
 */
void StateMachine::set_states(int states){
	this->_threshold_values.resize(states);

	std::vector<double> thresh(states);
	double step = 1.0/((double)states);

	for(int i = 0 ; i < states ; ++i){
		thresh[i] = (i+1)*step;
	}

	this->update_threshold(thresh);

	this->_n_states = states;
}

/**
 * @brief	update_threshold(std::vector<double> thresh).
 * @details	update_threshold(std::vector<double> thresh).
 */
void StateMachine::update_threshold(std::vector<double> thresh){
	this->_threshold_values.clear();
	this->_n_states = thresh.size()+1;
	for(unsigned int i = 0 ; i < thresh.size() ; ++i){
		this->_threshold_values.push_back(thresh[i]);
	}
}

/**
 * @brief	set_entries(int entries).
 * @details	set_entries(int entries).
 */
void StateMachine::set_entries(int entries){
	this->_entries_states.resize(entries);
	this->_entries_values.resize(entries);
	
	this->_n_entries = entries;
}


//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
/**
 * @brief	update_state(int id, double value).
 * @details	update_state(int id, double value).
 */
void StateMachine::update_state(int id, double value){
	if(id < 0 || id >= this->_n_entries) 
		return ;

	this->_entries_values[id] = value;
	int i = 0;

	for( ; i < this->_n_states -1 ; ++i){
		if(value < this->_threshold_values[i]){
			break;
		}
	}

	this->_entries_states[id] = i;
}


//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	get_state(int id).
 * @details	get_state(int id).
 */
int StateMachine::get_state(int id){
	if(id < 0 || id >= this->_n_entries) 
		return -1;

	return this->_entries_states[id];
}

}//namespace ToolBox