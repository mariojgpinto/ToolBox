/** 
 * @file	ToolBox_StateMachine.h 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Declaration of the StateMachine class.
 */
#ifndef _TOOLBOX_STATE_MACHINE
#define _TOOLBOX_STATE_MACHINE

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

class TOOLBOX_DLL StateMachine{
	public:
		StateMachine(int entries = 10, int states = 4);
		~StateMachine();

		//Setup
		void set_states(int states);
		void update_threshold(std::vector<double> thresh);

		void set_entries(int entries);

		//Updates
		void update_state(int id, double value);

		//Access
		int get_state(int id);

	public:
		int						_n_entries;				/**< .*/
		int						_n_states;				/**< .*/
		std::vector<int>		_entries_states;		/**< .*/
		std::vector<double>		_entries_values;		/**< .*/
		std::vector<double>		_threshold_values;		/**< .*/
};

}

#endif//_TOOLBOX_STATE_MACHINE