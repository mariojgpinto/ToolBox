/** 
 * @file	ToolBoxIO.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	February, 2014
 * @brief	Declaration of ToolBoxIO namespace.
 */
#ifndef _TOOLBOX_IO
#define _TOOLBOX_IO

#pragma warning(disable: 4251) //Disable dll interface warning for std::vector

#include "ToolBoxIO_TCPServer.h"

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <deque>

#ifdef TOOLBOX_DLL_EXPORT
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllexport)
	#endif
#else
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllimport)
	#endif
#endif

namespace ToolBoxIO{
	class TOOLBOX_DLL TCPClient{
		public:
			TCPClient(char* host = "localhost", char* port = "9991", bool threaded = true);
			~TCPClient();

			bool is_connected();

			void add_message(std::string msg);			
			void add_message(int size, void* data);

			void stop_client();

		private:
			bool connect();

			void run();

		private:
			std::string							_host;
			std::string							_port;

			boost::asio::io_service*			_io_service;	/**< Connection's IO Service.*/

			bool								_flag_connected;
			boost::asio::ip::tcp::socket*		_socket;
		
			bool								_threaded;
			bool								_running;
			boost::thread*						_thread;
			boost::mutex						_mutex;			/**< Mutex for the contition _condition.*/
			boost::condition_variable			_condition;		/**< Condition variable to wait for new messages.*/

			std::deque<std::string*>			_messages;		/**< DeQueue of messages to be send.*/
	};

}

#endif//_TOOLBOX_IO