/** 
 * @file	TCPServer.h 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Declaration of the TCPServer class.
 */
#ifndef _TOOLBOX_IO_TCP_SERVER
#define _TOOLBOX_IO_TCP_SERVER

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
	class TOOLBOX_DLL TCPSocketSession{
		public:
			TCPSocketSession(boost::asio::io_service* io_service);
			~TCPSocketSession();

			//Setup
			void set_id(int id);
			void set_output_message_deque(std::deque<std::pair<int,std::string*>>* deque);
			void stop_session();
		
			//Run
			void start();

			//Messages
			void add_message(std::string msg);

			//Access
			boost::asio::ip::tcp::socket* get_socket();
			bool is_connected();
			int  get_id();
				

		private:
			void run_thread();
			void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
			void handle_write(const boost::system::error_code& error);

		private:
			int								_id;
			bool							_connected;
			bool							_running;
			boost::asio::ip::tcp::socket*	_socket;

			//Read
			enum { 
				buff_size = 1048576 
			};
			char							_buffer[buff_size];

			//Write
			boost::thread*					_thread;
			boost::mutex					_mutex;				/**< Mutex for the contition _condition.*/
			boost::condition_variable		_condition;			/**< Condition variable to wait for new messages.*/

			boost::mutex								_mutex_write;			/**< Mutex for the contition _condition.*/
			std::deque<std::string*>					_messages_to_write;		/**< DeQueue of messages to be send.*/

			std::deque<std::pair<int,std::string*>>*	_messages_read;
	};


	class TOOLBOX_DLL TCPServer{
		public:
			TCPServer(int port = 9991);
			~TCPServer();
		
			void add_message(std::string msg, int id = -1);

			void set_port(int port);
			void set_ip(char* ip);

			void init_server();
			void stop_server();

			short get_port();
			std::string* get_ip();

			bool new_message();
			void consume_message(std::string& message);

		private:
			void run_tcp_thread();
			void run_maintenance_thread();

			void handle_accept(TCPSocketSession* new_session, const boost::system::error_code& error);

			void manage_messages();
			void manage_sessions();

			void add_message_to_queue(std::string msg, int id);

		private:
			std::string									_host;
			std::string									_port;
			short										_port_short;

			boost::asio::io_service*					_io_service;	/**< Connection's IO Service.*/

			boost::thread*								_io_thread;
			boost::asio::ip::tcp::acceptor*				_acceptor;
		
			std::deque<TCPSocketSession*>				_sessions;
		
			std::deque<std::pair<int,std::string*>>		_messages;		/**< DeQueue of messages to be send.*/

			//bool								_tcp_flag;
			//TCPSocketSession*					_tcp_gui_socket;
		
			bool										_running;
			boost::thread*								_maintenance_thread;
			boost::mutex								_mutex;			/**< Mutex for the contition _condition.*/
			boost::condition_variable					_condition;		/**< Condition variable to wait for new messages.*/

			std::deque<std::pair<int,std::string*>>		_messages_received;		/**< DeQueue of messages to be send.*/
	};
}
#endif//_TOOLBOX_IO_TCP_SERVER