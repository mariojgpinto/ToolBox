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
	class TOOLBOX_DLL TCPMessageSession{
		public:
			//Read
			enum { 
				buff_size = 1280*960*3 
			};


		public:
			TCPMessageSession(boost::asio::io_service* io_service);
			~TCPMessageSession();

			//Setup
			void set_id(int id);
			void set_output_message_deque(std::deque<std::pair<int,std::string*>>* deque);
			void set_output_image_buffer(void **data, int *size);
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

			
			char							_buffer[buff_size];

			//Write
			boost::thread*					_thread;
			boost::mutex					_mutex;				/**< Mutex for the contition _condition.*/
			boost::condition_variable		_condition;			/**< Condition variable to wait for new messages.*/

			boost::mutex								_mutex_write;			/**< Mutex for the contition _condition.*/
			std::deque<std::string*>					_messages_to_write;		/**< DeQueue of messages to be send.*/

			std::deque<std::pair<int,std::string*>>*	_messages_read;
			void*										_image_buffer;
			void*										_image_buffer_temp;
			int*										_image_size;

			int _temp_idx;
	};

	class TOOLBOX_DLL TCPImageSession{
		public:
			enum STATUS{
				CONNECTED,
				RUNNING,
				OUT_IMAGE
			};
			static const int _status_size = 3;

		public:
			TCPImageSession(boost::asio::io_service* io_service, int image_size = 640*480*3);
			~TCPImageSession();

			//Setup
			void set_id(int id);
			void set_output_image_buffer(void **data, int *size);
			void stop_session();
		
			//Run
			void start();

			//Messages
			void add_message(std::string msg);

			//Access
			boost::asio::ip::tcp::socket* get_socket();
			bool is_connected();
			bool is_running();
			int  get_id();
				

		private:
			void run_thread();
			void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
			void handle_write(const boost::system::error_code& error);

			void save_new_image();

		private:
			bool							_status[_status_size];
			
			int								_id;
			boost::asio::ip::tcp::socket*	_socket;

			
			unsigned int					_buffer_size;
			char*							_buffer;

			//Write
			boost::thread*					_thread;
			boost::mutex					_mutex;				/**< Mutex for the contition _condition.*/
			boost::condition_variable		_condition;			/**< Condition variable to wait for new messages.*/

			boost::mutex					_mutex_write;			/**< Mutex for the contition _condition.*/
			std::deque<std::string*>		_messages_to_write;		/**< DeQueue of messages to be send.*/

			
			void*							_image_buffer;
			void*							_image_buffer_aux;
			void*							_image_buffer_out;
			int*							_image_size_out;
				
			int								_buffer_idx;
			int								_image_counter;
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

			int new_image();
			void get_image(void* data);

		private:
			void run_tcp_thread();
			void run_maintenance_thread();

			void handle_accept(TCPMessageSession* new_session, const boost::system::error_code& error);

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
		
			std::deque<TCPMessageSession*>				_message_sessions;
			
		
			std::deque<std::pair<int,std::string*>>		_messages;		/**< DeQueue of messages to be send.*/

			//bool									_tcp_flag;
			//TCPMessageSession*					_tcp_gui_socket;
		
			bool										_running;
			boost::thread*								_maintenance_thread;
			boost::mutex								_mutex;			/**< Mutex for the contition _condition.*/
			boost::condition_variable					_condition;		/**< Condition variable to wait for new messages.*/

			std::deque<std::pair<int,std::string*>>		_messages_received;		/**< DeQueue of messages to be send.*/
			
			//Image/Video
			std::deque<TCPMessageSession*>				_video_sessions;
			int											_image_size;
			void*										_image_buffer;//[691200];
	};
}
#endif//_TOOLBOX_IO_TCP_SERVER