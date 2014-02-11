/** 
 * @file	TCPSocketSession.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Implementation of the TCPSocketSession class.
 */
#include "ToolBoxIO_TCPServer.h"

namespace ToolBoxIO{

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
TCPSocketSession::TCPSocketSession(boost::asio::io_service* io_service){
	this->_connected = false;
	this->_id = -1;

	if(io_service){
		this->_socket = new boost::asio::ip::tcp::socket(*io_service);
	}
}

/**
 * @brief	.
 * @details	.
 */
TCPSocketSession::~TCPSocketSession(){
	this->stop_session();
}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPSocketSession::set_id(int id){
	this->_id = id;
}

/**
 * @brief	.
 * @details	.
 */
void TCPSocketSession::stop_session(){
	this->_running = false;
	this->_connected = false;
	
	if(this->_socket){
		try{
			this->_socket->close();
		}
		catch(boost::exception& e){
			printf("Exception at TCPSocketSession(%d): TCPSocketSession::stop_client\n",this->_id);
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPSocketSession::set_output_message_deque(std::deque<std::pair<int,std::string*>>* deque){
	if(deque){
		this->_messages_read = deque;
	}
}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPSocketSession::start(){
	this->_connected = true;

	boost::asio::socket_base::receive_buffer_size option(buff_size);
	this->_socket->set_option(option);

	this->_socket->async_read_some(	boost::asio::buffer(_buffer, buff_size),
									boost::bind(&TCPSocketSession::handle_read, this,
									boost::asio::placeholders::error,
									boost::asio::placeholders::bytes_transferred));

	this->_thread = new boost::thread(&TCPSocketSession::run_thread,this);
}

/**
 * @brief	.
 * @details	.
 */
void TCPSocketSession::run_thread(){
	this->_running = true;
	while(this->_running){
		if(!this->_connected){
			this->_running = false;
		}
		else{
			this->_mutex_write.lock();
			while(this->_messages_to_write.size()){
				if(this->_messages_to_write[0]){
					try{
						size_t request_length = this->_messages_to_write[0]->length();
						boost::asio::write(*this->_socket, boost::asio::buffer(this->_messages_to_write[0]->data(), request_length));
					}
					catch(boost::exception& e){
						printf("Exception at: TCPClient::run\nStopping Client\n");
						this->stop_session();
					}
				}

				this->_messages_to_write.pop_front();
			}
			this->_mutex_write.unlock();
		}
		
		{
			boost::mutex::scoped_lock lock(this->_mutex);
			this->_condition.wait(lock);
		}
	}
}


//-----------------------------------------------------------------------------
// MESSAGES
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPSocketSession::add_message(std::string msg){
	this->_mutex_write.lock();
		this->_messages_to_write.push_back(new std::string(msg));
	this->_mutex_write.unlock();
	
	this->_condition.notify_all();
}


//-----------------------------------------------------------------------------
// HANDLE
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPSocketSession::handle_read(const boost::system::error_code& error, size_t bytes_transferred){
	if (!error)	{
		_messages_read->push_back(std::pair<int,std::string*>(this->_id,new std::string(_buffer,bytes_transferred)));

		_socket->async_read_some(	boost::asio::buffer(_buffer, buff_size),
									boost::bind(&TCPSocketSession::handle_read, this,
									boost::asio::placeholders::error,
									boost::asio::placeholders::bytes_transferred));
    }
	else{
		//delete this;
		this->stop_session();
    }
}

void TCPSocketSession::handle_write(const boost::system::error_code& error){
	if (!error){

    }
    else{
		//delete this;
		printf("Error on Writting\n");
		this->stop_session();
    }
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
int TCPSocketSession::get_id(){
	return this->_id;
}

/**
 * @brief	.
 * @details	.
 */
bool TCPSocketSession::is_connected(){
	return this->_connected;
}

/**
 * @brief	.
 * @details	.
 */
boost::asio::ip::tcp::socket* TCPSocketSession::get_socket(){
	return _socket;
}

}//namespace ToolBoxIO