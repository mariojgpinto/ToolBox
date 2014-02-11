/** 
 * @file	TCPServer.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Implementation of the TCPServer class.
 */
#include "ToolBoxIO_TCPServer.h"

#include <iostream>

namespace ToolBoxIO{

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
TCPServer::TCPServer(int port){
	this->_port_short = port;
	this->_host = "localhost";

	this->init_server();

	this->_io_thread = new boost::thread(&TCPServer::run_tcp_thread,this);
	this->_maintenance_thread = new boost::thread(&TCPServer::run_maintenance_thread,this);
}

/**
 * @brief	.
 * @details	.
 */
TCPServer::~TCPServer(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPServer::init_server(){
	this->_io_service = new boost::asio::io_service();
	this->_acceptor = new boost::asio::ip::tcp::acceptor(*_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), this->_port_short));

	TCPSocketSession* next_session = new TCPSocketSession(this->_io_service);
	this->_acceptor->async_accept(	*next_session->get_socket(),
									boost::bind(&TCPServer::handle_accept, 
												this, next_session,
												boost::asio::placeholders::error));

	std::cout << "Socket Server Initialized on Port " << this->_port << ".\n";
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::stop_server(){
	this->_running = false;
	this->_condition.notify_all();

	for(int i = 0 ; i < this->_sessions.size() ; ++i){
		this->_sessions[i]->get_socket()->close();
		this->_sessions[i]->~TCPSocketSession();
	}
	this->_sessions.clear();

	this->_io_service->stop();
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::handle_accept(TCPSocketSession* new_session, const boost::system::error_code& error){
	if (!error){
		this->_sessions.push_back(new_session);
		this->_sessions[this->_sessions.size()-1]->set_id(this->_sessions.size()-1);
		this->_sessions[this->_sessions.size()-1]->set_output_message_deque(&this->_messages_received);
		this->_sessions[this->_sessions.size()-1]->start();

		this->_condition.notify_all();
		//if(this->_message_buffer.size()){
		//for(int i = 0 ; i < this->_message_buffer.size() ; ++i){
		//		this->add_message(*this->_message_buffer[i]);
		//		printf("Message on Buffer Sent (%s)\n",this->_message_buffer[i]->data());
		//	}
		//	this->_message_buffer.clear();
		//}
		
		
		TCPSocketSession* next_session = new TCPSocketSession(_io_service);
		this->_acceptor->async_accept(	*next_session->get_socket(),
										boost::bind(&TCPServer::handle_accept, this, next_session,
										boost::asio::placeholders::error));
    }
    else{
		delete new_session;
    }
}


//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPServer::run_tcp_thread(){
	this->_io_service->run();
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::run_maintenance_thread(){
	this->_running = true;

	while(this->_running){
		this->manage_sessions();
		this->manage_messages();

		{
			boost::mutex::scoped_lock lock(this->_mutex);
			this->_condition.timed_wait(lock,boost::posix_time::millisec(1000));
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::manage_sessions(){
	for(int i = 0 ; i < this->_sessions.size() ; ++i){
		if(!this->_sessions[i]->is_connected()){
			this->_sessions.erase(this->_sessions.begin() + i);
			i--;
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
void TCPServer::manage_messages(){
	if(this->_messages.size() && this->_sessions.size()){
		while(this->_messages.size()){
			if(this->_messages[0].second){
				this->add_message(*this->_messages[0].second,this->_messages[0].first);
			}

			this->_messages.pop_front();
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::add_message(std::string msg, int id){
	if(!this->_sessions.size()){
		this->add_message_to_queue(msg,id);
	}
	else{
		if(id == -1){
			for(int i = 0 ; i < this->_sessions.size() ; ++i){
				this->_sessions[i]->add_message(msg);
			}
		}
		else{
			if(id >= 0 && id < this->_sessions.size()){
				if(this->_sessions[id]->is_connected()){
					this->_sessions[id]->add_message(msg);
				}
				else{
					this->add_message_to_queue(msg,id);
				}
			}
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::add_message_to_queue(std::string msg, int id){
	this->_messages.push_back(std::pair<int,std::string*>(id,new std::string(msg)));
}


//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
short TCPServer::get_port(){
	return this->_port_short;
}

/**
 * @brief	.
 * @details	.
 */
std::string* TCPServer::get_ip(){
	return &this->_host;
}

/**
 * @brief	.
 * @details	.
 */
bool TCPServer::new_message(){
	return (this->_messages_received.size()) ? true : false;
}

/**
 * @brief	.
 * @details	.
 */
void TCPServer::consume_message(std::string& message){
	if(this->_messages_received.size()){
		if(this->_messages_received[0].second){
			message.assign(this->_messages_received[0].second->data());
		}
		this->_messages_received.pop_front();
	}
}

}//namespace ToolBoxIO