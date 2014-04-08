/** 
 * @file	TCPClient.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Implementation of the TCPClient class.
 */
#include "ToolBoxIO.h"

namespace ToolBoxIO {

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	TCPClient(char* host, char* port).
 * @details TCPClient(char* host, char* port).
 */
TCPClient::TCPClient(char* host, char* port, bool threaded){
	this->_flag_connected = false;

	this->_host.assign(host);
	this->_port.assign(port);

	this->_threaded = threaded;
	if(threaded){
		this->_thread = new boost::thread(&TCPClient::run,this);
	}
}

/**
 * @brief	~TCPClient().
 * @details ~TCPClient().
 */
TCPClient::~TCPClient(){

}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	connect().
 * @details connect().
 */
bool TCPClient::connect(){
	this->_io_service = new boost::asio::io_service();

	boost::asio::ip::tcp::resolver				resolver(*this->_io_service);
	boost::asio::ip::tcp::resolver::query		query(boost::asio::ip::tcp::v4(), this->_host, this->_port);
	boost::asio::ip::tcp::resolver::iterator	iterator = resolver.resolve(query);

	this->_socket = new boost::asio::ip::tcp::socket(*this->_io_service);
	//this->_socket->connect(
	boost::asio::connect(*this->_socket, iterator);

	this->_flag_connected = true;

	return true;
}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	run().
 * @details run().
 */
void TCPClient::run(){
	if(this->connect()){
		this->_running = true;
		while(this->_running){
			{
				boost::mutex::scoped_lock lock(this->_mutex);
				this->_condition.wait(lock);
			}

			if(!this->_flag_connected){
				this->_running = false;
			}
			else{
				while(this->_messages.size()){
					if(this->_messages[0]){
						try{
							size_t request_length = this->_messages[0]->length();
							boost::asio::write(*this->_socket, boost::asio::buffer(this->_messages[0]->data(), request_length));
						}
						catch(boost::exception& e){
							printf("Exception at: TCPClient::run\nStopping Client\n");
							this->stop_client();
						}
					}

					this->_messages.pop_front();
				}
			}
		}
	}
	else{
		this->_running = false;
	}
}

//-----------------------------------------------------------------------------
// INTERACTION
//-----------------------------------------------------------------------------
/**
 * @brief	add_message(std::string msg).
 * @details add_message(std::string msg).
 */
void TCPClient::add_message(std::string msg){
	if(this->_threaded){
		this->_messages.push_back(new std::string(msg));
		this->_condition.notify_all();
	}
	else{
		if(this->_flag_connected){
			try{
				size_t request_length = this->_messages[0]->length();
				boost::asio::write(*this->_socket, boost::asio::buffer(this->_messages[0]->data(), request_length), boost::asio::transfer_exactly(request_length));
			}
			catch(boost::exception& e){
				printf("Exception at: TCPClient::add_message\nStopping Client\n");
				this->stop_client();
			}
		}
	}
}

void TCPClient::add_message(int size, void* data){
	if(this->_threaded){
		try{
			printf("Sending Data\n");
			size_t request_length = size; 
			//boost::asio::write(*this->_socket, boost::asio::buffer("[START]", 7));
			boost::asio::write(*this->_socket, boost::asio::buffer(data, request_length));
			//boost::asio::write(*this->_socket, boost::asio::buffer("[END]", 5));
		}
		catch(boost::exception& e){
			printf("Exception at: TCPClient::add_message2\nStopping Client\n");
			this->stop_client();
		}
	}
	else{
		if(this->_flag_connected && size && data){
			try{
				//printf("Sending Data\n");
				size_t request_length = size;
				boost::asio::write(*this->_socket, boost::asio::buffer(data, request_length));
			}
			catch(boost::exception& e){
				printf("Exception at: TCPClient::add_message2\nStopping Client\n");
				this->stop_client();
			}
		}
	}
}
/**
 * @brief	is_connected().
 * @details is_connected().
 */
bool TCPClient::is_connected(){
	return this->_flag_connected;
}

/**
 * @brief	stop_client().
 * @details stop_client().
 */
void TCPClient::stop_client(){
	this->_running = false;
	this->_flag_connected = false;
	
	try{
		this->_socket->close();
		this->_io_service->stop();
	}
	catch(boost::exception& e){
		printf("Exception at: TCPClient::stop_client\n DAFUQ?!\n");
	}
}

}