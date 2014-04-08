/** 
 * @file	TCPMessageSession.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Implementation of the TCPMessageSession class.
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
TCPMessageSession::TCPMessageSession(boost::asio::io_service* io_service){
	this->_connected = false;
	this->_id = -1;
	this->_temp_idx = 0;
	this->_image_buffer_temp = malloc(buff_size);

	if(io_service){
		this->_socket = new boost::asio::ip::tcp::socket(*io_service);
	}
}

/**
 * @brief	.
 * @details	.
 */
TCPMessageSession::~TCPMessageSession(){
	this->stop_session();
}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::set_id(int id){
	this->_id = id;
}

/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::stop_session(){
	this->_running = false;
	this->_connected = false;
	
	if(this->_socket){
		try{
			this->_socket->close();
		}
		catch(boost::exception& e){
			printf("Exception at TCPMessageSession(%d): TCPMessageSession::stop_client\n",this->_id);
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::set_output_message_deque(std::deque<std::pair<int,std::string*>>* deque){
	if(deque){
		this->_messages_read = deque;
	}
}

void TCPMessageSession::set_output_image_buffer(void **data, int *size){
	if(data){
		this->_image_buffer = *data;
		this->_image_size = size;
	}
}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::start(){
	this->_connected = true;

	boost::asio::socket_base::receive_buffer_size option(buff_size);
	this->_socket->set_option(option);

	this->_socket->async_read_some(	boost::asio::buffer(_buffer, buff_size),
									boost::bind(&TCPMessageSession::handle_read, this,
									boost::asio::placeholders::error,
									boost::asio::placeholders::bytes_transferred));

	this->_thread = new boost::thread(&TCPMessageSession::run_thread,this);
}

/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::run_thread(){
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
void TCPMessageSession::add_message(std::string msg){
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
void TCPMessageSession::handle_read(const boost::system::error_code& error, size_t bytes_transferred){
	static int ac = 0;
	if (!error)	{
		//if(bytes_transferred < 1000){
		//	_messages_read->push_back(std::pair<int,std::string*>(this->_id,new std::string(_buffer,bytes_transferred)));
		//	//printf("Read \"%s\" - size(%d)\n",_messages_read->at(_messages_read->size()-1).second->data(),bytes_transferred);

		//} else{
			
			//if(_temp_idx == 0){
			//	std::string start(_buffer,7);
			//	printf("%s ",start.data());

			//	memcpy((void*)((char*)_image_buffer_temp+7),_buffer,bytes_transferred-7);
			//	_temp_idx += bytes_transferred-7;
			//}
			//else{
			//	if(_temp_idx + bytes_transferred > 640*480*3){
			//		int temp = 640*480*3 - _temp_idx;
			//		int temp2 = (_temp_idx + bytes_transferred) - (640*480*3);

			//		memcpy((void*)((char*)_image_buffer_temp+_temp_idx),_buffer,temp);
			//	
			//		_temp_idx += temp;

			//		memcpy(_image_buffer,_image_buffer_temp,_temp_idx);
			//		*_image_size = _temp_idx;
			//		_temp_idx = 0;

			//		std::string end(_buffer+temp,5);
			//		printf("%s ",end.data());

			//		std::string start(_buffer+temp,7);
			//		printf("%s ",start.data());

			//		memcpy((void*)((char*)_image_buffer_temp+_temp_idx),(void*)((char*)_buffer+temp+5+7),temp2-5-7);
			//	
			//		_temp_idx += temp2-5-7;
			//		memcpy(_image_buffer,_image_buffer_temp,_temp_idx);
			//		*_image_size = _temp_idx;
			//		_temp_idx = 0;
			//	}
			//	else{
			//		memcpy((void*)((char*)_image_buffer_temp+_temp_idx),_buffer,bytes_transferred);

			//		_temp_idx += bytes_transferred;
			//	}
			//}

			//printf("Read - size(%d) ",bytes_transferred);
			//printf("\n");
			if(bytes_transferred <= buff_size){
				if((_temp_idx + bytes_transferred) > buff_size){
					printf("Adjust c(%d),n(%d)",_temp_idx,bytes_transferred);
					int buffer_size_first = buff_size - _temp_idx;
					int buffer_size_second = bytes_transferred - buffer_size_first; //(_temp_idx + bytes_transferred) - (640*480*3);
					printf("f(%d),s(%d)",buffer_size_first,buffer_size_second);

					//Complete buffer
					memcpy((void*)((char*)_image_buffer_temp+_temp_idx),_buffer,buffer_size_first);
					_temp_idx += buffer_size_first;
					printf("t(%d)",_temp_idx);

					//Copy image to Server and reset variables
					memcpy(_image_buffer,_image_buffer_temp,_temp_idx);
					*_image_size = _temp_idx;
					_temp_idx = 0;

					//Copy buffer
					memcpy(	_image_buffer_temp, (void*)((char*)_buffer+buffer_size_first), buffer_size_second);
					_temp_idx += buffer_size_second;
					printf("e(%d)\n",_temp_idx);				
					
				}
				else{
					//printf("idx(%d)  ",_temp_idx);
					//printf("copy image- size(%d)\n",bytes_transferred);
					memcpy((void*)((char*)_image_buffer_temp+_temp_idx),_buffer,bytes_transferred);
					_temp_idx += bytes_transferred;

					if(_temp_idx == buff_size){
						printf("Image %d read (%d bytes)\n",ac++,_temp_idx);
				
						memcpy(_image_buffer,_image_buffer_temp,_temp_idx);
						*_image_size = _temp_idx;
						_temp_idx = 0;
					}
					else{
						if(_temp_idx > buff_size)
							printf("Error bytes read (%d)\n", _temp_idx);
					}
				}	
			}
			else{
				printf("Adjust c(%d),n(%d)",_temp_idx,bytes_transferred);
				int buffer_size_first = buff_size - _temp_idx;
				int buffer_size_second = bytes_transferred - buffer_size_first; //(_temp_idx + bytes_transferred) - (640*480*3);
				printf("f(%d),s(%d)",buffer_size_first,buffer_size_second);

				//Complete buffer
				memcpy((void*)((char*)_image_buffer_temp+_temp_idx),_buffer,buffer_size_first);
				_temp_idx += buffer_size_first;
				printf("t(%d)",_temp_idx);

				//Copy image to Server and reset variables
				memcpy(_image_buffer,_image_buffer_temp,_temp_idx);
				*_image_size = _temp_idx;
				_temp_idx = 0;

				//Copy buffer
				memcpy(	_image_buffer_temp, (void*)((char*)_buffer+buffer_size_first), buffer_size_second);
				_temp_idx += buffer_size_second;
				printf("e(%d)\n",_temp_idx);				

				printf("Error bytes read (%d)\n", bytes_transferred);
				//getchar();
			}
		//}
		_socket->async_read_some(	boost::asio::buffer(_buffer, buff_size),
									boost::bind(&TCPMessageSession::handle_read, this,
									boost::asio::placeholders::error,
									boost::asio::placeholders::bytes_transferred));
    }
	else{
		//delete this;
		this->stop_session();
    }
}

void TCPMessageSession::handle_write(const boost::system::error_code& error){
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
int TCPMessageSession::get_id(){
	return this->_id;
}

/**
 * @brief	.
 * @details	.
 */
bool TCPMessageSession::is_connected(){
	return this->_connected;
}

/**
 * @brief	.
 * @details	.
 */
boost::asio::ip::tcp::socket* TCPMessageSession::get_socket(){
	return _socket;
}

}//namespace ToolBoxIO