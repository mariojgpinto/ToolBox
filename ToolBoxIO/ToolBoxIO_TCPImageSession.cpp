/** 
 * @file	TCPImageSession.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Implementation of the TCPImageSession class.
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
TCPImageSession::TCPImageSession(boost::asio::io_service* io_service, int image_size){
	for(int i = 0 ; i < TCPImageSession::_status_size ; ++i){
		this->_status[i] = false;
	}

	this->_id = -1;
	this->_buffer_idx = 0;
	this->_image_counter = 0;

	this->_buffer_size = image_size;
	this->_image_buffer = malloc(image_size);
	this->_image_buffer_aux = malloc(image_size);
	this->_image_buffer_out = NULL;

	if(io_service){
		this->_socket = new boost::asio::ip::tcp::socket(*io_service);
	}
}

/**
 * @brief	.
 * @details	.
 */
TCPImageSession::~TCPImageSession(){
	this->stop_session();
}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::set_id(int id){
	this->_id = id;
}

/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::stop_session(){
	this->_status[TCPImageSession::CONNECTED] = false;
	this->_status[TCPImageSession::RUNNING]   = false;

	Sleep(100);
	
	if(this->_socket){
		try{
			this->_socket->close();
		}
		catch(boost::exception& e){
			printf("Exception at TCPImageSession(%d): TCPImageSession::stop_client\n",this->_id);
		}
	}
}

void TCPImageSession::set_output_image_buffer(void **data, int *size){
	if(data){
		this->_status[TCPImageSession::OUT_IMAGE] = true;
		this->_image_buffer_out = *data;
		this->_image_size_out = size;

		if(this->_buffer_size != *size){
			this->_buffer_size = *size;
			
			free(this->_image_buffer);
			free(this->_image_buffer_aux);

			this->_image_buffer		= malloc(this->_buffer_size);
			this->_image_buffer_aux = malloc(this->_buffer_size);
		}
	}
}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::start(){
	if(this->_socket->is_open()){
		this->_status[TCPImageSession::CONNECTED] = true;
	}
	else{
		this->_status[TCPImageSession::CONNECTED] = false;
		return;
	}

	boost::asio::socket_base::receive_buffer_size option(this->_buffer_size);
	this->_socket->set_option(option);
	this->_socket->async_read_some(	boost::asio::buffer(_buffer, this->_buffer_size),
									boost::bind(&TCPImageSession::handle_read, this,
									boost::asio::placeholders::error,
									boost::asio::placeholders::bytes_transferred));

	this->_thread = new boost::thread(&TCPImageSession::run_thread,this);
}

/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::run_thread(){
	this->_status[TCPImageSession::RUNNING] = true;
	
	while(this->_status[TCPImageSession::RUNNING] = true){
		if(!this->_status[TCPImageSession::CONNECTED]){
			this->_status[TCPImageSession::RUNNING] = false;
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

	this->_status[TCPImageSession::RUNNING] = FALSE;
}


//-----------------------------------------------------------------------------
// MESSAGES
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::add_message(std::string msg){
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
void TCPImageSession::handle_read(const boost::system::error_code& error, size_t bytes_transferred){
	if (!error)	{
		if(bytes_transferred <= this->_buffer_size){
			if((this->_buffer_idx + bytes_transferred) > this->_buffer_size){
				//printf("Adjust c(%d),n(%d)",this->_buffer_idx,bytes_transferred);
				int buffer_size_first = this->_buffer_size - this->_buffer_idx;
				int buffer_size_second = bytes_transferred - buffer_size_first; //(_temp_idx + bytes_transferred) - (640*480*3);
				//printf("f(%d),s(%d)",buffer_size_first,buffer_size_second);

				//Complete buffer
				memcpy((void*)((char*)this->_image_buffer_aux + this->_buffer_idx),this-> _buffer, buffer_size_first);
				this->_buffer_idx += buffer_size_first;
				//printf("t(%d)",this->_buffer_idx);

				//Copy image to Server and reset variables
				this->save_new_image();

				//Copy buffer
				memcpy(	this->_image_buffer_aux, (void*)((char*)this->_buffer+buffer_size_first), buffer_size_second);
				this->_buffer_idx += buffer_size_second;
				//printf("e(%d)\n",this->_buffer_idx);				
					
			}
			else{
				//printf("idx(%d)  ",_temp_idx);
				//printf("copy image- size(%d)\n",bytes_transferred);
				memcpy((void*)((char*)this->_image_buffer_aux + this->_buffer_idx), _buffer, bytes_transferred);
				this->_buffer_idx += bytes_transferred;

				if(this->_buffer_idx == this->_buffer_size){
					this->save_new_image();
				}
				else{
					if(this->_buffer_idx > this->_buffer_size){
						printf("IT SHOULD NOT ENTER HERE\n", this->_buffer_idx);
						printf("Error bytes read (%d)\n", this->_buffer_idx);
					}
				}
			}	
		}
		else{
			//printf("Adjust c(%d),n(%d)",this->_buffer_idx,bytes_transferred);
			int buffer_init = 0;
			int buffer_size_first = 0;// = this->_buffer_size - this->_buffer_idx;
			int buffer_size_second = 0;// = bytes_transferred - buffer_size_first; //(_temp_idx + bytes_transferred) - (640*480*3);
			//printf("f(%d),s(%d)",buffer_size_first,buffer_size_second);

			do{
				buffer_size_first = this->_buffer_size - this->_buffer_idx;
	
				//Complete buffer
				memcpy((void*)((char*)this->_image_buffer_aux + this->_buffer_idx),
									  this->_buffer + buffer_init, 
									  buffer_size_first);

				this->_buffer_idx += buffer_size_first;
				//printf("t(%d)",this->_buffer_idx);

				//Copy image to Server and reset variables
				if(this->_buffer_idx == this->_buffer_size){
					this->save_new_image();
				}

				//printf("e(%d)\n",this->_buffer_idx);	
				buffer_init += buffer_size_first;
				buffer_size_first = this->_buffer_size - this->_buffer_idx;
				buffer_size_second = bytes_transferred - buffer_init;
			}
			while(buffer_size_second > this->_buffer_size);

			if(buffer_size_second > 0){
				//copy rest of the buffer
				memcpy(	this->_image_buffer_aux, (void*)((char*)this->_buffer+buffer_size_first), buffer_size_second);
				this->_buffer_idx += buffer_size_second;
			}
		}

		_socket->async_read_some(	boost::asio::buffer(_buffer, this->_buffer_size),
									boost::bind(&TCPImageSession::handle_read, this,
									boost::asio::placeholders::error,
									boost::asio::placeholders::bytes_transferred));
    }
	else{
		//delete this;
		this->stop_session();
    }
}

/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::handle_write(const boost::system::error_code& error){
	if (!error){

    }
    else{
		//delete this;
		printf("Error on Writting\n");
		this->stop_session();
    }
}

/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::save_new_image(){
	memcpy(this->_image_buffer, this->_image_buffer_aux, this->_buffer_size);
	this->_image_counter++;
	_buffer_idx = 0;

	if(this->_status[TCPImageSession::OUT_IMAGE]){
		memcpy(this->_image_buffer_out, this->_image_buffer, this->_buffer_size);
		*this->_image_size_out = this->_buffer_size;
	}
	//printf("Image %d read (%d bytes)\n",this->_image_counter, this->_buffer_size);
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
int TCPImageSession::get_id(){
	return this->_id;
}

/**
 * @brief	.
 * @details	.
 */
bool TCPImageSession::is_connected(){
	return this->_status[TCPImageSession::CONNECTED];
}

/**
 * @brief	.
 * @details	.
 */
bool TCPImageSession::is_running(){
	return this->_status[TCPImageSession::RUNNING];
}

/**
 * @brief	.
 * @details	.
 */
boost::asio::ip::tcp::socket* TCPImageSession::get_socket(){
	return _socket;
}

}//namespace ToolBoxIO