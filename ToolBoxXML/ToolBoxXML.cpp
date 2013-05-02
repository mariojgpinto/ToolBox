#include "ToolBoxXML.h"

#include "ToolBoxXMLMacros.h"

namespace ToolBoxXML{
	/**
	 * @brief	Creates a XML root document.
	 * @details	.
	 */
	tinyxml2::XMLDocument* create_xml_doc(){
		tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();

		doc->NewDeclaration();

		return doc;
	}

	//-----------------------------------------------------------------------------
	// FOLDERS
	//-----------------------------------------------------------------------------
	/**
	 * @brief	Creates a folder with the given name.
	 * @details	.
	 *
	 * @param	name
	 *			Name of the folder to be created.
	 */
	bool create_folder(std::string* name){
		if(!name) return false;

		char cmd[_X_STRSIZE];
		strcpy(_X_MKDIR,cmd);
		strcat((char*)name->data(),cmd);

		system(cmd);

		return true;
	}

	/**
	 * @brief	Creates a folder within the a main folder.
	 * @details	.
	 *
	 * @param	main_folder
	 *			.
	 *
	 * @param	name
	 *			.
	 *
	 * @todo	Check if main_folder already exists.
	 * @todo	Check if main_folder already have the end slash before adding it.
	 */
	bool create_sub_folder(std::string* main_folder, std::string* name){
		if(!main_folder || !name) return false;

		char cmd[_X_STRSIZE];
		strcpy(_X_MKDIR,cmd);
		strcat((char*)main_folder->data(),cmd);
		strcat(_X_SLASH,cmd);
		strcat((char*)name->data(),cmd);

		system(cmd);

		return true;
	}

	/**
	 * @brief	Creates the folder and the required sub-folders
	 * @details	.
	 *
	 * @param	main_folder
	 *			.
	 *
	 * @param	sub_folders
	 *			.
	 *
	 * @return	
	 *
	 */
	bool create_folders(std::string* main_folder, std::vector<std::string*>* sub_folders){
		if(!main_folder || !sub_folders) return false;

		char main_mkdir[_X_STRSIZE];
		strcpy(_X_MKDIR,main_mkdir);
		strcat((char*)main_folder->data(),main_mkdir);

		system(main_mkdir);

		strcat(_X_SLASH,main_mkdir);

		for(unsigned int i = 0 ; i < sub_folders->size() ; i++){
			if(sub_folders->at(i)){
				char sub_mkdir[_X_STRSIZE];
				strcpy(main_mkdir,sub_mkdir);
				strcat((char*)sub_folders->at(i)->data(),sub_mkdir);
				system(sub_mkdir);
			}
		}

		return true;
	}


	//-----------------------------------------------------------------------------
	// OPENCV
	//-----------------------------------------------------------------------------
	/**
	 * @brief	Adds a cv::Size to the XML element.
	 * @details	.
	 *
	 * @param	elem
	 *			XML element where the attribute will be added.
	 *
	 * @param	size
	 *			Attribute to be added.
	 *
	 * @param	name
	 *			Name of the attribute to be added (not necessary).
	 */
	bool cv_add_size(tinyxml2::XMLElement* elem, cv::Size *size, char* name){
		if(!elem || !size) return false;

		char size_w[256]; 
		char size_h[256]; 

		if(name){
			strcpy(size_w,name); strcat(size_w,"_");
            strcpy(size_h,name); strcat(size_h,"_");
			strcat(size_w,_X_CV_SIZE_WIDTH);
			strcat(size_h,_X_CV_SIZE_HEIGHT);
		}
		else{
			strcpy(size_w,_X_CV_SIZE_WIDTH);
			strcpy(size_h,_X_CV_SIZE_HEIGHT);
		}


		elem->SetAttribute(size_w, size->width);
		elem->SetAttribute(size_h, size->height);

		return true;
	}

	/**
	 * @brief	Adds a cv::Point to the XML element.
	 * @details	.
	 *
	 * @param	elem
	 *			XML element where the attribute will be added.
	 *
	 * @param	size
	 *			Attribute to be added.
	 *
	 * @param	name
	 *			Name of the attribute to be added (not necessary).
	 */
	bool cv_add_point(tinyxml2::XMLElement* elem, cv::Point *point, char* name){
		if(!elem || !point) return false;

		char point_x[256]; 
		char point_y[256]; 

		if(name){
			strcpy(point_x,name); strcat(point_x,"_");
			strcpy(point_y,name); strcat(point_y,"_");
			strcat(point_x,_X_CV_POINT_X);
			strcat(point_y,_X_CV_POINT_Y);
		}
		else{
			strcpy(point_x,_X_CV_POINT_X);
			strcpy(point_y,_X_CV_POINT_Y);
		}

		elem->SetAttribute(point_x, point->x);
		elem->SetAttribute(point_y, point->y);

		return true;
	}

	/**
	 * @brief	Adds a cv::Point to the XML element.
	 * @details	.
	 *
	 * @param	elem
	 *			XML element where the attribute will be added.
	 *
	 * @param	size
	 *			Attribute to be added.
	 *
	 * @param	name
	 *			Name of the attribute to be added (not necessary).
	 */
	bool cv_add_point(tinyxml2::XMLElement* elem, cv::Point3f *point, char* name){
		if(!elem || !point) return false;

		char point_x[256]; 
		char point_y[256]; 
		char point_z[256]; 

		if(name){
			strcpy(point_x,name); strcat(point_x,"_");
			strcpy(point_y,name); strcat(point_y,"_");
			strcpy(point_z,name); strcat(point_z,"_");
			strcat(point_x,_X_CV_POINT_X);
			strcat(point_y,_X_CV_POINT_Y);
			strcat(point_z,_X_CV_POINT_Z);
		}
		else{
			strcpy(point_x,_X_CV_POINT_X);
			strcpy(point_y,_X_CV_POINT_Y);
			strcpy(point_z,_X_CV_POINT_Z);
		}

		elem->SetAttribute(point_x, point->x);
		elem->SetAttribute(point_y, point->y);
		elem->SetAttribute(point_z, point->z);

		return true;
	}


	bool cv_read_size(tinyxml2::XMLElement* elem, cv::Size *size, char* name){
		//size = NULL;
		if(!elem || !size) return false;

		int error = 0; 
		int w,h;

		char size_w[256]; 
		char size_h[256]; 

		if(name){
			strcpy(size_w,name); strcat(size_w,"_");
            strcpy(size_h,name); strcat(size_h,"_");
			strcat(size_w,_X_CV_SIZE_WIDTH);
			strcat(size_h,_X_CV_SIZE_HEIGHT);
		}
		else{
			strcpy(size_w,_X_CV_SIZE_WIDTH);
			strcpy(size_h,_X_CV_SIZE_HEIGHT);
		}

		error += elem->QueryIntAttribute(size_w,&w);
		error += elem->QueryIntAttribute(size_h,&h);

		if(error) return false;

		size->height = h;
		size->width = w;
		//= new cv::Size(w,h);

		return true;
	}

	bool cv_read_point(tinyxml2::XMLElement* elem, cv::Point *point, char* name){
		if(!elem || !point) return false;

		int error = 0;
		int x,y;
		
		char point_x[256]; 
		char point_y[256]; 

		if(name){
			strcpy(point_x,name); strcat(point_x,"_");
			strcpy(point_y,name); strcat(point_y,"_");
			strcat(point_x,_X_CV_POINT_X);
			strcat(point_y,_X_CV_POINT_Y);
		}
		else{
			strcpy(point_x,_X_CV_POINT_X);
			strcpy(point_y,_X_CV_POINT_Y);
		}

		error += elem->QueryIntAttribute(point_x,&x);
		error += elem->QueryIntAttribute(point_y,&y);

		if(error) return false;

		//point = new cv::Point(x,y);
		point->x = x;
		point->y = y;

		return true;
	}

	bool cv_read_point(tinyxml2::XMLElement* elem, cv::Point3f *point, char* name){
		if(!elem || !point) return false;

		int error = 0; 
		double x,y,z;

		char point_x[256]; 
		char point_y[256]; 
		char point_z[256]; 

		if(name){
			strcpy(point_x,name); strcat(point_x,"_");
			strcpy(point_y,name); strcat(point_y,"_");
			strcpy(point_z,name); strcat(point_z,"_");
			strcat(point_x,_X_CV_POINT_X);
			strcat(point_y,_X_CV_POINT_Y);
			strcat(point_z,_X_CV_POINT_Z);
		}
		else{
			strcpy(point_x,_X_CV_POINT_X);
			strcpy(point_y,_X_CV_POINT_Y);
			strcpy(point_z,_X_CV_POINT_Z);
		}

		error += elem->QueryDoubleAttribute(point_x,&x);
		error += elem->QueryDoubleAttribute(point_y,&y);
		error += elem->QueryDoubleAttribute(point_z,&z);

		if(error) return false;

		//point = new cv::Point3f((float)x,(float)y,(float)z);
		point->x = x;
		point->y = y;
		point->z = z;

		return true;
	}	
	//-----------------------------------------------------------------------------
	// OPENCV IMAGES
	//-----------------------------------------------------------------------------
	/**
	 * @brief	Writes a cv::Mat image to the disk.
	 * @details	.
	 *
	 * @param	image_root
	 *			Root XML element where the image_elem will be linked.
	 *
	 * @param	image_elem
	 *			XML element where the image's path will be added.
	 *
	 * @param	main_path
	 *			Name of the path where the image will be added.
	 *
	 * @param	image_name
	 *			Name of the image file to be saved.
	 *
	 * @param	image
	 *			Image to be recorded.
	 *
	 * @return	The success of the operation.
	 * @retval	true
	 *			If the image was recorded with success.
	 * @retval	false
	 *			If some error occured.
	 */
	bool cv_save_image(tinyxml2::XMLElement* image_root, tinyxml2::XMLElement* image_elem, char* main_path, char* image_name, cv::Mat& image){
		if(!image_root || !image_elem || !main_path || !image_name) return false;

		if(!image.data) return false;

		char path[_X_STRSIZE];
		strcpy(path,main_path);
		strcat(path,_X_SLASH);
		strcat(path,image_name);

		image_elem->SetAttribute(_X_PATH, path);

		cv::imwrite(path,image);

		image_root->LinkEndChild(image_elem);

		return true;
	}

	bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat1b& out_image){
		tinyxml2::XMLElement *image = root->FirstChildElement(elem);

		if(image){
			const char* path = image->Attribute(_X_PATH);
			if(path){ 
				FILE * pFile = fopen (path,"r");

				if(pFile){
					fclose(pFile);

					cv::Mat aux = cv::imread(path,0);
					aux.copyTo(out_image);

					return true;
				}
				else
					return false;
			}
			else
				return false;
		}
		else{
			return false;
		}
	}

	bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat1f& out_image){
		tinyxml2::XMLElement *image = root->FirstChildElement(elem);

		if(image){
			const char* path = image->Attribute(_X_PATH);
			if(path){ 
				FILE * pFile = fopen (path,"r");

				if(pFile){
					fclose(pFile);

					cv::Mat aux = cv::imread(path,0);
					aux.copyTo(out_image);

					return true;
				}
				else
					return false;
			}
			else
				return false;
		}
		else{
			return false;
		}
	}

	bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat3b& out_image){
		tinyxml2::XMLElement *image = root->FirstChildElement(elem);

		if(image){
			const char* path = image->Attribute(_X_PATH);
			if(path){ 
				FILE * pFile = fopen (path,"r");

				if(pFile){
					fclose(pFile);

					cv::Mat aux = cv::imread(path);
					aux.copyTo(out_image);

					return true;
				}
				else
					return false;
			}
			else
				return false;
		}
		else{
			return false;
		}
	}
}