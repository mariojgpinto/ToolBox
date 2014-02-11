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

	/**
	 * @brief	Adds a cv::Rect to the XML element.
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
	bool cv_add_rect(tinyxml2::XMLElement* elem, cv::Rect *rect, char* name){
		if(!elem || !rect) return false;

		char rect_x[256]; 
		char rect_y[256]; 
		char rect_w[256]; 
		char rect_h[256]; 

		if(name){
			strcpy(rect_x,name); strcat(rect_x,"_");
			strcpy(rect_y,name); strcat(rect_y,"_");
			strcpy(rect_w,name); strcat(rect_w,"_");
			strcpy(rect_h,name); strcat(rect_h,"_");
			strcat(rect_x,_X_CV_RECT_X);
			strcat(rect_y,_X_CV_RECT_Y);
			strcat(rect_w,_X_CV_RECT_W);
			strcat(rect_h,_X_CV_RECT_H);
		}
		else{
			strcat(rect_x,_X_CV_RECT_X);
			strcat(rect_y,_X_CV_RECT_Y);
			strcat(rect_w,_X_CV_RECT_W);
			strcat(rect_h,_X_CV_RECT_H);
		}

		elem->SetAttribute(rect_x, rect->x);
		elem->SetAttribute(rect_y, rect->y);
		elem->SetAttribute(rect_w, rect->width);
		elem->SetAttribute(rect_h, rect->height);

		return true;
	}

	bool cv_add_mat_double(tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc, cv::Mat *mat){
		if(!elem || !mat) return false;

		if(!mat->cols || !mat->rows) return false;

		char mat_str[256];
		strcpy_s(mat_str,_X_CV_MAT);

		elem->SetAttribute(_X_CV_MAT_ROWS,mat->rows);
		elem->SetAttribute(_X_CV_MAT_COLS,mat->cols);

		//_matrix.ptr<double>(0)[0] = vec_1.val[0];
		for(int row = 0 ; row < mat->rows ; row++){
			char elem_row_str[8];
			sprintf_s(elem_row_str,"%s%d",_X_CV_MAT_ATT_ROWS,row);
			tinyxml2::XMLElement* elem_row = doc->NewElement(elem_row_str);

			for(int col = 0 ; col < mat->cols ; col++){
				char att_col_str[8];
				sprintf_s(att_col_str,"%s%d",_X_CV_MAT_ATT_COLS,col);

				elem_row->SetAttribute(att_col_str,mat->ptr<double>(row)[col]);
			}

			elem->LinkEndChild(elem_row);
		}
		
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

	bool cv_read_rect(tinyxml2::XMLElement* elem, cv::Rect* rect, char* name){
		if(!elem || !rect) return false;

		int error = 0; 
		double x,y,w,h;

		char rect_x[256]; 
		char rect_y[256]; 
		char rect_w[256]; 
		char rect_h[256];

		if(name){
			strcpy(rect_x,name); strcat(rect_x,"_");
			strcpy(rect_y,name); strcat(rect_y,"_");
			strcpy(rect_w,name); strcat(rect_w,"_");
			strcpy(rect_h,name); strcat(rect_h,"_");
			strcat(rect_x,_X_CV_RECT_X);
			strcat(rect_y,_X_CV_RECT_Y);
			strcat(rect_w,_X_CV_RECT_W);
			strcat(rect_h,_X_CV_RECT_H);
		}
		else{
			strcat(rect_x,_X_CV_RECT_X);
			strcat(rect_y,_X_CV_RECT_Y);
			strcat(rect_w,_X_CV_RECT_W);
			strcat(rect_h,_X_CV_RECT_H);
		}

		error += elem->QueryDoubleAttribute(rect_x,&x);
		error += elem->QueryDoubleAttribute(rect_y,&y);
		error += elem->QueryDoubleAttribute(rect_w,&w);
		error += elem->QueryDoubleAttribute(rect_h,&h);

		if(error) return false;

		//rect = new cv::Rect((float)x,(float)y,(float)y,(float)h);
		rect->x = x;
		rect->y = y;
		rect->width = w;
		rect->height = h;

		return true;
	}

	bool cv_read_mat_double(tinyxml2::XMLElement* elem, cv::Mat& mat){
		if(!elem) return false;

		int error = 0; 
		int rows,cols;

		char mat_str[256];
		strcpy_s(mat_str,_X_CV_MAT);

		error += elem->QueryIntAttribute(_X_CV_MAT_ROWS,&rows);
		error += elem->QueryIntAttribute(_X_CV_MAT_COLS,&cols);

		if(error) return false;

		mat.create(rows,cols,CV_64FC1);
		if(!mat.data) return false;

		for(int r = 0 ; r < rows ; r++){
			char elem_row_str[8];
			sprintf_s(elem_row_str,"%s%d",_X_CV_MAT_ATT_ROWS,r);
			
			tinyxml2::XMLElement* row_elem = elem->FirstChildElement(elem_row_str);

			if(row_elem){
				for(int c = 0 ; c < cols ; c++){
					char att_col_str[8];
					sprintf_s(att_col_str,"%s%d",_X_CV_MAT_ATT_COLS,c);
					double value;
					error = 0;

					error = row_elem->QueryDoubleAttribute(att_col_str,&value);

					mat.ptr<double>(r)[c] = value;

					if(error)
						return false;
				}
			}
			else
				return false;
		}
		
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

					out_image = cv::Mat(cvLoadImage(path,0));//cv::imread(buff,flag);

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

					out_image = cv::Mat(cvLoadImage(path,0));//cv::imread(buff,flag);;

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

					out_image = cv::Mat(cvLoadImage(path));//cv::imread(buff,flag);

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

	bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat& out_image, int flag){
		tinyxml2::XMLElement *image = root->FirstChildElement(elem);

		if(image){
			const char* path = image->Attribute(_X_PATH);
			if(path){ 
				
				FILE * pFile = fopen (path,"r");
				
				if(pFile){
					fclose(pFile);
					
					out_image = cv::Mat(cvLoadImage(path,flag));//cv::imread(buff,flag);

					if(out_image.rows && out_image.cols)
						return true;
					else
						return false;
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

	//-----------------------------------------------------------------------------
	// TOOLBOX
	//-----------------------------------------------------------------------------
	/**
	 * @brief	Adds a ToolBox::Plane to the XML element.
	 * @details	.
	 *
	 * @param	elem
	 *			XML element where the attribute will be added.
	 *
	 * @param	plane
	 *			Attribute to be added.
	 *
	 * @param	name
	 *			Name of the attribute to be added (not necessary).
	 */
	bool toolbox_add_plane(tinyxml2::XMLElement* elem, ToolBox::Plane* plane, char* name){
		if(!elem || !plane) return false;

		char plane_a[256]; 
		char plane_b[256]; 
		char plane_c[256]; 
		char plane_d[256]; 

		if(name){
			strcpy(plane_a,name); strcat(plane_a,"_");
			strcpy(plane_b,name); strcat(plane_b,"_");
			strcpy(plane_c,name); strcat(plane_c,"_");
			strcpy(plane_d,name); strcat(plane_d,"_");
			strcat(plane_a,_X_TOOLBOX_PLANE_A);
			strcat(plane_b,_X_TOOLBOX_PLANE_B);
			strcat(plane_c,_X_TOOLBOX_PLANE_C);
			strcat(plane_d,_X_TOOLBOX_PLANE_D);
		}
		else{
			strcat(plane_a,_X_CV_RECT_X);
			strcat(plane_b,_X_CV_RECT_Y);
			strcat(plane_c,_X_CV_RECT_W);
			strcat(plane_d,_X_CV_RECT_H);
		}

		elem->SetAttribute(plane_a, plane->_a);
		elem->SetAttribute(plane_b, plane->_b);
		elem->SetAttribute(plane_c, plane->_c);
		elem->SetAttribute(plane_d, plane->_d);

		return true;
	}

	bool toolbox_read_plane(tinyxml2::XMLElement* elem, ToolBox::Plane* plane, char* name){
		if(!elem || !plane) return false;

		int error = 0; 
		double a,b,c,d;

		char plane_a[256]; 
		char plane_b[256]; 
		char plane_c[256]; 
		char plane_d[256]; 

		if(name){
			strcpy(plane_a,name); strcat(plane_a,"_");
			strcpy(plane_b,name); strcat(plane_b,"_");
			strcpy(plane_c,name); strcat(plane_c,"_");
			strcpy(plane_d,name); strcat(plane_d,"_");
			strcat(plane_a,_X_TOOLBOX_PLANE_A);
			strcat(plane_b,_X_TOOLBOX_PLANE_B);
			strcat(plane_c,_X_TOOLBOX_PLANE_C);
			strcat(plane_d,_X_TOOLBOX_PLANE_D);
		}
		else{
			strcat(plane_a,_X_CV_RECT_X);
			strcat(plane_b,_X_CV_RECT_Y);
			strcat(plane_c,_X_CV_RECT_W);
			strcat(plane_d,_X_CV_RECT_H);
		}

		error += elem->QueryDoubleAttribute(plane_a,&a);
		error += elem->QueryDoubleAttribute(plane_b,&b);
		error += elem->QueryDoubleAttribute(plane_c,&c);
		error += elem->QueryDoubleAttribute(plane_d,&d);

		if(error) return false;

		//plane = new ToolBox::Plane();
		plane->_a = a;
		plane->_b = b;
		plane->_c = c;
		plane->_d = d;

		return true;
	}
}