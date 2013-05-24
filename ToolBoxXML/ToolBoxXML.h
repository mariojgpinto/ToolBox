#ifndef _TOOLBOX_XML
#define _TOOLBOX_XML

#include <tinyxml2.h>
#include <opencv2\opencv.hpp>

namespace ToolBoxXML{
	tinyxml2::XMLDocument* create_xml_doc();

	//tinyxml2::XMLElement* add_elem(tinyxml2::XMLDocument* doc, char* elem_name);

	//FOLDERS
	bool create_folder(std::string* name);
	bool create_sub_folder(std::string* main_folder, std::string* name);
	bool create_folders(std::string* main_folder, std::vector<std::string*>* sub_folders);
	
	//OPENCV
	bool cv_add_size(tinyxml2::XMLElement* elem, cv::Size *size, char* name = 0);
	bool cv_add_point(tinyxml2::XMLElement* elem, cv::Point *point, char* name = 0);
	bool cv_add_point(tinyxml2::XMLElement* elem, cv::Point3f *point, char* name = 0);
	bool cv_add_mat_double(tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc, cv::Mat *mat);

	bool cv_read_size(tinyxml2::XMLElement* elem, cv::Size *size, char* name = 0);
	bool cv_read_point(tinyxml2::XMLElement* elem, cv::Point *point, char* name = 0);
	bool cv_read_point(tinyxml2::XMLElement* elem, cv::Point3f *point, char* name = 0);
	bool cv_read_mat_double(tinyxml2::XMLElement* elem, cv::Mat& mat);

	//OPENCV IMAGE
	bool cv_save_image(tinyxml2::XMLElement* image_root, tinyxml2::XMLElement* image_elem, char* main_path, char* image_name, cv::Mat& image);

	bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat1b& out_image);
	bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat1f& out_image);
	bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat3b& out_image);

}

#endif