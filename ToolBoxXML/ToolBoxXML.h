#ifndef _TOOLBOX_XML
#define _TOOLBOX_XML

#include <tinyxml2.h>
#include <opencv2\opencv.hpp>
#include <ToolBox.h>

namespace ToolBoxXML{
	__declspec(dllexport) tinyxml2::XMLDocument* create_xml_doc();

	//tinyxml2::XMLElement* add_elem(tinyxml2::XMLDocument* doc, char* elem_name);

	//FOLDERS
	__declspec(dllexport)bool create_folder(std::string* name);
	__declspec(dllexport)bool create_sub_folder(std::string* main_folder, std::string* name);
	__declspec(dllexport)bool create_folders(std::string* main_folder, std::vector<std::string*>* sub_folders);
	
	//OPENCV
	__declspec(dllexport)bool cv_add_size(tinyxml2::XMLElement* elem, cv::Size *size, char* name = 0);
	__declspec(dllexport)bool cv_add_point(tinyxml2::XMLElement* elem, cv::Point *point, char* name = 0);
	__declspec(dllexport)bool cv_add_point(tinyxml2::XMLElement* elem, cv::Point3f *point, char* name = 0);
	__declspec(dllexport)bool cv_add_rect(tinyxml2::XMLElement* elem, cv::Rect *rect, char* name = 0);
	__declspec(dllexport)bool cv_add_mat_double(tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc, cv::Mat *mat);

	__declspec(dllexport)bool cv_read_size(tinyxml2::XMLElement* elem, cv::Size *size, char* name = 0);
	__declspec(dllexport)bool cv_read_point(tinyxml2::XMLElement* elem, cv::Point *point, char* name = 0);
	__declspec(dllexport)bool cv_read_point(tinyxml2::XMLElement* elem, cv::Point3f *point, char* name = 0);
	__declspec(dllexport)bool cv_read_rect(tinyxml2::XMLElement* elem, cv::Rect* rect, char* name = 0);
	__declspec(dllexport)bool cv_read_mat_double(tinyxml2::XMLElement* elem, cv::Mat& mat);

	//OPENCV IMAGE
	__declspec(dllexport)bool cv_save_image(tinyxml2::XMLElement* image_root, tinyxml2::XMLElement* image_elem, char* main_path, char* image_name, cv::Mat& image);

	__declspec(dllexport)bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat1b& out_image);
	__declspec(dllexport)bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat1f& out_image);
	__declspec(dllexport)bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat3b& out_image);
	__declspec(dllexport)bool cv_load_image_xml(tinyxml2::XMLElement* root, char* elem, cv::Mat& out_image, int flag = 1);

	//TOOLBOX
	__declspec(dllexport)bool toolbox_add_plane(tinyxml2::XMLElement* elem, ToolBox::Plane* plane, char* name = 0);
	__declspec(dllexport)bool toolbox_read_plane(tinyxml2::XMLElement* elem, ToolBox::Plane* plane, char* name = 0);
}

#endif