#include "../ToolBox/ToolBox.h"
#include "../ToolBoxCV/ToolBoxCV.h"
#include "../ToolBoxPCL/ToolBoxPCL.h"


int main(int argc, char* argv[]){

	std::vector<cv::Point3f*> vec;
	vec.push_back(new cv::Point3f(0,0,0));

	int n = 10;

	for(float i = 0 ; i < n ; i+=0.1){		
		vec.push_back(new cv::Point3f(0,0,i));
		vec.push_back(new cv::Point3f(0,i,0));
		vec.push_back(new cv::Point3f(0,i,i));
		vec.push_back(new cv::Point3f(i,0,0));
		vec.push_back(new cv::Point3f(i,0,i));
		vec.push_back(new cv::Point3f(i,i,0));
		vec.push_back(new cv::Point3f(i,i,i));

		vec.push_back(new cv::Point3f(n,n,i));
		vec.push_back(new cv::Point3f(n,i,n));
		vec.push_back(new cv::Point3f(n,i,i));
		vec.push_back(new cv::Point3f(i,n,n));
		vec.push_back(new cv::Point3f(i,n,i));
		vec.push_back(new cv::Point3f(i,i,n));
		vec.push_back(new cv::Point3f(i,i,i));
	}

	ToolBoxPCL::write_to_ply_file(&vec,"object");

	printf("End\n");

	return 0;
}