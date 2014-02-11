#ifndef _TOOLBOX_QT
#define _TOOLBOX_QT

#include "ToolBoxQT_CVWidget.h"

#ifdef TOOLBOX_DLL_EXPORT
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllexport)
	#endif
#else
	#ifndef TOOLBOX_DLL
		#define TOOLBOX_DLL __declspec(dllimport)
	#endif
#endif

namespace ToolBoxQT{
	TOOLBOX_DLL QImage MatToQImage(const cv::Mat&);
}

#endif//_TOOLBOX_QT