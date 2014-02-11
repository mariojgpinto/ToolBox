#ifndef _TOOLBOX_QT_CV_WIDGET
#define _TOOLBOX_QT_CV_WIDGET

#include <opencv2\opencv.hpp>

#include <QPoint>
#include <QImage>
#include <QWidget>
#include <QPen>

class QMouseEvent;
class QPaintEvent;
class QPen;

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
	class TOOLBOX_DLL CVWidget :public QWidget{
		 //Q_OBJECT

		public:
			struct TextData
			{
				std::string text;
				int x, y;
				float size;
				cv::Vec3b color;
			};

		public: // FIXME: Move these where they really belong.
			static bool setImage(QImage& image, cv::Mat1f *im, double* min_val = 0, double* max_val = 0);
			static bool setImage(QImage& image, cv::Mat1b *im);
			static bool setImage(QImage& image, cv::Mat3b *im);

		public:
			CVWidget(QWidget* parent);

		public:
			double scaleX() const;
			double scaleY() const;

			void getLastMousePos(int& x, int& y)
			{ x = m_last_mouse_pos.x(); y = m_last_mouse_pos.y(); }

			void setImage(const QImage& im);
			void setImage(cv::Mat *im);
			void setImage(cv::Mat1f *im, double* min_val = 0, double* max_val = 0);
			void setImage(cv::Mat1b *im);
			void setImage(cv::Mat3b *im);

			void setPen(QPen q);
			void setRects(const std::list<cv::Rect>& rects, const cv::Vec3b& color = cv::Vec3b(0,0,0));
			void setTexts(const std::vector<TextData> texts);

		signals:
			void mouseMoved(int image_x, int image_y);

		protected:
			virtual void mouseMoveEvent(QMouseEvent* event);
			virtual void paintEvent(QPaintEvent* event);
			virtual QSize        sizeHint() const;
			virtual QSize minimumSizeHint() const;
			virtual int heightForWidth(int width) const;

		public:
			void setRatioKeeping (bool ratio_keeping = true);

		private:
			QRect imageRect () const;

		private:
			bool keep_ratio;

		private:
			QPoint m_last_mouse_pos;
			QImage m_image;
			QPen m_pen;
			std::list<cv::Rect> m_rects;
			cv::Vec3b m_rect_color;
			std::vector<TextData> m_texts;
	};
}

#endif//_TOOLBOX_QT_CV_WIDGET