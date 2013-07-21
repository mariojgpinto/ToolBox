#include "ToolBoxQT.h"

//#include <QDebug>
#include <QMouseEvent>
#include <QPainter>

#define foreach_const_it(Itname, Instance, Type) \
 for (Type::const_iterator Itname = (Instance).begin(); Itname != (Instance).end(); ++Itname)

#define foreach_idx(Idxname, Vector) \
 for (size_t Idxname = 0; Idxname < (size_t)(Vector).size(); ++Idxname)

  template <class T>
  T saturate_to_range(T value, T min_value, T max_value)
  {
    if (value < min_value) value = min_value;
    else if (value > max_value) value = max_value;
    return value;
  }

//-----------------------------------------------------------------------------
// CV WIDGET
//-----------------------------------------------------------------------------
namespace ToolBoxQT{
	CVWidget::CVWidget (QWidget* parent)
	: QWidget(parent)
	, keep_ratio(false)
	, m_last_mouse_pos(-1,-1)
	{
	}

	void CVWidget :: mouseMoveEvent ( QMouseEvent * event )
	{
		if (m_image.isNull())
		{
			event->ignore();
			return;
		}

		int x = event->x() * (m_image.width() / float(width()));
		int y = event->y() * (m_image.height() / float(height()));
		m_last_mouse_pos = QPoint(x,y);
		//TODO: Lanch Event
		//emit mouseMoved(x,y);
	}

	void CVWidget :: setImage(const QImage& im)
	{
		m_image = im;
		update();
	}

	void CVWidget :: setRects(const std::list<cv::Rect>& rects, const cv::Vec3b& color)
	{
		m_rects = rects;
		m_rect_color = color;
		update();
	}

	void CVWidget :: setTexts(const std::vector<TextData> texts)
	{
		m_texts = texts;
		update();
	}

	bool
	CVWidget :: setImage(QImage& image, cv::Mat1b *im)
	{
		bool geometryUpdated = false;

		if (image.width() != im->cols
				|| image.height() != im->rows)
		{
			image = QImage(im->cols, im->rows, QImage::Format_RGB32);
			geometryUpdated = true;
		}

		for (int r = 0; r < im->rows; ++r)
		{
			QRgb* ptr = (QRgb*) image.scanLine(r);
			const uchar* cv_ptr = im->ptr(r);
			for (int i = 0; i < im->cols; ++i)
			{
				float v = *cv_ptr;
				*ptr = qRgb(v,v,v);
				++ptr;
				++cv_ptr;
			}
		}

		//for (int r = 0; r < im->rows; ++r)
		//{
		//	QRgb* ptr = (QRgb*) image.scanLine(r);
		//	for (int c = 0; c < im->cols; ++c)
		//	{
		//		int v = (*im)(r,c);
		//		*ptr = qRgb(v,v,v);
		//		++ptr;
		//	}
		//}

		return geometryUpdated;
	}

	bool
	CVWidget :: setImage(QImage& image, cv::Mat1f *im, double* i_min_val, double* i_max_val)
	{
		bool geometryUpdated = false;

		if (image.width() != im->cols
				|| image.height() != im->rows)
		{
			image = QImage(im->cols, im->rows, QImage::Format_RGB32);
			geometryUpdated = true;
		}

		//double min_val, max_val;
		//if (i_min_val && i_max_val)
		//{
		//	min_val = *i_min_val;
		//	max_val = *i_max_val;
		//}
		//else
		//	minMaxLoc(*im, &min_val, &max_val);
		//if (min_val == max_val)
		//{
		//	image.fill(qRgb(0,0,0));
		//	return geometryUpdated;
		//}

		//for (int r = 0; r < im->rows; ++r)
		//{
		//	QRgb* ptr = (QRgb*) image.scanLine(r);
		//	const float* cv_ptr = im->ptr<float>(r);
		//	for (int c = 0; c < im->cols; ++c)
		//	{
		//		int v = 255*(*cv_ptr-min_val)/(max_val-min_val);
		//		v = saturate_to_range(v, 0, 255);
		//		int rgb = (0xff << 24) + (v << 16) + (v << 8) + v;
		//		*ptr = rgb;
		//		++ptr;
		//		++cv_ptr;
		//	}
		//}
		
		for (int r = 0; r < im->rows; ++r)
		{
			//const uchar* cv_ptr = im->ptr(r);s
			QRgb* ptr = (QRgb*) image.scanLine(r);
			const float* cv_ptr = im->ptr<float>(r);
			for (int i = 0; i < im->cols; ++i)
			{
				float v = *cv_ptr;
				*ptr = qRgb(v,v,v);
				++ptr;
				++cv_ptr;
			}
		}

		return geometryUpdated;
	}

	bool
	CVWidget :: setImage(QImage& image, cv::Mat3b *im)
	{
		bool geometryUpdated = false;

		if (image.isNull()
				|| image.width() != im->cols
				|| image.height() != im->rows)
		{
			image = QImage(im->cols, im->rows, QImage::Format_RGB32);
			geometryUpdated = true;
		}

		for (int r = 0; r < im->rows; ++r)
		{
			QRgb* ptr = (QRgb*) image.scanLine(r);
			const uchar* cv_ptr = im->ptr(r);
			for (int i = 0; i < im->cols; ++i)
			{
				int rgb = 0xff << 24;
				rgb |= (*cv_ptr++);
				rgb |= ((*cv_ptr++) << 8);
				rgb |= ((*cv_ptr++) << 16);
				*ptr++ = rgb;
			}
		}

		return geometryUpdated;
	}

	void CVWidget :: setImage(cv::Mat *im){
		if(im->type() == CV_8UC3){
			setImage((cv::Mat3b*)im);
		} else
		if(im->type() == CV_8UC1){
			setImage((cv::Mat1b*)im);
		} else
		if(im->type() == CV_32FC1){
			setImage((cv::Mat1f*)im);
		}
	}

	void CVWidget :: setImage(cv::Mat1b *im)
	{
		if (setImage(m_image, im))
			updateGeometry();
    
		update();
	}

	void CVWidget :: setImage(cv::Mat1f *im, double* i_min_val, double* i_max_val)
	{
		if (setImage(m_image, im, i_min_val, i_max_val))
			updateGeometry();

		update();
	}

	void CVWidget :: setImage(cv::Mat3b *im)
	{
		if (setImage(m_image, im))
			updateGeometry();

		update();
	}

	double CVWidget :: scaleX() const
	{
		return double(rect().width())/m_image.rect().width();
	}

	double CVWidget :: scaleY() const
	{
		return double(rect().height())/m_image.rect().height();
	}

	void CVWidget :: setPen(QPen pen)
	{
		m_pen = pen;
	}

	void CVWidget :: paintEvent(QPaintEvent * event)
	{
		double sx = scaleX();
		double sy = scaleY();

		if (m_pen.data_ptr() == NULL){
			QPen p;
			m_pen = p;
			m_pen.setColor(Qt::white);
			m_pen.setWidth(2);
		}

		QPainter painter(this);

		// FIXME: The image rect should be cached and recomputed on resize events.

		painter.drawImage(imageRect(), m_image, m_image.rect());

		m_pen.setColor(qRgb(m_rect_color[0], m_rect_color[1], m_rect_color[2]));
		painter.setPen(m_pen);
		foreach_const_it(it, m_rects, std::list<cv::Rect>)
		{
			const cv::Rect& r = *it;
			QRect qr (r.x*sx, r.y*sy, r.width*sx, r.height*sy);
			painter.drawRect(qr);
		}
		int i = 0;
		foreach_idx(i, m_texts)
		{
			const cv::Vec3b& c = m_texts[i].color;
			m_pen.setColor(qRgb(c[0], c[1], c[2]));
			painter.setFont(QFont("Helvetica", 14));
			painter.setPen(m_pen);
			QString s (m_texts[i].text.c_str());
			QPoint p (m_texts[i].x*sx, m_texts[i].y*sy);
			painter.drawText(p, s);
		}
	}

	void
	CVWidget::setRatioKeeping (bool ratio_keeping)
	{
		if (keep_ratio == ratio_keeping)
			return;

		keep_ratio = ratio_keeping;

		if (keep_ratio)
		{
			QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
			sizePolicy.setHeightForWidth(true);
			setSizePolicy(sizePolicy);
		}
		else
		{
			QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
			sizePolicy.setHeightForWidth(false);
			setSizePolicy(sizePolicy);
		}

		updateGeometry();
	}

	QSize CVWidget :: sizeHint() const
	{
		if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
			return QWidget::sizeHint();

		return QSize(m_image.width(), m_image.height());
	}

	QSize CVWidget :: minimumSizeHint () const
	{
		if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
			return QWidget::minimumSizeHint();

		return QSize(m_image.width() / 8, m_image.height() / 8);
	}

	int CVWidget :: heightForWidth(int width) const
	{
		if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
			return QWidget::heightForWidth(width);

		return m_image.height() / m_image.width() * width;
	}

	QRect CVWidget :: imageRect () const
	{
		if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
			return rect();

		const QRect widgetRect = rect();

		const float widgetRatio = float(widgetRect.height()) / float(widgetRect.width());
		const float imageRatio  = float(m_image   .height()) / float(m_image   .width());

		QRect ret = widgetRect;

		if (imageRatio < widgetRatio)
		{
			const int extraHeight = widgetRect.height() - int(widgetRect.width() * imageRatio);

			ret.adjust(0, extraHeight / 2, 0, -extraHeight / 2);
		}
		else if (widgetRatio < imageRatio)
		{
			const int extraWidth = widgetRect.width() - int(widgetRect.height() / imageRatio);

			ret.adjust(extraWidth / 2, 0, -extraWidth / 2, 0);
		}

		return ret;
	}
}








namespace ToolBoxQT{
	QImage MatToQImage(const cv::Mat& mat)
	{
		// 8-bits unsigned, NO. OF CHANNELS=1
		if(mat.type()==CV_8UC1)
		{
			// Set the color table (used to translate colour indexes to qRgb values)
			QVector<QRgb> colorTable;
			for (int i=0; i<256; i++)
				colorTable.push_back(qRgb(i,i,i));
			// Copy input Mat
			const uchar *qImageBuffer = (const uchar*)mat.data;
			// Create QImage with same dimensions as input Mat
			QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
			img.setColorTable(colorTable);
			return img;
		}
		// 8-bits unsigned, NO. OF CHANNELS=3
		else if(mat.type()==CV_8UC3)
		{
			// Copy input Mat
			const uchar *qImageBuffer = (const uchar*)mat.data;
			// Create QImage with same dimensions as input Mat
			QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
			return img.rgbSwapped();
		}
		else
		{
			//qDebug() << "ERROR: Mat could not be converted to QImage.";
			return QImage();
		}
	}

}
