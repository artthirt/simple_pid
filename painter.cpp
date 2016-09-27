#include "painter.h"
#include "ui_painter.h"

#include <QPaintEvent>
#include <QImage>
#include <QDebug>
#include <QPainter>

using namespace cv;

QImage Mat2QImage(cv::Mat const& src)
{
	if(src.empty())
		return QImage();
	cv::Mat temp; // make the same cv::Mat

	switch (src.type()) {
		case CV_8UC3:
			cvtColor(src, temp, cv::COLOR_BGR2RGB); // cvtColor Makes a copt, that what i need
			break;
		case CV_8UC4:
			cvtColor(src, temp, cv::COLOR_BGRA2RGB); // cvtColor Makes a copt, that what i need
			break;
		case CV_8UC1:
			cvtColor(src, temp, cv::COLOR_GRAY2RGB); // cvtColor Makes a copt, that what i need
			break;
		case CV_16UC1:
		{
			temp = src;
			temp /= 256;
			temp.convertTo(temp, CV_8U);
			cvtColor(temp, temp, cv::COLOR_GRAY2RGB); // cvtColor Makes a copt, that what i need
			break;
		}
		default:
			qDebug() << "not realized. type:" << src.type() << "; channels:" << src.channels();
			break;
	}

	QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
	QImage dest2(dest);
	dest2.detach(); // enforce deep copy
	return dest2;
}

Painter::Painter(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::Painter)
{
	ui->setupUi(this);
}

Painter::~Painter()
{
	delete ui;
}

void Painter::setMat(const Mat &mat)
{
	m_mat = mat;
	update();
}

void Painter::updateMat(const Mat &mat)
{
	setMat(mat);
}

void Painter::paintEvent(QPaintEvent *)
{
	QPainter painter(this);

	QRect rt = rect();

	painter.fillRect(rt, Qt::black);

	Mat img = m_mat;

	if(img.empty())
		return;

	Mat out;
	QImage tmp;

	double aspect_ratio_widget = (double)rt.width()/rt.height();
	double aspect_ratio = (double)img.cols/img.rows;

	if(aspect_ratio_widget > aspect_ratio){
		cv::resize(img, out, Size(rt.height() * aspect_ratio, rt.height()));
	}else{
		cv::resize(img, out, Size(rt.width(), rt.width()/aspect_ratio));
	}

	tmp = Mat2QImage(out);

	painter.drawImage(QPoint(rt.width()/2 - tmp.width()/2, rt.height()/2 - tmp.height()/2), tmp);

}
