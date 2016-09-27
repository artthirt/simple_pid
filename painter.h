#ifndef PAINTER_H
#define PAINTER_H

#include <QWidget>

#include <opencv2/opencv.hpp>

namespace Ui {
class Painter;
}

class Painter : public QWidget
{
	Q_OBJECT

public:
	explicit Painter(QWidget *parent = 0);
	~Painter();

	void setMat(const cv::Mat& mat);

public slots:
	void updateMat(const cv::Mat& mat);

private:
	Ui::Painter *ui;

	cv::Mat m_mat;

	// QWidget interface
protected:
	void paintEvent(QPaintEvent *);
};

#endif // PAINTER_H
