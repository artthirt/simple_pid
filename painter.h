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

signals:
	void mouse_event(const QPoint& pt, int state);

public slots:
	void updateMat(const cv::Mat& mat);

private:
	Ui::Painter *ui;

	cv::Mat m_mat;
	QPoint m_leftTop;
	double m_aspect_ratio;
	double m_aspect_ratio_widget;

	QPoint restoreCoord(const QPoint& pt);

	// QWidget interface
protected:
	void paintEvent(QPaintEvent *);

	// QWidget interface
protected:
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
};

#endif // PAINTER_H
