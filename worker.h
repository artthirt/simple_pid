#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QThread>
#include <QTimer>

#include <tracking.h>

class Worker : public QThread
{
	Q_OBJECT
public:
	Worker(QObject* parent = 0);
	~Worker();

	tracking::TrackerPoint &trackerPoint();

	uint delay() const;
	void setTimeout(uint delay);

	void randn(uint cnt);

	int width() const;
	int height() const;

	void setSize(int w, int h);

	size_t count() const;

	void start_process();

	QString print_parameters() const;

protected:
	virtual void run();

signals:
	void starting();
	void ending();
	void newMat(const cv::Mat&);
	void resMat(const cv::Mat&);

public slots:
	void onStarting();

private:
	bool m_start;
	bool m_done;
	tracking::TrackerPoint m_trackerPoint;
	uint m_delay;

	QTimer *m_timer;

	void updateMat(const cv::Mat& mat);
};

#endif // WORKER_H
