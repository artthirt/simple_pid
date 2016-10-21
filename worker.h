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

	bool is_update_mat() const;
	void reset_update();

	void load_xml();
	void save_xml();

	/**
	 * @brief mouse_event
	 * @param pt
	 * @param state - 0 - down, 1 - up, 2 - move
	 */
	void mouse_event(const cv::Point& pt, int state);

protected:
	virtual void run();

signals:
	void starting();
	void ending();
	void newMat(const cv::Mat&);
	void resMat(const cv::Mat&);

public slots:
	void onStarting();
	void onTimeout();

private:
	bool m_start;
	bool m_done;
	tracking::TrackerPoint m_trackerPoint;
	uint m_delay;

	bool m_mouse_down;
	bool m_lock_object;
	bool m_update_mat;

	cv::Point m_mouse_pt;

	QTimer *m_timer;

	void updateMat(const cv::Mat& mat);
};

#endif // WORKER_H
