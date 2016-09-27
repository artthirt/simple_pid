#include "worker.h"

#include <QTime>

#include <QObject>
#include <QMetaType>

using namespace cv;

Worker::Worker(QObject* parent)
	: QThread(parent)
{
	m_delay = 100;
	m_done = false;
	m_start = false;
	m_trackerPoint.func = std::function< void(const cv::Mat&) >(std::bind(&Worker::updateMat, this, std::placeholders::_1));

	load_xml();
}

Worker::~Worker()
{
	save_xml();

	m_trackerPoint.close();
	m_done = true;
	quit();
	wait();
}

tracking::TrackerPoint &Worker::trackerPoint()
{
	return m_trackerPoint;
}

uint Worker::delay() const
{
	return m_delay;
}

void Worker::setTimeout(uint delay)
{
	m_delay = delay;
}

void Worker::randn(uint cnt)
{
	m_trackerPoint.randn(cnt);
}

int Worker::width() const
{
	return m_trackerPoint.width();
}

int Worker::height() const
{
	return m_trackerPoint.height();
}

void Worker::setSize(int w, int h)
{
	m_trackerPoint.set_size(w, h);
}

size_t Worker::count() const
{
	return m_trackerPoint.count();
}

void Worker::start_process()
{
	if(m_start)
		return;

	m_start = true;
	emit starting();
}

QString Worker::print_parameters() const
{
	return QString(m_trackerPoint.print_parameters().c_str());
}

const std::string xml_config("config.worker.xml");

void Worker::load_xml()
{
	FileStorage fs(xml_config, FileStorage::READ);

	if(!fs.isOpened())
		return;

	m_delay = (int)fs["delay"];
}

void Worker::save_xml()
{
	FileStorage fs(xml_config, FileStorage::WRITE);

	fs << "delay" << (int)m_delay;
}

void Worker::run()
{
	connect(this, SIGNAL(starting()), SLOT(onStarting()), Qt::QueuedConnection);

	exec();
}

void Worker::onStarting()
{
	m_trackerPoint.calc();

	m_start = false;

	cv::Mat res;
	m_trackerPoint.paint(res);
	emit resMat(res);

	emit ending();
}

void Worker::updateMat(const cv::Mat &mat)
{
	QTime time;
	time.start();

	emit newMat(mat);

	while(time.elapsed() < m_delay && !m_done){
		usleep(30);
	}
}
