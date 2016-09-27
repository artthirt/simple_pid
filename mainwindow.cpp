#include "mainwindow.h"
#include "ui_mainwindow.h"

#define MAX_RANGE_SPEED	3.0

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	qRegisterMetaType<cv::Mat>("cv::Mat");

	m_wrk = new Worker;
	m_wrk->moveToThread(m_wrk);
	m_wrk->start();

	ui->sb_count_point->setValue(m_wrk->count());
	ui->sb_width->setValue(m_wrk->width());
	ui->sb_height->setValue(m_wrk->height());
	ui->sb_delay->setValue(m_wrk->delay());

	ui->dsb_kp->setValue(m_wrk->trackerPoint().obj().Kp);
	ui->dsb_ki->setValue(m_wrk->trackerPoint().obj().Ki);
	ui->dsb_kd->setValue(m_wrk->trackerPoint().obj().Kd);

	ui->hs_speedMax->setValue(m_wrk->trackerPoint().speedMax()/MAX_RANGE_SPEED * ui->hs_speedMax->maximum());
	ui->dsb_speedMax->setValue(m_wrk->trackerPoint().speedMax());

	connect(m_wrk, SIGNAL(newMat(cv::Mat)), ui->w_process, SLOT(updateMat(cv::Mat)), Qt::QueuedConnection);
	connect(m_wrk, SIGNAL(resMat(cv::Mat)), ui->w_res, SLOT(updateMat(cv::Mat)), Qt::QueuedConnection);

	connect(m_wrk, SIGNAL(ending()), this, SLOT(onEnding()), Qt::QueuedConnection);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(timeout()));
	m_timer.start(300);

	cv::Mat m;
	m_wrk->trackerPoint().paint(m);
	ui->w_process->setMat(m);
}

MainWindow::~MainWindow()
{
	if(m_wrk)
		delete m_wrk;

	delete ui;
}

void MainWindow::onEnding()
{
	ui->pb_start->setEnabled(true);
}

void MainWindow::timeout()
{
	ui->lb_out->setText(m_wrk->print_parameters());
}

void MainWindow::on_pb_start_clicked()
{
	ui->pb_start->setEnabled(false);

	m_wrk->setTimeout(ui->sb_delay->value());

	m_wrk->randn(ui->sb_count_point->value());
	m_wrk->setSize(ui->sb_width->value(), ui->sb_height->value());

	m_wrk->start_process();
}

void MainWindow::on_dsb_kp_valueChanged(double arg1)
{
	m_wrk->trackerPoint().obj().Kp = arg1;
}

void MainWindow::on_dsb_ki_valueChanged(double arg1)
{
	m_wrk->trackerPoint().obj().Ki = arg1;
}

void MainWindow::on_dsb_kd_valueChanged(double arg1)
{
	m_wrk->trackerPoint().obj().Kd = arg1;
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
}

void MainWindow::on_hs_speedMax_valueChanged(int value)
{
	double val = (double)MAX_RANGE_SPEED * value / ui->hs_speedMax->maximum();
	m_wrk->trackerPoint().setSpeedMax(val);
	ui->dsb_speedMax->setValue(m_wrk->trackerPoint().speedMax());
}

void MainWindow::on_pb_stop_clicked()
{
	m_wrk->trackerPoint().close();
}
