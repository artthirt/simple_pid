#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <worker.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void onEnding();

	void timeout();

	void on_pb_start_clicked();

	void on_dsb_kp_valueChanged(double arg1);

	void on_dsb_ki_valueChanged(double arg1);

	void on_dsb_kd_valueChanged(double arg1);

	void on_horizontalSlider_valueChanged(int value);

	void on_hs_speedMax_valueChanged(int value);

	void on_pb_stop_clicked();

	void on_pb_track_clicked();

	void on_dsb_kp_speed_valueChanged(double arg1);

	void on_dsb_kd_speed_valueChanged(double arg1);

	void on_dsb_dist_trigger_valueChanged(double arg1);

private:
	Ui::MainWindow *ui;

	Worker *m_wrk;

	QTimer m_timer;
};

#endif // MAINWINDOW_H
