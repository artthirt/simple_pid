#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/ml.hpp>
#include <ctime>
#include <algorithm>
#include <memory>

#include <doublefann.h>
#include <fann_cpp.h>

#include "tracking.h"

#include "mainwindow.h"

#include <QApplication>

using namespace std;
using namespace cv;


void onMouse(int event, int x, int y, int flags, void* user_data)
{
	tracking::TrackerPoint *tp = (tracking::TrackerPoint*)user_data;

	if(event == EVENT_MOUSEMOVE){
	}
	if(event == EVENT_LBUTTONDOWN){
		tp->set_pos(x, y);
	}
}

void show_mat(const std::string& nm ,const vector< double >& X, size_t cnt = 1000, int w = 800, int h = 600)
{
	double m1, m2;
	minMaxLoc(X, &m1, &m2);

	double d = m2 - m1;

	Mat mat = Mat::zeros(h, w, CV_8UC3);

	size_t mc = min(X.size(), cnt);

	for(size_t i = 0; i < mc - 1; i++){
		double t1 = (double)i / mc;
		double t2 = (double)(i + 1) / mc;

		Point p1(t1 * w, h/2 + X[i] / d * h/1.5);
		Point p2(t2 * w, h/2 + X[i + 1] / d * h/1.5);

		line(mat, p1, p2, Scalar(0, 20, 255));
	}

	imshow(nm, mat);

	imwrite(nm + ".bmp", mat);
}

void show_mats(const std::string& nm ,const vector< double >& X, size_t offset, const vector< double >& y, size_t cnt = 1000, int w = 800, int h = 600)
{
	double m1, m2;
	minMaxLoc(X, &m1, &m2);

	double d =	abs(m2 - m1);

	Mat mat = Mat::zeros(h, w, CV_8UC3);

	size_t mc = min(X.size() - offset, cnt);
	if(offset + mc > X.size())
		mc = X.size() - offset;

	for(size_t i = offset; i < offset + mc - 1; i++){
		double t1 = (double)(i - offset) / mc;
		double t2 = (double)(i + 1 - offset) / mc;

		Point p1(t1 * w, h/2 + X[i] / d * h/1.5);
		Point p2(t2 * w, h/2 + X[i + 1] / d * h/1.5);

		line(mat, p1, p2, Scalar(0, 20, 255));
	}

	minMaxLoc(y, &m1, &m2);
	Scalar s = mean(y);

	d = abs(m2 - m1);

	mc = min(y.size(), cnt);
	for(size_t i = 0; i < mc - 1; i++){
		double t1 = (double)i / mc;
		double t2 = (double)(i + 1) / mc;

		Point p1(t1 * w, h/2 + (y[i] - s[0]) / d * h/1.5);
		Point p2(t2 * w, h/2 + (y[i + 1] - s[0]) / d * h/1.5);

		line(mat, p1, p2, Scalar(0, 255, 20));
	}

	imshow(nm, mat);

	imwrite(nm + ".bmp", mat);
}

typedef unsigned int uint;

void _ml()
{
	cout << "begin nn...\n";

	uint depth = 20;
	uint num_layers = 5;
	uint layers[] = {depth, 2 * depth, 2 * depth, 2 * depth, 1};

	const size_t cnt = 2000;
	const double freq = 20;

	FANN::neural_net nn;
	nn.create_standard_array(num_layers, layers);

	vector < fann_type > X, y, Xm;

	for(uint i = 0; i < cnt + 1; i++){
		double d, val;
		d = (double)i/cnt;
		val = sin(d * 2 * CV_PI * freq);
		X.push_back(val);
	}

	for(size_t i = 0, k = 0; i < cnt; i++){
		for(size_t j = 0; j < depth; j++, k++){
			if(i >= depth)
				Xm.push_back(X[i - j]);
			else
				Xm.push_back(0);
		}
		y.push_back(X[i + 1]);
	}
	y[0] = 0;

	nn.train(Xm.data(), y.data());

	cout << "learning rate: " << nn.get_learning_rate() << endl;

	cout << "MSE: " << nn.get_MSE() << endl;

	nn.print_parameters();

	uint indexes[] = {101, 203, 40, 708, 512, 1033, 756, 1354};
	uint cnt_i = sizeof(indexes) / sizeof(*indexes);

	vector < fann_type > Xtest, y_test;
	for(uint i = 0; i < cnt_i; i++){
		for(uint j = 0; j < depth; j++){
			Xtest.push_back(X[indexes[i] - j]);
		}
		y_test.push_back(X[indexes[i + 1]]);
	}
//	nn.test(Xtest.data(), y_test.data());

	int index = 2;

	fann_type *res = nn.run(Xtest.data() + indexes[index] * depth);

	cout << "pass: " << y_test[index] << " versus " << *res << endl;
	cout << "test score: " << nn.get_MSE() << endl;

	cout << "end nn\n";
}

void uses_cv()
{
	cout << "Hello World!" << endl;

//	_ml();

	tracking::TrackerPoint tp;

	namedWindow("mat");
	setMouseCallback("mat", onMouse, &tp);

	tp.randn();

	while(1){
		char key = waitKey(20);

		Mat mat = Mat::zeros(c_height, c_width, CV_8UC3);

		tp.paint(mat);
		imshow("mat", mat);

		if(key == 27)
			break;

		if(key == 'c')
			tp.clear();
		if(key == 'e'){
			tp.calc();
		}
	}
}

////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	MainWindow w;
	w.show();

	app.exec();

	return 0;
}
