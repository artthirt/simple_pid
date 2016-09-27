#include "tracking.h"

#include <ctime>
#include <algorithm>
#include <memory>
#include <functional>
#include <sstream>

#include <opencv2/opencv.hpp>

using namespace tracking;
using namespace cv;
using namespace std;

const size_t max_points = 20000;

const int c_width = 2048;
const int c_height = 2048;

const float place_length = 50;
const float c_place_coeff = c_height;

const int c_randn = 15;

/////////////////////////////////

inline double R2A(double val)
{
	return val * 180. / CV_PI;
}

/////////////////////////////////

double Obj::get_theta() const
{
	return theta;
}

void Obj::update_values()
{
//	theta += w;

//	theta = atan2(sin(theta), cos(theta));
//	cout << "theta=" << theta * 180. / CV_PI << endl;
}

Vec2f Obj::ensure_v()
{
	w = std::min(max_w, w);
	v = std::min(max_v, v);

	w = std::max(-max_w, w);
	v = std::max(-max_v, v);

	theta += w;
	theta = atan2(sin(theta), cos(theta));
	cout << "theta=" << theta * 180. / CV_PI << endl;

	cv::Vec2f res = speed();

	return res;
}

Vec2f Obj::speed() const
{
		return cv::Vec2f(
					v * cos(theta),
					v * sin(theta)
					);
}

void Obj::draw_obj(Mat &mat, float place_coeff)
{
	if(mat.empty())
		return;

	vector< Point2f > in_v, out_v;
	vector< Point > out_vi;
	in_v.push_back(Point2f(-30, 27));
	in_v.push_back(Point2f(30, 0));
	in_v.push_back(Point2f(-30, -27));

	Mat m = cv::getRotationMatrix2D(Point2f(), -theta * 180  /CV_PI, 1);
	cv::transform(in_v, out_v, m);

	Vec2f cv(mat.cols/2, mat.rows/2);

	for(size_t i = 0; i < out_v.size(); i++){
		Vec2f offset = pos / place_length * place_coeff + cv;

		out_v[i] += Point2f(offset[0], offset[1]);
	}

	Mat(out_v).convertTo(out_vi, CV_32S);

	fillConvexPoly(mat, out_vi, Scalar(110, 130, 150));
	polylines(mat, out_vi, true, Scalar(20, 50, 100), 1, 4);
}

/////////////////////////////////

void crop(Vec2f& val, float max_val)
{
	float len = norm(val);
	if(len > max_val){
		Vec2f nval = normalize(val);
		val = max_val * nval;
	}
}

/////////////////////////////////

TrackerPoint::TrackerPoint()
{
	m_done = false;
	m_delta = 0;
	m_current_id = 0;

	m_width = c_width;
	m_height = c_height;
	m_place_coeff = c_place_coeff;

	m_speed_max = 0.5;
	m_epsilon = 0.3;

	m_koeff_resist = 0.98;

	init();

	load_xml();
}

TrackerPoint::~TrackerPoint()
{
	save_xml();
}

void TrackerPoint::set_pos(int x, int y)
{
	while(m_track.size() > max_points)
		m_track.erase(m_track.begin());
	while(m_speed.size() > max_points)
		m_speed.erase(m_speed.begin());
	while(m_accel.size() > max_points)
		m_accel.erase(m_accel.begin());

	m_track.push_back(Vec2f(x - m_width/2, y - m_height/2)/m_place_coeff * place_length);

	if(m_track.size() > 1){
		Vec2f p = *(m_track.end() - 1) - *(m_track.end() - 2);
		m_delta = norm(p);

		m_speed.push_back(p);
	}

	if(m_speed.size() > 1){
		Vec2f p = *(m_speed.end() - 1) - *(m_speed.end() - 2);

		m_accel.push_back(p);
	}
}

void TrackerPoint::clear()
{
	m_track.clear();
}

void TrackerPoint::draw_searching(const vector< Vec2f> &pts)
{
	Mat mat = Mat(m_height, m_width, CV_8UC3, Scalar(255, 255, 255));

	Vec2f cv = Vec2f(m_width/2, m_height/2);

	for(size_t i = 1; i < m_track.size(); i++){
		circle(mat, V2P(m_track[i - 1]/place_length * m_place_coeff + cv), 10, cv::Scalar(180, 180, 255), 3);

		line(mat, V2P(m_track[i - 1]/place_length * m_place_coeff + cv),
				V2P(m_track[i]/place_length * m_place_coeff + cv), Scalar(180, 180, 255), 1);

//			line(mat_speed, c, c + m_speed[i - 1], Scalar(0, 255, 0), 1);
	}
	circle(mat, V2P(m_track.back()/place_length * m_place_coeff + cv), 10, cv::Scalar(255, 180, 180), 3);

	draw_axes(mat);

	if(pts.size())
		circle(mat, V2P(pts[0]/place_length * m_place_coeff + cv), 10, cv::Scalar(0, 0, 150), 4);
	if(pts.size() > 1)
		circle(mat, V2P(pts[1]/place_length * m_place_coeff + cv), 10, cv::Scalar(0, 150, 0), 4);

	for(size_t j = 2; j < pts.size(); j++){
		circle(mat, V2P(pts[j]/place_length * m_place_coeff + cv), 10, cv::Scalar(150, 0, 0), 4);
	}



	for(size_t i = 1; i < m_searching.size(); i++){
		double d = (double)( i) / m_searching.size();
		line(mat, V2P(m_searching[i - 1]/place_length * m_place_coeff + cv),
				V2P(m_searching[i]/place_length * m_place_coeff + cv), Scalar(255 - d * 255., 255 - d * 145., 255 - d * 250.), 3, 4);

//			line(mat_speed, c, c + m_speed[i - 1], Scalar(0, 255, 0), 1);
	}

	m_obj.draw_obj(mat, m_place_coeff);

//	imshow("searching", mat);

	if(func){
		func(mat);
	}
}

void TrackerPoint::draw_axes(Mat &mat)
{
	if(mat.empty())
		return;

	size_t cnt = place_length;

	Point cp = Point(mat.cols/2, mat.rows/2), p1, p2;

	double x = 0;
	for(size_t i = 0; i < cnt; i++){
		x = i / place_length;
		p1 = Point(mat.cols/2 + x * m_place_coeff, 0);
		p2 = Point(mat.cols/2 + x * m_place_coeff, mat.rows);

		line(mat, p1, p2, Scalar(160, 160, 160), 2, 4);

		p1 = Point(mat.cols/2 - x * m_place_coeff, 0);
		p2 = Point(mat.cols/2 - x * m_place_coeff, mat.rows);
		line(mat, p1, p2, Scalar(160, 160, 160), 2, 4);

		p1 = Point(0, mat.rows/2 + x * m_place_coeff);
		p2 = Point(mat.cols, mat.rows/2 + x * m_place_coeff);

		line(mat, p1, p2, Scalar(160, 160, 160), 2, 4);

		p1 = Point(0, mat.rows/2 - x * m_place_coeff);
		p2 = Point(mat.cols, mat.rows/2 - x * m_place_coeff);
		line(mat, p1, p2, Scalar(160, 160, 160), 2, 4);
	}
}

void TrackerPoint::randn(size_t cnt){
//	setRNGSeed(time(0));

	m_track.resize(cnt);
	cv::randn(m_track, Vec2f(0, 0), Vec2f(place_length/4, place_length/4));
}

void TrackerPoint::init()
{
	m_obj.Kp = 0.98;
	m_obj.Ki = 0.07;
	m_obj.Kd = 0.4;
	m_obj.v = 0.5;
	m_obj.max_v = 0.5;

	randn();
}

void TrackerPoint::calc(){

	if(m_track.size() < 2){
		cout << "need more points\n";
		return;
	}

	m_done = false;

	m_searching.clear();

	Vec2f v;

	Vec2f pt = (m_track.front()), prev = pt;
	size_t id = 1, id_prev = 0;;

	cout << "count_point=" << m_track.size() << endl;

//	namedWindow("searching");

	Vec2f u;

	m_obj.pos = pt;

	while(id < m_track.size() && !m_done){
		Vec2f pn = (m_track[id]);
		Vec2f d = pn - pt;
		double n = norm(d);

		Vec2f v_des = d;
//			d = clip(d, q_max);

		if(n < m_epsilon){
			prev = pn;
			id_prev = id;
			id++;
			continue;
		}

		m_current_id = id;

		double theta_des = atan2(v_des[1], v_des[0]);
		double e_k = theta_des - m_obj.get_theta();

		e_k = atan2(sin(e_k), cos(e_k));
		cout << "theta_des=" << theta_des * 180. / CV_PI << " e_k=" << e_k * 180. / CV_PI << endl;

		double e_p = e_k;

		double E_I = m_obj.E_k + e_k;

		double e_d = e_k - m_obj.e_k;

		m_obj.w = m_obj.Kp * e_p + m_obj.Ki * E_I + m_obj.Kd * e_d;

		m_obj.e_k = e_k;
		m_obj.E_k = E_I;

		v += m_obj.ensure_v();

		crop(v, m_speed_max);

		pt += v;
		m_obj.pos = pt;

		m_searching.push_back(pt);

		vector< Vec2f > pts;
		pts.push_back(prev);
		pts.push_back(pn);
		if(id + 1 < m_track.size()){
			pts.push_back(m_track[id + 1]);
		}

		stringstream ss;
		ss << "next id:\t" << id << endl;
		ss << "dist:\t" << n << endl;
		ss << "e_k:\t" << e_k << endl;
		ss << "E_I:\t" << E_I << endl;
		ss << "theta:\t" << R2A(m_obj.theta) << endl;
		ss << "th_des:\t" << R2A(theta_des) << endl;
		ss << "w:\t" << R2A(m_obj.w) << endl;
		ss << "v:\t" << R2A(m_obj.v) << endl;
		ss << "pos.x:\t" << m_obj.pos[0] << endl;
		ss << "pos.y:\t" << m_obj.pos[1] << endl;
		ss << "v.x:\t" << v[0] << endl;
		ss << "v.y:\t" << v[1] << endl;

		m_parameters = ss.str();

		draw_searching(pts);

		{
			m_obj.update_values();
			v *= m_koeff_resist;
		}
	}
}

void TrackerPoint::paint(Mat& mat){
	mat = Mat(m_height, m_width, CV_8UC3, Scalar(255, 255, 255));

	draw_axes(mat);

	if(m_track.empty())
		return;

//		Mat mat_speed = Mat::zeros(mat.size(), mat.type());
	Vec2f cv = Vec2f(m_width/2, m_height/2);

	for(size_t i = 1; i < m_track.size(); i++){
		circle(mat, V2P(m_track[i - 1]/place_length * m_place_coeff + cv), 10, cv::Scalar(0, 0, 150), 3);

		line(mat, V2P(m_track[i - 1]/place_length * m_place_coeff + cv),
				V2P(m_track[i]/place_length * m_place_coeff + cv), Scalar(0, 40, 255),1);

//			line(mat_speed, c, c + m_speed[i - 1], Scalar(0, 255, 0), 1);
	}
	circle(mat, V2P(m_track.back()/place_length * m_place_coeff + cv), 10, cv::Scalar(0, 0, 150), 3);

	for(size_t i = 1; i < m_searching.size(); i++){
		line(mat, V2P(m_searching[i - 1]/place_length * m_place_coeff + cv),
				V2P(m_searching[i]/place_length * m_place_coeff + cv), Scalar(0, 255, 45),3);

//			line(mat_speed, c, c + m_speed[i - 1], Scalar(0, 255, 0), 1);
	}


//		imshow("mat_speed", mat_speed);

	stringstream ss;
	ss << "count: " << m_track.size() << "; delta: " << m_delta;
	putText(mat, ss.str(), Point(10, 10), 1, 1, Scalar(255, 255,255), 2);
}

void TrackerPoint::close()
{
	m_done = true;
}

void TrackerPoint::set_size(int w, int h)
{
	m_width = w;
	m_height = h;
	m_place_coeff = min(m_width, m_height);
}

Obj &TrackerPoint::obj()
{
	return m_obj;
}

string TrackerPoint::print_parameters() const
{
	return m_parameters;
}

const std::string xml_config("config.xml");

void TrackerPoint::load_xml()
{
	FileStorage fs(xml_config, FileStorage::READ);

	if(!fs.isOpened())
		return;

	m_obj.Kp = fs["Kp"];
	m_obj.Ki = fs["Ki"];
	m_obj.Kd = fs["Kd"];

	m_obj.max_v = fs["max_v"];
	m_obj.max_w = fs["max_w"];

	m_width = fs["width"];
	m_height = fs["height"];

	int cnt = fs["count_track"];

	if(!fs["speedMax"].empty()){
		m_speed_max = fs["speedMax"];
	}

	if(!fs["resist"].empty()){
		m_koeff_resist = fs["resist"];
	}
	if(!fs["epsilon_pass"].empty()){
		m_epsilon = fs["epsilon_pass"];
	}

	randn(cnt);
}

void TrackerPoint::save_xml()
{
	FileStorage fs(xml_config, FileStorage::WRITE);

	fs << "Kp" << m_obj.Kp;
	fs << "Ki" << m_obj.Ki;
	fs << "Kd" << m_obj.Kd;
	fs << "max_w" << m_obj.max_w;
	fs << "max_v" << m_obj.max_v;
	fs << "width" << m_width;
	fs << "height" << m_height;

	int cnt = m_track.size();
	fs << "count_track" << cnt;

	fs << "speedMax" << m_speed_max;
	fs << "resist" << m_koeff_resist;

	fs << "epsilon_pass" << m_epsilon;
}
