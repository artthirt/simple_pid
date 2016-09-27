#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <memory>
#include <functional>

extern const int c_width;
extern const int c_height;

namespace tracking{

inline cv::Vec2f P2V(const cv::Point& pt)
{
	return cv::Vec2f(pt.x, pt.y);
}

inline cv::Point V2P(const cv::Vec2f& v)
{
	return cv::Point(v[0], v[1]);
}

inline cv::Vec2f clip(const cv::Vec2f& p, double max)
{
	cv::Vec2f res = p;
	double n = cv::norm(res);
	if(n > max){
		res = cv::normalize(res);
		res *= max;
	}
	return res;
}

struct Obj{
	Obj(){
		v = w = e_k = E_k = 0;
		theta = 0;
		Kp = Ki = Kd = 0;

		max_w = CV_PI/10;
		max_v = 0.1;
	}

	double v;
	double w;
	double theta;
	double e_k;
	double E_k;

	double Kp;
	double Ki;
	double Kd;

	double max_v;
	double max_w;

	cv::Vec2f pos;

	double get_theta() const;

	void update_values();

	cv::Vec2f ensure_v();

	cv::Vec2f speed() const;

	void draw_obj(cv::Mat& mat, float place_coeff);
};

class TrackerPoint{
public:
	TrackerPoint();
	~TrackerPoint();

	std::function< void(const cv::Mat&) > func;

	void set_pos(int x, int y);

	void clear();

	void draw_searching(const std::vector<cv::Vec2f> &pts);

	void draw_axes(cv::Mat& mat);

	void randn(size_t cnt = 20);

	void init();

	void calc();

	void paint(cv::Mat& mat);

	void close();

	void set_size(int w, int h);

	Obj &obj();

	size_t count() const { return m_track.size(); }

	int width() const { return m_width; }
	int height() const { return m_height; }

	size_t current_id() const { return m_current_id; }

	std::string print_parameters() const;

	void setSpeedMax(double val) { m_speed_max = val; }
	double speedMax() const { return m_speed_max; }

	void load_xml();
	void save_xml();

private:
	std::vector< cv::Vec2f > m_track;
	std::vector< cv::Vec2f > m_speed;
	std::vector< cv::Vec2f > m_accel;
	std::vector< cv::Vec2f > m_searching;
	double m_delta;
	double m_speed_max;
	double m_koeff_resist;
	double m_epsilon;
	int m_next;

	std::string m_parameters;

	size_t m_current_id;

	bool m_done;

	int m_width;
	int m_height;
	float m_place_coeff;

	Obj m_obj;
};

}


#endif // TRACKING_H
