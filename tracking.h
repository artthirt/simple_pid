#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <memory>
#include <functional>

extern const int c_width;
extern const int c_height;

namespace tracking{

/**
 * @brief P2V
 * @param pt
 * @return
 */
inline cv::Vec2f P2V(const cv::Point& pt)
{
	return cv::Vec2f(pt.x, pt.y);
}

/**
 * @brief V2P
 * @param v
 * @return
 */
inline cv::Point V2P(const cv::Vec2f& v)
{
	return cv::Point(v[0], v[1]);
}

/**
 * @brief clip
 * clip 'p' value to 'max' range
 * @param p
 * @param max
 * @return
 */
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

/**
 * @brief The Obj struct
 */
struct Obj{
	Obj();

	double v;		// speed
	double w;		// angle speed
	double theta;	// angle of object
	double e_k;
	double E_k;		// integral accumulator

	double Kp;		// proportional
	double Ki;		// integral
	double Kd;		// derivative

	double max_v;	// restrict of speed value
	double max_w;	// restrict of angle speed value

	cv::Vec2f pos;	// position of object

	double distance_trigger;	// distance trigger
	double kp_v;				// proportional
	double kd_v;				// derivative
	double prev_speed;			// for calc derivative
	double dist_switch;			// for get distance when trigger on
	double speed_switch;		// for get speed when trigger on
	bool last;					// if last goal
	bool lock;					// for trigger
	/**
	 * @brief get_theta
	 * current theta angle
	 * @return
	 */
	double get_theta() const;
	/**
	 * @brief update_values
	 * for udpate some values (now do not anything)
	 */
	void update_values();
	/**
	 * @brief ensure_v
	 * get vector of speed with restriction
	 * @return
	 */
	cv::Vec2f ensure_v();
	/**
	 * @brief speed
	 * get poor vector of speed
	 * @return
	 */
	cv::Vec2f speed() const;
	/**
	 * @brief draw_obj
	 * draw object on image with its direction
	 * @param mat
	 * @param place_coeff
	 */
	void draw_obj(cv::Mat& mat, float place_coeff);
	/**
	 * @brief init
	 * init some parameters
	 */
	void init();
	/**
	 * @brief calc_v
	 * calculation speed when distance to goal less than distance trigger
	 * @param dist_to_goal
	 * @param real_speed
	 */
	void calc_v(double dist_to_goal, double real_speed);
};

class TrackerPoint{
public:
	TrackerPoint();
	~TrackerPoint();
	/**
	 * @brief func
	 * for signal for one step when work calc
	 */
	std::function< void(const cv::Mat&) > func;
	/**
	 * @brief set_pos
	 * add point on track
	 * @param x
	 * @param y
	 */
	void set_pos(int x, int y);
	/**
	 * @brief clear
	 */
	void clear();
	/**
	 * @brief draw_searching
	 * draw on image current trajectory with current next and after next goal point
	 * @param pts - [0] point - current; [1] - next goal; [2] - after next goal
	 */
	void draw_searching(const std::vector<cv::Vec2f> &pts);
	/**
	 * @brief draw_axes
	 * draw grid in image
	 * @param mat
	 */
	void draw_axes(cv::Mat& mat);
	/**
	 * @brief randn
	 * generate random track with 'cnt' points
	 * @param cnt
	 */
	void randn(size_t cnt = 20);
	/**
	 * @brief init
	 * reset some parameters for begin
	 */
	void init();
	/**
	 * @brief calc
	 * main cycle for work pid
	 */
	void calc();
	/**
	 * @brief paint
	 * draw ending trajectory with track points
	 * @param mat
	 */
	void paint(cv::Mat& mat);
	/**
	 * @brief close
	 * break calc
	 */
	void close();
	/**
	 * @brief set_size
	 * set size of output image
	 * @param w
	 * @param h
	 */
	void set_size(int w, int h);
	/**
	 * @brief obj
	 * object for pid
	 * @return
	 */
	Obj &obj();
	/**
	 * @brief count
	 * @return
	 */
	size_t count() const { return m_track.size(); }
	/**
	 * @brief width
	 * @return
	 */
	int width() const { return m_width; }
	/**
	 * @brief height
	 * @return
	 */
	int height() const { return m_height; }
	/**
	 * @brief current_id
	 * @return
	 */
	size_t current_id() const { return m_current_id; }
	/**
	 * @brief print_parameters
	 * some debug parameters
	 * @return
	 */
	std::string print_parameters() const;
	/**
	 * @brief setSpeedMax
	 * set restriction for maximum speed
	 * @param val
	 */
	void setSpeedMax(double val) { m_speed_max = val; }
	/**
	 * @brief speedMax
	 * @return
	 */
	double speedMax() const { return m_speed_max; }
	/**
	 * @brief load_xml
	 */
	void load_xml();
	/**
	 * @brief save_xml
	 */
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
