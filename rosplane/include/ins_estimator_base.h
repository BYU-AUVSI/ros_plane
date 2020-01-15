/**
 * @file ins_estimator_base.h
 *
 * Inertial sense estimator
 *
 */

#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rosplane_msgs/State.h>
// #include <rosflight_msgs/GPS.h>
#include <inertial_sense/GPS.h>
#include <sensor_msgs/Imu.h>
#include <rosflight_msgs/Barometer.h>
#include <rosflight_msgs/Airspeed.h>
#include <rosflight_msgs/Status.h>
#include <math.h>
#include <Eigen/Eigen>

#include <numeric> // for 18.04

#define EARTH_RADIUS 6378145.0f
#define FILTER_LENGTH 16

namespace rosplane
{


class ins_estimator_base
{
public:
  ins_estimator_base();

protected:

  struct input_s
  {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float static_pres;
    float diff_pres;
    bool gps_new;
    float gps_n;
    float gps_e;
    float gps_h;
    float gps_Vg;
    float gps_course;
    bool status_armed;
    bool armed_init;
  };

  struct output_s
  {
    float pn;
    float pe;
    float h;
    float Va;
    float alpha;
    float beta;
    float phi;
    float theta;
    float psi;
    float chi;
    float p;
    float q;
    float r;
    float Vg;
    float wn;
    float we;
  };

  struct params_s
  {
    double gravity;
    double rho;
    double sigma_accel;
    double sigma_n_gps;
    double sigma_e_gps;
    double sigma_Vg_gps;
    double sigma_course_gps;
    double Ts;
  };

  virtual void estimate(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher vehicle_state_pub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber baro_sub_;
  ros::Subscriber airspeed_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber inertial_sense_sub_;

  void updateAltitudeAndAirspeed(const ros::TimerEvent &);
  void update(const ros::TimerEvent &);
  // void gpsCallback(const rosflight_msgs::GPS &msg);
  void gpsCallback(const inertial_sense::GPS &msg);
  void imuCallback(const sensor_msgs::Imu &msg);
  void baroAltCallback(const rosflight_msgs::Barometer &msg);
  void airspeedCallback(const rosflight_msgs::Airspeed &msg);
  void statusCallback(const rosflight_msgs::Status &msg);
  void inertialSenseCallback(const nav_msgs::Odometry &msg_in);
  void updateAirspeed(const ros::TimerEvent &);
	void filterAirspeed(float &Va);

  double update_rate_;
  ros::Timer update_timer_;
  std::string gps_topic_;
  std::string imu_topic_;
  std::string baro_topic_;
  std::string airspeed_topic_;
  std::string status_topic_;

  float lpf_static_base_;
  float lpf_diff_base_;
  float Ve_base_;
  float Vn_base_;
  float alpha1_base_;
  float Vahat_;
  float hhat_;
	float filter_taps_[FILTER_LENGTH] = {0.00970489747784546,	0.0145629416394102,	0.0282895307460455,	0.0485513149017384,
		0.0718604720902472,	0.0941826681792385,	0.111642954778265,	0.121205220187210,	0.121205220187210,
		0.111642954778265,	0.0941826681792385,	0.0718604720902472,	0.0485513149017384,	0.0282895307460455,
		0.0145629416394102,	0.00970489747784546};
	float Va_history_[FILTER_LENGTH];
  bool gps_new_;
  bool gps_init_;
  double init_lat_;       /**< Initial latitude in degrees */
  double init_lon_;       /**< Initial longitude in degrees */
  float init_alt_;        /**< Initial altitude in meters above MSL  */
  bool armed_first_time_; /**< Arm before starting estimation  */
  bool baro_init_;        /**< Initial barometric pressure */
  float init_static_;     /**< Initial static pressure (mbar)  */
  int baro_count_;        /**< Used to grab the first set of baro measurements */
  std::vector<float> init_static_vector_; /**< Used to grab the first set of baro measurements */

  struct params_s                 params_;
  struct input_s                  input_;


  double height_offset_;
  double calibrate_to_this_;
  double calibration_sum_;
  int avg_this_many_;
  int counted_this_many_ ;
};

} //end namespace

#endif // ESTIMATOR_BASE_H
