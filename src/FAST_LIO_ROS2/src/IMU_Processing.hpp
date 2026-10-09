#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"
#include "GapProcessCovariance.hpp"
#include "AdaptiveGapCovariance.hpp"
#include "ImuOnlyGapLearning.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  // The single acceleration convention used by FAST-LIO propagation and
  // Local BA preintegration: scaled_acc = raw_acc * accel_scale_.
  double acceleration_scale() const { return accel_scale_; }
  bool initialized() const { return !imu_need_init_; }
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  AdaptiveGapCovariance gap_adaptive;
  ImuOnlyGapLearning gap_imu_only;
  std::vector<std::pair<double,double>> gap_imu_only_pending;
  std::string gap_block_mode = "full";
  std::ofstream gap_applied_log;
  bool gap_covariance_enable = false;
  bool gap_skip_empty = false;
  double gap_min_seconds = 0.1;
  // Adaptive gap input (requires gap_imu_only.adaptive_input and a ready learner).
  // For IMU intervals longer than gap_input_min_dt and for the scan-end tail, choose
  // per channel the candidate input (endpoint or window mean) with the smallest
  // learned dynamics error + per-sample noise / actual sample count.
  bool gap_input_adaptive = false;
  // LiDAR constant-velocity deskew fallback: when the IMU samples inside a scan leave a gap
  // longer than deskew_cv_max_imu_gap, deskew with the body angular/linear velocity of the
  // last two LiDAR-corrected states instead of the IMU poses (sparse or mistimed IMU).
  bool deskew_lidar_cv_fallback = false;
  double deskew_cv_max_imu_gap = 0.025;
  bool lidar_cv_valid = false;
  V3D lidar_cv_w = Zero3d, lidar_cv_v = Zero3d;   // body frame, at the last scan end
  size_t deskew_cv_scans = 0;
  bool last_scan_used_cv = false;
  // false: legacy deskew anchor (retained state labelled at scan start, offset 0).
  bool deskew_anchor_at_state_time = true;
  // Gyro change test (adaptive mode): a window-mean gyro is kept only when it agrees with
  // the endpoint average within IMU noise (chi-square, 3 dof); otherwise the angular rate
  // is changing (window mean lags) and the endpoint average is used.
  bool gap_gyro_change_test = false;
  double gap_gyro_change_chi2 = 11.345;  // chi2(3) 99 %
  size_t gyro_change_rejects = 0;
  // Fixed-rule baseline: for the same intervals, replace the acceleration input by the
  // value giving zero world acceleration (constant velocity); gyro stays endpoint.
  bool gap_cv_fixed = false;
  // > 0: in the fixed rule, gyro = mean of received samples in [head - W, head] plus the tail
  // sample (no learning). 0: endpoint average (the original fixed rule).
  double gap_cv_gyro_window = 0.0;
  double gap_input_min_dt = 0.0125;
  Eigen::Matrix3d gap_q_alpha = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d gap_q_accel_world = Eigen::Matrix3d::Zero();
  bool deskew_enable = true; // false => skip backward propagation (de-skew); forward propagation always runs
  // Values greater than 1 increase IMU process noise, so the filter trusts IMU less.
  double imu_trust_scale = 1.0;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double accel_scale_ = 1.0;
  double start_timestamp_;
  double last_lidar_end_time_ = 0.0;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
  // Raw-unit acc such that R*(scale*acc - ba) + g = 0.
  V3D constantVelocityAcc(const state_ikfom &x) const {
    const V3D g(x.grav[0], x.grav[1], x.grav[2]);
    return (x.ba - x.rot.conjugate() * g) / accel_scale_;
  }
  struct RawImu { double t; V3D w, a; };
  std::deque<RawImu> input_hist_;
  void pushInputHistory(const sensor_msgs::msg::Imu::ConstSharedPtr &m);
  // Replaces w/a (raw units) for an interval of length h ending at `tail` (nullptr:
  // scan-end extrapolation). Returns effective densities for gap covariance.
  void chooseGapInput(double head_t, const sensor_msgs::msg::Imu::ConstSharedPtr &tail, double h,
                      V3D &w, V3D &a, double &q_alpha, double &q_accel, std::string &tag);
  // true = angular rate is changing relative to the window mean (use the endpoint average).
  bool gyroChanging(const V3D &endpoint_w, const V3D &window_w, int n_window) const;
};

void ImuProcess::pushInputHistory(const sensor_msgs::msg::Imu::ConstSharedPtr &m)
{
  const double t = rclcpp::Time(m->header.stamp).seconds();
  if (!input_hist_.empty() && t <= input_hist_.back().t) return;
  input_hist_.push_back({t, V3D(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z),
                         V3D(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z)});
  while (!input_hist_.empty() && t - input_hist_.front().t > std::max(gap_imu_only.maxWindow(), gap_cv_gyro_window) + 1.0) input_hist_.pop_front();
}

void ImuProcess::chooseGapInput(double head_t, const sensor_msgs::msg::Imu::ConstSharedPtr &tail, double h,
                                V3D &w, V3D &a, double &q_alpha, double &q_accel, std::string &tag)
{
  const auto &L = gap_imu_only;
  const size_t K = L.input_windows.size() + 1;
  std::vector<V3D> mw(K, V3D::Zero()), ma(K, V3D::Zero());
  std::vector<int> n(K, 0);
  for (const auto &r : input_hist_) {
    if (r.t > head_t) continue;
    for (size_t k = 1; k < K; ++k)
      if (r.t >= head_t - L.input_windows[k-1]) { mw[k] += r.w; ma[k] += r.a; ++n[k]; }
  }
  if (tail) {
    const V3D tw(tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z);
    const V3D ta(tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z);
    for (size_t k = 1; k < K; ++k) { mw[k] += tw; ma[k] += ta; ++n[k]; }
  }
  // Candidate 0 (endpoint): learned on the same two-sample structure, no extra noise.
  size_t g = 0, c = 0;
  double best_g = L.qr_c[0], best_a = L.qv_c[0];
  double noise_g = 0., noise_a = 0.;
  for (size_t k = 1; k < K; ++k) {
    if (n[k] < 2) continue;
    // Window-mean noise expressed in the same densities: rotation var sigma_g^2 h^2/N
    // = q h^3/3, velocity var sigma_a^2 h^2/N = q h.
    const double ng = 3. * L.noise_gyr / (n[k] * h), na = L.noise_acc * h / n[k];
    if (L.qr_c[k] + ng < best_g) { best_g = L.qr_c[k] + ng; g = k; noise_g = ng; }
    if (L.qv_c[k] + na < best_a) { best_a = L.qv_c[k] + na; c = k; noise_a = na; }
  }
  bool rejected = false;
  if (g && gap_gyro_change_test && gyroChanging(w, mw[g] / n[g], n[g])) {
    // Case 2: rotation changing -> endpoint gyro and its own learned density.
    g = 0; noise_g = 0.; rejected = true; ++gyro_change_rejects;
  }
  if (g) w = mw[g] / n[g];   // case 1: rotation steady within noise -> smoothed gyro
  if (c) a = ma[c] / n[c];   // raw units, like the endpoint average (scaled by the caller)
  q_alpha = L.qr_c[g] + noise_g;
  q_accel = L.qp_c[c] + noise_a;
  std::ostringstream os;
  os << "g" << (g ? L.input_windows[g-1] : 0.) << (rejected ? "t" : "") << "/n" << n[g] << ",a" << (c ? L.input_windows[c-1] : 0.) << "/n" << n[c];
  tag = os.str();
}

bool ImuProcess::gyroChanging(const V3D &endpoint_w, const V3D &window_w, int n_window) const
{
  // Per-axis noise var of the difference: endpoint average (2 samples) + window mean (N samples).
  const double var = gap_imu_only.noise_gyr * (0.5 + 1.0 / std::max(n_window, 1));
  if (!(var > 0.)) return true;  // noise unknown -> keep the endpoint average
  return (endpoint_w - window_w).squaredNorm() / var > gap_gyro_change_chi2;
}

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  accel_scale_  = 1.0;
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  accel_scale_  = 1.0;
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  last_lidar_end_time_ = 0.0;
  acc_s_last.setZero();
  input_hist_.clear();
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  accel_scale_ = G_m_s2 / std::max(1e-12, mean_acc.norm());
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  // The retained state belongs to the previous propagated scan end, even
  // when empty scans were skipped. Keep that timestamp in the current scan's
  // coordinates; labelling this state as offset 0 shifts the first deskew
  // segment forward in time. No extra mean/covariance propagation is done here.
  const double state_offset = deskew_anchor_at_state_time ? last_lidar_end_time_ - pcl_beg_time : 0.0;
  IMUpose.push_back(set_pose6d(state_offset, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
    double head_stamp = rclcpp::Time(head->header.stamp).seconds();

    if (tail_stamp < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
    pushInputHistory(head);
    std::string gap_input = "native";
    Eigen::Matrix3d interval_q_alpha = gap_q_alpha, interval_q_accel = gap_q_accel_world;
    if (gap_input_adaptive && tail_stamp - head_stamp > gap_input_min_dt) {
      double qa_eff, qacc_eff;
      chooseGapInput(head_stamp, tail, tail_stamp - head_stamp, angvel_avr, acc_avr, qa_eff, qacc_eff, gap_input);
      interval_q_alpha = Eigen::Matrix3d::Identity() * qa_eff;
      interval_q_accel = Eigen::Matrix3d::Identity() * qacc_eff;
    } else if (gap_cv_fixed && tail_stamp - head_stamp > gap_input_min_dt) {
      acc_avr = constantVelocityAcc(kf_state.get_x());
      gap_input = "cv_fixed";
      if (gap_cv_gyro_window > 0) {
        V3D w = V3D(tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z); int n = 1;
        for (const auto &r : input_hist_)
          if (r.t <= head_stamp && r.t >= head_stamp - gap_cv_gyro_window) { w += r.w; ++n; }
        angvel_avr = w / n;
        gap_input = "cv_fixed_gyro_window";
      }
    }

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * accel_scale_; // bias is subtracted inside get_f().

    if(head_stamp < last_lidar_end_time_)
    {
      dt = tail_stamp - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail_stamp - head_stamp;
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr * imu_trust_scale;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc * imu_trust_scale;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    if (gap_adaptive.enabled) {
      gap_covariance::Mat23 F;
      Eigen::Matrix<double,23,12> G;
      kf_state.predict(dt,Q,in,&F,&G);
      gap_adaptive.predict(F,G);
    } else kf_state.predict(dt, Q, in);
    // Experimental completed-gap injection. The interval is processed once:
    // dt is clipped to the last propagated LiDAR end, not record/header lag.
    // This hold-model approximation is NOT calibrated for endpoint averaging.
    if (gap_covariance_enable && tail_stamp-head_stamp >= gap_min_seconds && dt > 0.) {
      const auto added = gap_covariance::embedFastLio(
          gap_covariance::selectBlocks(gap_covariance::discrete(dt, interval_q_alpha, interval_q_accel),gap_block_mode));
      if(gap_applied_log) {
        gap_applied_log<<std::setprecision(17)<<head_stamp<<','<<tail_stamp<<','<<dt<<','<<gap_block_mode;
        for(int i=0;i<3;++i)gap_applied_log<<','<<interval_q_alpha(i,i);
        for(int i=0;i<3;++i)gap_applied_log<<','<<interval_q_accel(i,i);
        gap_applied_log<<','<<added.block<3,3>(3,3).trace()<<','<<added.block<3,3>(12,12).trace()<<','<<added.block<3,3>(0,0).trace()<<'\n';gap_applied_log.flush();
      }
      auto covariance = kf_state.get_P();
      covariance += added;
      kf_state.change_P(covariance);
      gap_adaptive.addGap(dt);
      if(gap_imu_only.enabled) gap_imu_only_pending.emplace_back(dt,tail_stamp);
      std::cout << "GAP_DP head=" << std::setprecision(17) << head_stamp
                << " tail=" << tail_stamp << " integrated_dt=" << dt
                << " theta_trace=" << added.block<3,3>(3,3).trace()
                << " velocity_trace=" << added.block<3,3>(12,12).trace()
                << " position_trace=" << added.block<3,3>(0,0).trace()
                << " input=" << gap_input << std::endl;
    }

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail_stamp - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  // Scan-end tail extrapolation has the same single-sample input problem.
  // No covariance is added for the tail (unchanged); only the mean input.
  if (gap_input_adaptive && note > 0 && dt > gap_input_min_dt) {
    pushInputHistory(v_imu.back());
    V3D w = in.gyro, a = in.acc / accel_scale_;
    double qa_eff, qacc_eff; std::string tag;
    chooseGapInput(imu_end_time, nullptr, dt, w, a, qa_eff, qacc_eff, tag);
    in.gyro = w; in.acc = a * accel_scale_;
  } else if (gap_cv_fixed && note > 0 && dt > gap_input_min_dt) {
    in.acc = constantVelocityAcc(kf_state.get_x()) * accel_scale_;
    if (gap_cv_gyro_window > 0) {
      pushInputHistory(v_imu.back());
      V3D w = V3D::Zero(); int n = 0;
      for (const auto &r : input_hist_)
        if (r.t <= imu_end_time && r.t >= imu_end_time - gap_cv_gyro_window) { w += r.w; ++n; }
      if (n > 0) in.gyro = w / n;
    }
  }
  if (gap_adaptive.enabled) {
    gap_covariance::Mat23 F;
    Eigen::Matrix<double,23,12> G;
    kf_state.predict(dt,Q,in,&F,&G);
    gap_adaptive.predict(F,G);
  } else kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  last_scan_used_cv = false;
  if (deskew_enable && deskew_lidar_cv_fallback && lidar_cv_valid && !pcl_out.points.empty())
  {
    // Largest IMU spacing over [pcl_beg, pcl_end], counting the scan edges.
    double prev = pcl_beg_time, max_gap = 0.;
    for (const auto &m : v_imu) {
      const double t = rclcpp::Time(m->header.stamp).seconds();
      if (t <= pcl_beg_time) continue;
      if (t > pcl_end_time) break;
      max_gap = std::max(max_gap, t - prev); prev = t;
    }
    max_gap = std::max(max_gap, pcl_end_time - prev);
    if (max_gap > deskew_cv_max_imu_gap) {
      // q_end = Exp(w*tau) q_t + v*tau, tau = t_point - t_end <= 0 (IMU body frame), then to LiDAR.
      for (auto &pt : pcl_out.points) {
        const double tau = pt.curvature / 1000.0 + pcl_beg_time - pcl_end_time;
        V3D q = imu_state.offset_R_L_I * V3D(pt.x, pt.y, pt.z) + imu_state.offset_T_L_I;
        V3D q_end = Exp(lidar_cv_w, tau) * q + lidar_cv_v * tau;
        V3D p = imu_state.offset_R_L_I.conjugate() * (q_end - imu_state.offset_T_L_I);
        pt.x = p(0); pt.y = p(1); pt.z = p(2);
      }
      last_scan_used_cv = true; ++deskew_cv_scans;
      return;
    }
  }
  if (deskew_enable)
  {
    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out.points.begin() == pcl_out.points.end()) return;
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu<<MAT_FROM_ARRAY(head->rot);
      // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
      vel_imu<<VEC_FROM_ARRAY(head->vel);
      pos_imu<<VEC_FROM_ARRAY(head->pos);
      acc_imu<<VEC_FROM_ARRAY(tail->acc);
      angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

      for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
      {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame, using only the rotation
         * Note: Compensation direction is INVERSE of Frame's moving direction
         * So if we want to compensate a point at timestamp-i to the frame-e
         * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
        M3D R_i(R_imu * Exp(angvel_avr, dt));

        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
        V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!

        // save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_out.points.begin()) break;
      }
    }
  }
  // else: pcl_out (feats_undistort) keeps each point's original coordinates
  //       as sampled at its own timestamp; only forward propagation ran.
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if (gap_adaptive.enabled) {
    // A prior event without a valid LiDAR update is rejected, never learned
    // from a later unrelated correction (e.g. map initialization/too few points).
    if (gap_adaptive.gaps>0)
      gap_adaptive.finish(last_lidar_end_time_, Eigen::Matrix<double,23,1>::Zero(), false,
                          gap_q_alpha, gap_q_accel_world);
    gap_adaptive.beginEvent();
  }
  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();
    // Initialization has consumed measurements up to this actual timestamp.
    // Establish a defined anchor before the first normal propagation.
    last_lidar_end_time_ = rclcpp::Time(last_imu_->header.stamp).seconds();
    acc_s_last.setZero();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      std::cout << "IMU Initial Done" << std::endl;
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      {
        state_ikfom s = kf_state.get_x();
        printf("[InitDone] t=%lf p=%lf %lf %lf v=%lf %lf %lf bg=%lf %lf %lf ba=%lf %lf %lf g=%lf %lf %lf |g|=%lf\n",
          first_lidar_time,
          s.pos[0], s.pos[1], s.pos[2],
          s.vel[0], s.vel[1], s.vel[2],
          s.bg[0], s.bg[1], s.bg[2],
          s.ba[0], s.ba[1], s.ba[2],
          s.grav[0], s.grav[1], s.grav[2],
          sqrt(s.grav[0]*s.grav[0] + s.grav[1]*s.grav[1] + s.grav[2]*s.grav[2]));
      }
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
