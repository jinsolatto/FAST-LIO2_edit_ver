// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <array>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <mutex>
#include <unordered_set>
#include <math.h>
#include <thread>
#include <fstream>
#include <filesystem>
#include <csignal>
#include <chrono>
#include <deque>
#include <iomanip>
#include <limits>
#include <cstdlib>
#include <unistd.h>
#include <sched.h>
#include <sstream>
#include <cerrno>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "IMU_Processing.hpp"
#include "voxel_plane_shadow.hpp"
#include "vslam_port/voxel_map.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double imu_trust_scale = 1.0;
bool imu_trust_window_enable = false; // true => use imu_trust_window_value instead of imu_trust_scale inside the window below
double imu_trust_window_start_time = 0.0;
double imu_trust_window_end_time = 1.0e9;
double imu_trust_window_value = 1.0; // <1.0 => trust IMU MORE than imu_trust_scale during the window (lower process noise -> lidar update pulls less)
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   point_selected_stage2[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;
bool   tum_trajectory_log_enable = false;
bool   pose_trajectory_log_enable = false;
double imu_degraded_dt_threshold_ms = 15.0;
double lidar_weak_rotation_threshold = -1.0; // <=0 => unset: run once to inspect the r_lambda_min column, then set explicitly
constexpr double ADAPTIVE_FINE_VOXEL_SCALE = 0.5;
bool deskew_enable = true; // false => skip backward propagation (de-skew) in UndistortPcl; forward propagation (predict) always runs
bool front_reject_enable = false; // true => drop all front-hemisphere (+X) raw points inside the window below
double FRONT_REJECT_START_TIME = 260.0;
double FRONT_REJECT_END_TIME = 390.0;
bool phased_halfplane_fine_voxel_enable = false; // true => switch fine voxel target half-plane by time window
double phased_halfplane_fine_voxel_leaf_size = -1.0; // <= 0 uses filter_size_surf * ADAPTIVE_FINE_VOXEL_SCALE
double phased_halfplane_other_leaf_size = -1.0; // <=0 => untargeted half-plane uses voxel_normal/total-background as before; >0 => it also ramps toward this value instead
double phased_halfplane_back_start_time = 20.0;
double phased_halfplane_back_end_time = 65.0;
double phased_halfplane_front_start_time = 66.0;
double phased_halfplane_front_end_time = 87.0;
double phased_halfplane_back_resume_start_time = 88.0;
double phased_halfplane_back_resume_end_time = 1.0e9;
string phased_halfplane_phase1_target = "back";
string phased_halfplane_phase2_target = "front";
string phased_halfplane_phase3_target = "back";
struct FineVoxelSegment
{
    double start_time = 0.0;
    double end_time = 0.0;
    string target = "back";
};
std::vector<FineVoxelSegment> phased_halfplane_segments;
bool windowed_voxel_enable = false; // true => use windowed_voxel_leaf_size instead of filter_size_surf inside the window below
double windowed_voxel_start_time = 290.0;
double windowed_voxel_end_time = 296.0;
double windowed_voxel_leaf_size = 0.2;
double blind_default_value = 0.01; // preprocess.blind's configured value; always used outside blind_window below (or when disabled)
bool blind_window_enable = false; // true => use blind_window_value instead of preprocess.blind inside the window below
double blind_window_start_time = 0.0;
double blind_window_end_time = 1.0e9;
double blind_window_value = 0.01;
// Ramp (linear-interpolation) voxel-size transition for "total"-target phased_halfplane
// segments, replacing the old hard step at segment_start/segment_end. <=0 sentinels fall
// back to the pre-existing values (filter_size_surf_min / resolve_fine_voxel_leaf_size)
// so this is opt-in and never silently drifts from the stock leaf size.
double voxel_size_normal = -1.0;   // <=0 => use active_surf_leaf_size (filter_size_surf_min or windowed override)
bool voxel_size_fine_enable = true; // false => "total"-target segments have no effect at all; voxel size stays voxel_size_normal throughout (voxel_size_fine is ignored)
double voxel_size_fine = -1.0;     // <=0 => use resolve_fine_voxel_leaf_size(active_surf_leaf_size)
double voxel_ramp_duration = 0.0;  // seconds; 0 => identical to the old hard step (regression case)

// IMU-only windows: while relative_time falls inside any window below, the LiDAR
// measurement update (kf.update_iterated_dyn_share_modified) is skipped entirely for
// that scan, so the state stays at whatever the IMU-only forward propagation already
// produced (kf.get_x() right before the update call). Map insertion still runs using
// that IMU-predicted pose. Meant for stretches where the robot is known to have been
// physically lifted/tilted, so the LiDAR scan (mismatched against the map) never gets
// to correct the state at all during that stretch.
bool imu_only_enable = false;
vector<double> imu_only_start_times;
vector<double> imu_only_end_times;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
string tum_trajectory_log_path;
string pose_trajectory_log_path;
string pcd_save_path;
constexpr double DEGENERACY_LAMBDA_MIN_THRESHOLD = 15.0;
// Read-only, per-scan diagnostic for comparing full-bag and partial-bag runs.
// This file is intentionally separate from the historical degeneracy/BA logs.
double degeneracy_direction_threshold = 0.12;
int degeneracy_min_valid_plane_num = 50;
int degeneracy_temporal_window = 10;
int degeneracy_temporal_bad_count = 7;
deque<bool> degeneracy_frame_bad_history;
double warning_direction_threshold = 0.15;
int warning_min_plane_num = 60;
double plane_drop_threshold = -0.35;
int warning_window = 5;
int warning_count_threshold = 3;
deque<int> degeneracy_plane_count_history;
deque<bool> degeneracy_warning_history;

// Local BA parameters. EKF pose feedback is optional and never changes
// velocity, biases, covariance, or IMU preintegration.
int local_ba_window_size = 10;
int local_ba_min_constraints = 50;
int local_ba_max_iterations = 10;
double local_ba_huber_delta = 0.1;
double local_ba_max_translation_correction = 0.5;
double local_ba_max_rotation_correction_deg = 5.0;
bool local_ba_imu_enable = false;
double local_ba_imu_rot_weight = 1.0;
double local_ba_imu_vel_weight = 1.0;
double local_ba_imu_pos_weight = 1.0;
bool local_ba_map_feedback_enable = false;
double local_ba_map_feedback_max_translation = 0.20;
double local_ba_map_feedback_max_rotation_deg = 2.0;
bool local_ba_map_pose_valid = false;
Eigen::Quaterniond local_ba_map_q = Eigen::Quaterniond::Identity();
Eigen::Vector3d local_ba_map_t = Eigen::Vector3d::Zero();
string local_ba_map_pose_source = "FAST_LIO";
bool local_ba_feedback_translation_gate_pass = false;
bool local_ba_feedback_rotation_gate_pass = false;
string local_ba_feedback_reject_reason = "DISABLED";
bool local_ba_ekf_pose_feedback_enable = false;
double local_ba_ekf_feedback_max_translation = 0.05;
double local_ba_ekf_feedback_max_rotation_deg = 0.5;
double local_ba_ekf_feedback_max_velocity = 0.20;
double local_ba_feedback_alpha_normal = 0.10;
double local_ba_feedback_alpha_warning = 0.05;
double local_ba_feedback_alpha_degenerate = 0.0;
double local_ba_velocity_alpha_normal = 0.10;
double local_ba_velocity_alpha_warning = 0.05;
double local_ba_velocity_alpha_degenerate = 0.0;
bool local_ba_ekf_pose_feedback_used = false;
string local_ba_ekf_feedback_reject_reason = "DISABLED";
// Kept false for pure FAST-LIO runs.  This gate is outside the estimator and
// prevents the optional Local BA side path from being invoked at all.
bool local_ba_enable = false;
bool balm_export_enable = false;
double balm_export_start_sec = 430.0;
double balm_export_end_sec = 480.0;
string balm_export_root = "/tmp/balm_export";

struct PlaneConstraint
{
    int frame_id = -1;
    Eigen::Vector3d point_body;
    Eigen::Vector3d plane_normal;
    double plane_d = 0.0;
};

struct LocalBAImuSample
{
    double timestamp = 0.0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

// Gravity is deliberately excluded from these deltas.  It is fixed and used
// only in the IMU residual, matching the standard preintegration convention.
struct IMUPreintegration
{
    int from_frame = -1;
    int to_frame = -1;
    Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
    double dt = 0.0;
    double accel_scale = 1.0;
    double raw_accel_norm = 0.0;
    double scaled_accel_norm = 0.0;
    double accel_bias_norm = 0.0;
    int imu_sample_count = 0;
};

struct LocalBAFrame
{
    int frame_id = -1;
    double timestamp = 0.0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    double accel_scale = 1.0;
    vector<LocalBAImuSample> imu_samples;
    PointCloudXYZI::Ptr scan;
    vector<PlaneConstraint> constraints;
};

struct LocalBAResult
{
    bool triggered = false;
    bool ba_executed = false;
    bool ba_converged = false;
    bool ba_accepted = false;
    int window_size = 0;
    int num_plane_constraints = 0;
    double lidar_rmse_before = std::numeric_limits<double>::quiet_NaN();
    double lidar_rmse_after = std::numeric_limits<double>::quiet_NaN();
    double cost_before = std::numeric_limits<double>::quiet_NaN();
    double cost_after = std::numeric_limits<double>::quiet_NaN();
    double latest_delta_translation = 0.0;
    double latest_delta_rotation_deg = 0.0;
    double ba_translation_correction = 0.0;
    double ba_rotation_correction_deg = 0.0;
    double ba_velocity_correction = 0.0;
    int imu_factor_count = 0;
    double imu_rot_rmse_before = std::numeric_limits<double>::quiet_NaN();
    double imu_rot_rmse_after = std::numeric_limits<double>::quiet_NaN();
    double imu_vel_rmse_before = std::numeric_limits<double>::quiet_NaN();
    double imu_vel_rmse_after = std::numeric_limits<double>::quiet_NaN();
    double imu_pos_rmse_before = std::numeric_limits<double>::quiet_NaN();
    double imu_pos_rmse_after = std::numeric_limits<double>::quiet_NaN();
    double preint_rot_residual_before = std::numeric_limits<double>::quiet_NaN();
    double preint_vel_residual_before = std::numeric_limits<double>::quiet_NaN();
    double preint_pos_residual_before = std::numeric_limits<double>::quiet_NaN();
    double accel_scale = 1.0;
    double raw_accel_norm = std::numeric_limits<double>::quiet_NaN();
    double scaled_accel_norm = std::numeric_limits<double>::quiet_NaN();
    double accel_bias_norm = std::numeric_limits<double>::quiet_NaN();
    double delta_v_norm = std::numeric_limits<double>::quiet_NaN();
    double delta_p_norm = std::numeric_limits<double>::quiet_NaN();
    double imu_dt = 0.0;
    int imu_sample_count = 0;
    Eigen::Quaterniond optimized_q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d optimized_t = Eigen::Vector3d::Zero();
    Eigen::Vector3d optimized_v = Eigen::Vector3d::Zero();
    bool feedback_allowed = false;
    bool feedback_applied = false;
    string feedback_reject_reason = "NOT_TRIGGERED";
    string feedback_mode = "BA_NOT_READY";
    double feedback_alpha_pose = 0.0;
    double feedback_alpha_velocity = 0.0;
    Eigen::Vector3d fast_pose_before = Eigen::Vector3d::Zero();
    Eigen::Vector3d fast_velocity_before = Eigen::Vector3d::Zero();
};

struct PointToPlaneCost
{
    PointToPlaneCost(const Eigen::Vector3d &point, const Eigen::Vector3d &normal, double d)
        : point_(point), normal_(normal), d_(d) {}

    template <typename T>
    bool operator()(const T *const q_xyzw, const T *const t, T *residual) const
    {
        // EigenQuaternionParameterization uses Eigen's [x, y, z, w] storage.
        const Eigen::Quaternion<T> q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
        const Eigen::Matrix<T, 3, 1> p = point_.cast<T>();
        const Eigen::Matrix<T, 3, 1> n = normal_.cast<T>();
        const Eigen::Matrix<T, 3, 1> translation(t[0], t[1], t[2]);
        residual[0] = n.dot(q * p + translation) + T(d_);
        return true;
    }

    Eigen::Vector3d point_;
    Eigen::Vector3d normal_;
    double d_;
};

struct IMUPreintegrationCost
{
    IMUPreintegrationCost(const IMUPreintegration &preint, const Eigen::Vector3d &gravity,
                          double rot_weight, double vel_weight, double pos_weight)
        : delta_q_(preint.delta_q), delta_v_(preint.delta_v), delta_p_(preint.delta_p),
          gravity_(gravity), dt_(preint.dt), sqrt_rot_weight_(std::sqrt(rot_weight)),
          sqrt_vel_weight_(std::sqrt(vel_weight)), sqrt_pos_weight_(std::sqrt(pos_weight)) {}

    template <typename T>
    bool operator()(const T *const qi_xyzw, const T *const pi, const T *const vi,
                    const T *const qj_xyzw, const T *const pj, const T *const vj,
                    T *residuals) const
    {
        const Eigen::Quaternion<T> qi(qi_xyzw[3], qi_xyzw[0], qi_xyzw[1], qi_xyzw[2]);
        const Eigen::Quaternion<T> qj(qj_xyzw[3], qj_xyzw[0], qj_xyzw[1], qj_xyzw[2]);
        const Eigen::Quaternion<T> dq_meas(T(delta_q_.w()), T(delta_q_.x()), T(delta_q_.y()), T(delta_q_.z()));
        const Eigen::Quaternion<T> q_error = dq_meas.conjugate() * (qi.conjugate() * qj);
        const Eigen::Matrix<T, 3, 1> q_vec(q_error.x(), q_error.y(), q_error.z());
        const T q_vec_norm = ceres::sqrt(q_vec.squaredNorm());
        // Log(q): 2 atan2(||imag(q)||, real(q)) * imag(q)/||imag(q)||.
        const T log_scale = T(2.0) * ceres::atan2(q_vec_norm, q_error.w()) / (q_vec_norm + T(1e-12));
        const Eigen::Matrix<T, 3, 1> r_rot = log_scale * q_vec;

        const Eigen::Matrix<T, 3, 1> p_i(pi[0], pi[1], pi[2]);
        const Eigen::Matrix<T, 3, 1> p_j(pj[0], pj[1], pj[2]);
        const Eigen::Matrix<T, 3, 1> v_i(vi[0], vi[1], vi[2]);
        const Eigen::Matrix<T, 3, 1> v_j(vj[0], vj[1], vj[2]);
        const Eigen::Matrix<T, 3, 1> gravity = gravity_.cast<T>();
        const Eigen::Matrix<T, 3, 1> r_vel = qi.conjugate() *
            (v_j - v_i - gravity * T(dt_)) - delta_v_.cast<T>();
        const Eigen::Matrix<T, 3, 1> r_pos = qi.conjugate() *
            (p_j - p_i - v_i * T(dt_) - gravity * T(0.5 * dt_ * dt_)) - delta_p_.cast<T>();
        for (int k = 0; k < 3; ++k)
        {
            residuals[k] = T(sqrt_rot_weight_) * r_rot(k);
            residuals[3 + k] = T(sqrt_vel_weight_) * r_vel(k);
            residuals[6 + k] = T(sqrt_pos_weight_) * r_pos(k);
        }
        return true;
    }

    Eigen::Quaterniond delta_q_;
    Eigen::Vector3d delta_v_, delta_p_, gravity_;
    double dt_, sqrt_rot_weight_, sqrt_vel_weight_, sqrt_pos_weight_;
};

vector<PlaneConstraint> latest_local_ba_constraints;

// h_share_model() runs once or more times per iterated EKF update. It only
// refreshes this result; LaserMappingNode writes it once per LiDAR scan.
struct DegeneracyResult
{
    double lambda_min = std::numeric_limits<double>::quiet_NaN();
    double lambda_mid = std::numeric_limits<double>::quiet_NaN();
    double lambda_max = std::numeric_limits<double>::quiet_NaN();
    int valid_plane_num = 0;
    double lambda_min_norm = std::numeric_limits<double>::quiet_NaN();
    // Eigen's SelfAdjoint solver returns ascending eigenvalues, and column 0
    // is therefore the direction corresponding to lambda_min.
    Eigen::Vector3d weak_eigenvector = Eigen::Vector3d::Zero();
    bool eigen_decomposition_valid = false;
    bool is_degenerate_voxelslam = false;
    bool direction_bad = false;
    bool support_bad = false;
    bool frame_bad = false;
    int bad_count_10 = 0;
    bool persistent_degenerate = false;
    bool direction_warning = false;
    bool support_warning = false;
    double plane_drop_ratio = 0.0;
    bool trend_warning = false;
    bool warning_frame = false;
    int warning_count_5 = 0;
    bool early_warning = false;
};
DegeneracyResult latest_degeneracy_result;
// Stage-0 shadow comparison only: built/queried alongside the real pipeline,
// never fed into the EKF. See voxel_plane_shadow.hpp.
voxel_shadow::ShadowVoxelMap g_shadow_voxel_map;

// When set (env var FASTLIO_USE_VOXEL_MATCHING), the real VoxelSLAM-port
// OctoTree map REPLACES ikd-tree matching in h_share_model and is grown with
// FAST-LIO2's own real pose (not GT) -- i.e. actually wired into the EKF,
// not just a shadow comparison. The Stage-0/0b/0c shadow-only blocks are
// skipped in this mode (they'd otherwise fight over the shared vp:: globals).
bool g_use_voxel_matching = (std::getenv("FASTLIO_USE_VOXEL_MATCHING") != nullptr);

// GT-pose lookup for the shadow map (isolates the matching algorithm from
// FAST-LIO2's own (drifting) pose estimate): loads a TUM file once and
// nearest-neighbor-matches by timestamp.
struct GtPose { double t; Eigen::Vector3d p; Eigen::Matrix3d R; };
std::vector<GtPose> g_gt_poses;
bool g_gt_loaded = false;

void load_gt_poses(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        std::cerr << "[shadow-gt] failed to open GT file: " << path << std::endl;
        return;
    }
    double t, x, y, z, qx, qy, qz, qw;
    while (f >> t >> x >> y >> z >> qx >> qy >> qz >> qw)
    {
        GtPose gp;
        gp.t = t;
        gp.p = Eigen::Vector3d(x, y, z);
        gp.R = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
        g_gt_poses.push_back(gp);
    }
    std::cerr << "[shadow-gt] loaded " << g_gt_poses.size() << " GT poses from " << path << std::endl;
}

// Nearest-timestamp GT pose lookup (binary search, since g_gt_poses is sorted
// by time in the file). Returns false if nothing within max_dt.
bool lookup_gt_pose(double t, Eigen::Vector3d &p_out, Eigen::Matrix3d &R_out, double max_dt = 0.02)
{
    if (g_gt_poses.empty()) return false;
    auto it = std::lower_bound(g_gt_poses.begin(), g_gt_poses.end(), t,
        [](const GtPose &a, double val) { return a.t < val; });
    double best_dt = 1e18;
    const GtPose *best = nullptr;
    if (it != g_gt_poses.end()) { double d = std::fabs(it->t - t); if (d < best_dt) { best_dt = d; best = &(*it); } }
    if (it != g_gt_poses.begin()) { auto prev = std::prev(it); double d = std::fabs(prev->t - t); if (d < best_dt) { best_dt = d; best = &(*prev); } }
    if (!best || best_dt > max_dt) return false;
    p_out = best->p;
    R_out = best->R;
    return true;
}

// ---- Stage-0c: literal port of VoxelSLAM's real OctoTree/cut_voxel/match ----
// (vslam_port/*.hpp, copied verbatim from Voxel-SLAM/VoxelSLAM/src). Driven by
// GT poses, single-threaded, no local BA -- this is exactly what VoxelSLAM's
// odom_only frontend uses for matching + adaptive map building. Purely a
// shadow comparison; never touches state_point/kf.
namespace vp = vslam_port;
bool g_real_vp_inited = false;
std::unordered_map<vp::VOXEL_LOC, vp::OctoTree *> g_real_surf_map;
std::vector<vp::IMUST> g_real_xbuf;
std::vector<vp::PVecPtr> g_real_pvecbuf;
std::vector<std::vector<vp::SlideWindow *>> g_real_sws(1);
int g_real_win_count = 0;
const int g_real_win_size = 10;
vp::LidarFactor *g_real_voxhess = nullptr;

void init_real_vp()
{
    g_real_vp_inited = true;
    vp::mp = new int[g_real_win_size];
    for (int i = 0; i < g_real_win_size; i++) vp::mp[i] = i;
    vp::voxel_size = 1.0;
    vp::min_eigen_value = 0.0025;
    vp::max_layer = 2;
    vp::plane_eigen_value_thre = {4.0, 4.0, 4.0, 4.0};
    vp::min_point << 5, 5, 5, 5;
    g_real_voxhess = new vp::LidarFactor(g_real_win_size);
}

// Port of VoxelSLAM's calcBodyVar (voxelslam.hpp:162) -- per-point range/beam
// noise model, computed in LiDAR frame before the extrinsic rotation.
void calc_body_var_port(Eigen::Vector3d &pb, double range_inc, double degree_inc, Eigen::Matrix3d &var)
{
    if (pb[2] == 0) pb[2] = 0.0001;
    double range = pb.norm();
    double range_var = range_inc * range_inc;
    double s = std::sin(degree_inc * M_PI / 180.0);
    Eigen::Matrix2d direction_var;
    direction_var << s * s, 0, 0, s * s;
    Eigen::Vector3d direction = pb.normalized();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1),
                      direction(2), 0, -direction(0),
                      -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<int>                        raw_point_count_buffer;
deque<int>                        blind_rejected_buffer;
deque<int>                        front_rejected_buffer;
deque<int>                        front_raw_count_buffer;
deque<int>                        back_raw_count_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr stage2_normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;
KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());
inline double azimuth_deg_wrapped(const PointType &point)
{
    const double deg = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)) * 180.0 / M_PI;
    return deg < 0.0 ? deg + 360.0 : deg;
}

inline int azimuth_quadrant_from_wrapped(double wrapped_deg)
{
    return static_cast<int>((wrapped_deg + 45.0) / 90.0) % 4;
}

inline int azimuth_quadrant(const PointType &point)
{
    return azimuth_quadrant_from_wrapped(azimuth_deg_wrapped(point));
}

inline int azimuth_halfplane_from_wrapped(double wrapped_deg)
{
    return (wrapped_deg <= 90.0 || wrapped_deg >= 270.0) ? 0 : 1;
}

inline bool use_windowed_voxel_window(double scan_relative_time)
{
    return windowed_voxel_enable &&
           scan_relative_time >= windowed_voxel_start_time &&
           scan_relative_time <= windowed_voxel_end_time;
}

inline bool use_blind_window(double relative_time)
{
    return blind_window_enable &&
           relative_time >= blind_window_start_time &&
           relative_time <= blind_window_end_time;
}

inline bool in_imu_only_window(double relative_time)
{
    if (!imu_only_enable)
    {
        return false;
    }
    const size_t n = std::min(imu_only_start_times.size(), imu_only_end_times.size());
    for (size_t i = 0; i < n; i++)
    {
        if (relative_time >= imu_only_start_times[i] && relative_time <= imu_only_end_times[i])
        {
            return true;
        }
    }
    return false;
}

inline bool use_imu_trust_window(double relative_time)
{
    return imu_trust_window_enable &&
           relative_time >= imu_trust_window_start_time &&
           relative_time <= imu_trust_window_end_time;
}

inline bool use_back_halfplane_fine_voxel_window(double scan_relative_time)
{
    return phased_halfplane_fine_voxel_enable &&
           scan_relative_time >= phased_halfplane_back_start_time &&
           scan_relative_time <= phased_halfplane_back_end_time;
}

inline bool use_back_halfplane_fine_voxel_resume_window(double scan_relative_time)
{
    return phased_halfplane_fine_voxel_enable &&
           scan_relative_time >= phased_halfplane_back_resume_start_time &&
           scan_relative_time <= phased_halfplane_back_resume_end_time;
}

inline bool use_front_halfplane_fine_voxel_window(double scan_relative_time)
{
    return phased_halfplane_fine_voxel_enable &&
           scan_relative_time >= phased_halfplane_front_start_time &&
           scan_relative_time <= phased_halfplane_front_end_time;
}

inline bool get_active_fine_voxel_segment(double scan_relative_time, FineVoxelSegment &active_segment)
{
    if (!phased_halfplane_fine_voxel_enable)
    {
        return false;
    }

    for (auto it = phased_halfplane_segments.rbegin(); it != phased_halfplane_segments.rend(); ++it)
    {
        if (scan_relative_time >= it->start_time &&
            scan_relative_time <= it->end_time)
        {
            active_segment = *it;
            return true;
        }
    }

    return false;
}

inline bool is_front_halfplane_target(const std::string &target)
{
    return target == "front" || target == "Front" || target == "FRONT";
}

inline bool is_back_halfplane_target(const std::string &target)
{
    return target == "back" || target == "Back" || target == "BACK";
}

inline bool is_total_halfplane_target(const std::string &target)
{
    return target == "total" || target == "Total" || target == "TOTAL";
}

inline void rebuild_legacy_fine_voxel_segments()
{
    phased_halfplane_segments.clear();
    if (!phased_halfplane_fine_voxel_enable)
    {
        return;
    }

    phased_halfplane_segments.push_back(
        {phased_halfplane_back_start_time, phased_halfplane_back_end_time, phased_halfplane_phase1_target});
    phased_halfplane_segments.push_back(
        {phased_halfplane_front_start_time, phased_halfplane_front_end_time, phased_halfplane_phase2_target});
    phased_halfplane_segments.push_back(
        {phased_halfplane_back_resume_start_time, phased_halfplane_back_resume_end_time, phased_halfplane_phase3_target});
}

inline int argmax_quadrant(const std::array<double, 4> &values)
{
    int best_q = 0;
    double best_value = values[0];
    for (int q = 1; q < 4; q++)
    {
        if (values[q] > best_value)
        {
            best_value = values[q];
            best_q = q;
        }
    }
    return best_q;
}

struct AdaptiveSectorSelection
{
    bool valid = false;
    int translation_sector = -1;
    int rotation_sector = -1;
    std::array<double, 4> translation_contrib = {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> rotation_contrib = {{0.0, 0.0, 0.0, 0.0}};
};

inline AdaptiveSectorSelection analyze_adaptive_sector_selection(const PointCloudXYZI::Ptr &down_cloud,
                                                                 const state_ikfom &s)
{
    AdaptiveSectorSelection selection;
    M3D translation_normal = M3D::Zero();
    M3D rotation_normal = M3D::Zero();
    std::vector<char> selected(down_cloud->points.size(), 0);
    std::vector<V3D> normals(down_cloud->points.size(), Zero3d);
    std::vector<V3D> rotation_terms(down_cloud->points.size(), Zero3d);

    for (size_t i = 0; i < down_cloud->points.size(); i++)
    {
        const PointType &point_body = down_cloud->points[i];
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

        PointType point_world;
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);

        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        PointVector points_near;
        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        const bool gate1_pass = points_near.size() >= NUM_MATCH_POINTS &&
                                pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5.0f;
        if (!gate1_pass)
        {
            continue;
        }

        VF(4) pabcd;
        if (!esti_plane(pabcd, points_near, 0.1f))
        {
            continue;
        }

        const float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
        const float score = 1 - 0.9f * std::fabs(pd2) / std::sqrt(p_body.norm());
        if (score <= 0.9f)
        {
            continue;
        }

        V3D norm_vec(pabcd(0), pabcd(1), pabcd(2));
        V3D point_this = s.offset_R_L_I * p_body + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);

        selected[i] = 1;
        normals[i] = norm_vec;
        rotation_terms[i] = A;
        translation_normal.noalias() += norm_vec * norm_vec.transpose();
        rotation_normal.noalias() += A * A.transpose();
    }

    if (translation_normal.squaredNorm() <= std::numeric_limits<double>::epsilon() ||
        rotation_normal.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        return selection;
    }

    Eigen::SelfAdjointEigenSolver<M3D> translation_solver(translation_normal);
    Eigen::SelfAdjointEigenSolver<M3D> rotation_solver(rotation_normal);
    if (translation_solver.info() != Eigen::Success || rotation_solver.info() != Eigen::Success)
    {
        return selection;
    }

    const V3D t_vmin = translation_solver.eigenvectors().col(0);
    const V3D r_vmin = rotation_solver.eigenvectors().col(0);
    for (size_t i = 0; i < down_cloud->points.size(); i++)
    {
        if (!selected[i])
        {
            continue;
        }
        const int q = azimuth_quadrant(down_cloud->points[i]);
        selection.translation_contrib[q] += std::pow(normals[i].dot(t_vmin), 2);
        selection.rotation_contrib[q] += std::pow(rotation_terms[i].dot(r_vmin), 2);
    }

    selection.valid = true;
    selection.translation_sector = argmax_quadrant(selection.translation_contrib);
    selection.rotation_sector = argmax_quadrant(selection.rotation_contrib);
    return selection;
}

inline void downsample_target_halfplane_with_fine_voxel(const PointCloudXYZI::Ptr &input_cloud,
                                                        PointCloudXYZI::Ptr &output_cloud,
                                                        double base_leaf_size,
                                                        double fine_leaf_size,
                                                        bool target_front_halfplane)
{
    PointCloudXYZI::Ptr front_cloud(new PointCloudXYZI());
    PointCloudXYZI::Ptr back_cloud(new PointCloudXYZI());
    for (const auto &point : input_cloud->points)
    {
        const int halfplane = azimuth_halfplane_from_wrapped(azimuth_deg_wrapped(point));
        if (halfplane == 0)
        {
            front_cloud->points.push_back(point);
        }
        else
        {
            back_cloud->points.push_back(point);
        }
    }

    PointCloudXYZI front_down;
    PointCloudXYZI back_down;
    pcl::VoxelGrid<PointType> filter;

    if (!front_cloud->empty())
    {
        const double front_leaf_size = target_front_halfplane ? fine_leaf_size : base_leaf_size;
        filter.setLeafSize(front_leaf_size, front_leaf_size, front_leaf_size);
        filter.setInputCloud(front_cloud);
        filter.filter(front_down);
    }

    if (!back_cloud->empty())
    {
        const double back_leaf_size = target_front_halfplane ? base_leaf_size : fine_leaf_size;
        filter.setLeafSize(back_leaf_size, back_leaf_size, back_leaf_size);
        filter.setInputCloud(back_cloud);
        filter.filter(back_down);
    }

    output_cloud->clear();
    output_cloud->points.reserve(front_down.points.size() + back_down.points.size());
    output_cloud->points.insert(output_cloud->points.end(), front_down.points.begin(), front_down.points.end());
    output_cloud->points.insert(output_cloud->points.end(), back_down.points.begin(), back_down.points.end());
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true;
}

inline void downsample_total_with_fine_voxel(const PointCloudXYZI::Ptr &input_cloud,
                                             PointCloudXYZI::Ptr &output_cloud,
                                             double fine_leaf_size)
{
    pcl::VoxelGrid<PointType> filter;
    filter.setLeafSize(fine_leaf_size, fine_leaf_size, fine_leaf_size);
    filter.setInputCloud(input_cloud);
    filter.filter(*output_cloud);
}

inline double resolve_fine_voxel_leaf_size(double base_leaf_size)
{
    const double configured_leaf_size = (phased_halfplane_fine_voxel_leaf_size > 0.0)
                                            ? phased_halfplane_fine_voxel_leaf_size
                                            : (base_leaf_size * ADAPTIVE_FINE_VOXEL_SCALE);
    return std::max(configured_leaf_size, 1e-3);
}

// Linear ramp (0=normal, 1=fine) for a single segment: rises over
// [segment_start-ramp_duration, segment_start], holds at 1 for
// [segment_start, segment_end], falls over [segment_end, segment_end+ramp_duration].
// ramp_duration<=0 degenerates to the old hard step (alpha is 0 or 1 only), which is
// the regression case required to reproduce pre-ramp behavior exactly.
inline double compute_voxel_size(double current_time,
                                 double segment_start, double segment_end,
                                 double voxel_normal, double voxel_fine,
                                 double ramp_duration)
{
    double alpha;
    if (ramp_duration <= 0.0)
    {
        alpha = (current_time >= segment_start && current_time <= segment_end) ? 1.0 : 0.0;
    }
    else if (current_time < segment_start)
    {
        alpha = (current_time - (segment_start - ramp_duration)) / ramp_duration;
    }
    else if (current_time <= segment_end)
    {
        alpha = 1.0;
    }
    else
    {
        alpha = 1.0 - (current_time - segment_end) / ramp_duration;
    }
    alpha = std::min(1.0, std::max(0.0, alpha));
    return voxel_normal - (voxel_normal - voxel_fine) * alpha;
}

// Evaluates the ramp across every "total"-target segment (front/back-target segments
// are left to the pre-existing downsample_target_halfplane_with_fine_voxel path, which
// this ramp does not touch) and keeps the finest (smallest) result, so overlapping ramps
// resolve to whichever segment wants the denser cloud at that instant.
inline double compute_voxel_size(double current_time,
                                 const std::vector<FineVoxelSegment> &segments,
                                 double voxel_normal, double voxel_fine,
                                 double ramp_duration)
{
    double result = voxel_normal;
    for (const auto &seg : segments)
    {
        if (!is_total_halfplane_target(seg.target))
        {
            continue;
        }
        const double v = compute_voxel_size(current_time, seg.start_time, seg.end_time,
                                            voxel_normal, voxel_fine, ramp_duration);
        result = std::min(result, v);
    }
    return result;
}

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// Map-insertion-only transform. The EKF state remains untouched; q and t are
// used only when an accepted Local BA pose has passed the feedback gate.
void pointBodyToWorldWithPose(PointType const * const pi, PointType * const po,
                              const Eigen::Quaterniond &q, const Eigen::Vector3d &t)
{
    const V3D p_lidar(pi->x, pi->y, pi->z);
    const V3D p_imu = state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I;
    const Eigen::Vector3d p_world = q * Eigen::Vector3d(p_imu.x(), p_imu.y(), p_imu.z()) + t;
    po->x = p_world.x();
    po->y = p_world.y();
    po->z = p_world.z();
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
        raw_point_count_buffer.clear();
        blind_rejected_buffer.clear();
        front_rejected_buffer.clear();
        front_raw_count_buffer.clear();
        back_raw_count_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    if (blind_window_enable && first_lidar_time > 0.0)
    {
        const double relative_time = cur_time - first_lidar_time;
        p_pre->blind = use_blind_window(relative_time) ? blind_window_value : blind_default_value;
    }
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    raw_point_count_buffer.push_back(raw_point_count);
    blind_rejected_buffer.push_back(blind_points_rejected);
    front_rejected_buffer.push_back(front_points_rejected);
    front_raw_count_buffer.push_back(front_raw_point_count);
    back_raw_count_buffer.push_back(back_raw_point_count);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;

template <typename LivoxMsgT>
void livox_pcl_cbk_impl(LivoxMsgT &&msg)
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
        raw_point_count_buffer.clear();
        blind_rejected_buffer.clear();
        front_rejected_buffer.clear();
        front_raw_count_buffer.clear();
        back_raw_count_buffer.clear();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    if (blind_window_enable && first_lidar_time > 0.0)
    {
        const double relative_time = last_timestamp_lidar - first_lidar_time;
        p_pre->blind = use_blind_window(relative_time) ? blind_window_value : blind_default_value;
    }
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    raw_point_count_buffer.push_back(raw_point_count);
    blind_rejected_buffer.push_back(blind_points_rejected);
    front_rejected_buffer.push_back(front_points_rejected);
    front_raw_count_buffer.push_back(front_raw_point_count);
    back_raw_count_buffer.push_back(back_raw_point_count);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    livox_pcl_cbk_impl(std::move(msg));
}

void livox_legacy_pcl_cbk(const std::shared_ptr<rclcpp::SerializedMessage> msg)
{
    static rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serializer;
    auto decoded_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
    serializer.deserialize_message(msg.get(), decoded_msg.get());
    livox_pcl_cbk(std::move(decoded_msg));
}

// ROS2 refuses to register two different message types for the same topic name within
// one node, so we can't subscribe with both the v2 and legacy CustomMsg types up front.
// Instead, wait for a publisher to advertise the topic and pick the matching type; the
// legacy livox_ros_driver/msg/CustomMsg and livox_ros_driver2/msg/CustomMsg wire formats
// are byte-identical, so the same decoder works for both.
std::string detect_avia_topic_type(rclcpp::Node *node, const std::string &topic,
                                    std::chrono::milliseconds timeout = std::chrono::seconds(30))
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    bool warned = false;
    while (rclcpp::ok())
    {
        for (const auto &topic_and_types : node->get_topic_names_and_types())
        {
            if (topic_and_types.first != topic)
                continue;
            for (const auto &type : topic_and_types.second)
            {
                if (type == "livox_ros_driver/msg/CustomMsg" || type == "livox_ros_driver2/msg/CustomMsg")
                    return type;
            }
        }
        if (std::chrono::steady_clock::now() >= deadline)
            return "";
        if (!warned)
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for a publisher on '%s' to determine CustomMsg type (livox_ros_driver vs livox_ros_driver2)...", topic.c_str());
            warned = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return "";
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        meas.raw_point_count = raw_point_count_buffer.front();
        meas.blind_rejected = blind_rejected_buffer.front();
        meas.front_rejected = front_rejected_buffer.front();
        meas.front_raw_count = front_raw_count_buffer.front();
        meas.back_raw_count = back_raw_count_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    raw_point_count_buffer.pop_front();
    blind_rejected_buffer.pop_front();
    front_rejected_buffer.pop_front();
    front_raw_count_buffer.pop_front();
    back_raw_count_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        if (local_ba_map_pose_valid)
        {
            pointBodyToWorldWithPose(&(feats_down_body->points[i]), &(feats_down_world->points[i]),
                                     local_ba_map_q, local_ba_map_t);
        }
        else
        {
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        }

        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudmsg);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    latest_local_ba_constraints.clear();
    vector<double> plane_d_by_point(feats_down_size, std::numeric_limits<double>::quiet_NaN());
    // VoxelSLAM weighs each point's contribution by R_inv = 1/(0.0005+sigma_d)
    // (voxelslam.cpp L930), down-weighting uncertain matches instead of trusting
    // every accepted point equally. FAST-LIO2's IKFOM update takes one scalar R
    // for all points (esekfom.hpp update_iterated_dyn_share_modified), so we
    // reproduce the effect by pre-whitening: scale h_x/h for point i by
    // sqrt(LASER_POINT_COV/(0.0005+sigma_d_i)) before the uniform-R update runs.
    // Default 0.0005 makes ikd-tree points (no sigma_d) get weight 1 (no-op).
    vector<double> plane_sigma_d_by_point(feats_down_size, 0.0005);
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    // #ifdef MP_EN
        // omp_set_num_threads(MP_PROC_NUM);
        // #pragma omp parallel for
    // #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 
        point_selected_stage2[i] = false;

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        if (g_use_voxel_matching)
        {
            point_selected_surf[i] = false;
            point_selected_stage2[i] = false;
            if (g_real_vp_inited)
            {
                Eigen::Vector3d wld(point_world.x, point_world.y, point_world.z);
                Eigen::Matrix3d pvar;
                Eigen::Vector3d p_lidar_for_var = p_body;
                calc_body_var_port(p_lidar_for_var, 0.02, 0.05, pvar);
                Eigen::Matrix3d Rli = s.offset_R_L_I.matrix();
                // VoxelSLAM's own lio_state_estimation (voxelslam.cpp L909-910) adds
                // the CURRENT filter's own pose uncertainty on top of sensor noise:
                //   var_world = R*pv.var*R^T + phat*rot_var*phat^T + tsl_var
                // Omitting this (as before) under-estimates var_world whenever the
                // filter itself is uncertain -- exactly when imu_trust_scale is
                // cranked up for a bad-IMU sequence -- which makes vp::match()'s
                // acceptance radius (3*sqrt(sigma_l)) too tight and starves
                // valid_plane_num for reasons unrelated to the actual geometry.
                Eigen::Vector3d pnt_imu = Rli * p_body + s.offset_T_L_I;
                Eigen::Matrix3d phat; phat << SKEW_SYM_MATRX(pnt_imu);
                auto P_cur = kf.get_P();
                Eigen::Matrix3d rot_var = P_cur.block<3, 3>(3, 3);
                Eigen::Matrix3d tsl_var = P_cur.block<3, 3>(0, 0);
                Eigen::Matrix3d var_world = s.rot.matrix() * Rli * pvar * Rli.transpose() * s.rot.matrix().transpose()
                                          + phat * rot_var * phat.transpose() + tsl_var;
                vp::Plane *pla = nullptr;
                double sigma_d = 0, max_prob = 0;
                vp::OctoTree *oc = nullptr;
                int flag = vp::match(g_real_surf_map, wld, pla, var_world, sigma_d, oc);
                if (flag && pla)
                {
                    // VoxelSLAM's own lio_state_estimation (voxelslam.cpp L926) accepts
                    // every point vp::match() flags valid -- no extra residual-ratio
                    // gate. Confidence is instead expressed via plane_sigma_d_by_point,
                    // which feeds the per-point measurement weighting below.
                    Eigen::Vector3d normal = pla->normal;
                    double d = -normal.dot(pla->center);
                    float pd2 = normal(0) * point_world.x + normal(1) * point_world.y + normal(2) * point_world.z + d;
                    point_selected_surf[i] = true;
                    point_selected_stage2[i] = true;
                    normvec->points[i].x = normal(0);
                    normvec->points[i].y = normal(1);
                    normvec->points[i].z = normal(2);
                    normvec->points[i].intensity = pd2;
                    stage2_normvec->points[i] = normvec->points[i];
                    plane_d_by_point[i] = d;
                    plane_sigma_d_by_point[i] = sigma_d;
                    res_last[i] = fabs(pd2);
                }
            }
            continue;
        }

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            point_selected_stage2[i] = true;
            stage2_normvec->points[i].x = pabcd(0);
            stage2_normvec->points[i].y = pabcd(1);
            stage2_normvec->points[i].z = pabcd(2);
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                plane_d_by_point[i] = pabcd(3);
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;
    vector<double> effct_sigma_d(feats_down_size, 0.0005);

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            effct_sigma_d[effct_feat_num] = plane_sigma_d_by_point[i];
            PlaneConstraint constraint;
            const PointType &point_lidar = feats_down_body->points[i];
            constraint.point_body = s.offset_R_L_I * V3D(point_lidar.x, point_lidar.y, point_lidar.z) + s.offset_T_L_I;
            constraint.plane_normal = V3D(normvec->points[i].x, normvec->points[i].y, normvec->points[i].z);
            constraint.plane_d = plane_d_by_point[i];
            latest_local_ba_constraints.push_back(constraint);
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    // Use exactly the point-to-plane correspondences retained for this EKF
    // iteration. Do not write the CSV here: this function can run repeatedly
    // for one LiDAR scan.
    M3D normal_matrix = M3D::Zero();
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &norm_p = corr_normvect->points[i];
        const V3D normal(norm_p.x, norm_p.y, norm_p.z);
        normal_matrix.noalias() += normal * normal.transpose();
    }

    latest_degeneracy_result = DegeneracyResult{};
    latest_degeneracy_result.valid_plane_num = effct_feat_num;
    if (effct_feat_num > 0)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(normal_matrix);
        if (solver.info() == Eigen::Success)
        {
            const Eigen::Vector3d eigenvalues = solver.eigenvalues();
            latest_degeneracy_result.lambda_min = eigenvalues(0);
            latest_degeneracy_result.lambda_mid = eigenvalues(1);
            latest_degeneracy_result.lambda_max = eigenvalues(2);
            latest_degeneracy_result.lambda_min_norm = eigenvalues(0) / effct_feat_num;
            latest_degeneracy_result.weak_eigenvector = solver.eigenvectors().col(0);
            latest_degeneracy_result.eigen_decomposition_valid = true;
            // Experimental Voxel-SLAM baseline; this does not alter FAST-LIO2.
            latest_degeneracy_result.is_degenerate_voxelslam = eigenvalues(0) < DEGENERACY_LAMBDA_MIN_THRESHOLD;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        // Pre-whiten by this point's VoxelSLAM-style confidence weight so the
        // IKFOM update (single scalar R across all rows) ends up equivalent to
        // per-point R_i = 0.0005+sigma_d_i weighting -- see declaration comment
        // on plane_sigma_d_by_point above.
        double w = std::sqrt(LASER_POINT_COV / (0.0005 + effct_sigma_d[i]));
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekfom_data.h_x.block<1, 12>(i, 0) *= w;

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity * w;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("mapping.imu_trust_scale", 1.0);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<double>("mapping.degeneracy_direction_threshold", 0.12);
        this->declare_parameter<int>("mapping.degeneracy_min_valid_plane_num", 50);
        this->declare_parameter<int>("mapping.degeneracy_temporal_window", 10);
        this->declare_parameter<int>("mapping.degeneracy_temporal_bad_count", 7);
        this->declare_parameter<double>("mapping.warning_direction_threshold", 0.15);
        this->declare_parameter<int>("mapping.warning_min_plane_num", 60);
        this->declare_parameter<double>("mapping.plane_drop_threshold", -0.35);
        this->declare_parameter<int>("mapping.warning_window", 5);
        this->declare_parameter<int>("mapping.warning_count_threshold", 3);
        this->declare_parameter<bool>("mapping.local_ba_enable", false);
        this->declare_parameter<bool>("mapping.balm_export_enable", false);
        this->declare_parameter<double>("mapping.balm_export_start_sec", 430.0);
        this->declare_parameter<double>("mapping.balm_export_end_sec", 480.0);
        this->declare_parameter<string>("mapping.balm_export_root", "/tmp/balm_export");
        this->declare_parameter<int>("mapping.local_ba_window_size", 10);
        this->declare_parameter<int>("mapping.local_ba_min_constraints", 50);
        this->declare_parameter<int>("mapping.local_ba_max_iterations", 10);
        this->declare_parameter<double>("mapping.local_ba_huber_delta", 0.1);
        this->declare_parameter<double>("mapping.local_ba_max_translation_correction", 0.5);
        this->declare_parameter<double>("mapping.local_ba_max_rotation_correction_deg", 5.0);
        this->declare_parameter<bool>("mapping.local_ba_imu_enable", false);
        this->declare_parameter<double>("mapping.local_ba_imu_rot_weight", 1.0);
        this->declare_parameter<double>("mapping.local_ba_imu_vel_weight", 1.0);
        this->declare_parameter<double>("mapping.local_ba_imu_pos_weight", 1.0);
        this->declare_parameter<bool>("mapping.local_ba_map_feedback_enable", false);
        this->declare_parameter<double>("mapping.local_ba_map_feedback_max_translation", 0.20);
        this->declare_parameter<double>("mapping.local_ba_map_feedback_max_rotation_deg", 2.0);
        this->declare_parameter<bool>("mapping.local_ba_ekf_pose_feedback_enable", false);
        this->declare_parameter<double>("mapping.local_ba_ekf_feedback_max_translation", 0.05);
        this->declare_parameter<double>("mapping.local_ba_ekf_feedback_max_rotation_deg", 0.5);
        this->declare_parameter<double>("mapping.local_ba_ekf_feedback_max_velocity", 0.20);
        this->declare_parameter<double>("mapping.local_ba_feedback_alpha_normal", 0.10);
        this->declare_parameter<double>("mapping.local_ba_feedback_alpha_warning", 0.05);
        this->declare_parameter<double>("mapping.local_ba_feedback_alpha_degenerate", 0.0);
        this->declare_parameter<double>("mapping.local_ba_velocity_alpha_normal", 0.10);
        this->declare_parameter<double>("mapping.local_ba_velocity_alpha_warning", 0.05);
        this->declare_parameter<double>("mapping.local_ba_velocity_alpha_degenerate", 0.0);
        this->declare_parameter<bool>("mapping.deskew_enable", true);
        this->declare_parameter<bool>("mapping.front_reject_enable", false);
        this->declare_parameter<double>("mapping.front_reject_start_time", 260.0);
        this->declare_parameter<double>("mapping.front_reject_end_time", 390.0);
        this->declare_parameter<bool>("mapping.phased_halfplane_fine_voxel_enable", false);
        this->declare_parameter<double>("mapping.phased_halfplane_fine_voxel_leaf_size", -1.0);
        this->declare_parameter<double>("mapping.phased_halfplane_other_leaf_size", -1.0);
        this->declare_parameter<double>("mapping.phased_halfplane_back_start_time", 20.0);
        this->declare_parameter<double>("mapping.phased_halfplane_back_end_time", 65.0);
        this->declare_parameter<double>("mapping.phased_halfplane_front_start_time", 66.0);
        this->declare_parameter<double>("mapping.phased_halfplane_front_end_time", 87.0);
        this->declare_parameter<double>("mapping.phased_halfplane_back_resume_start_time", 88.0);
        this->declare_parameter<double>("mapping.phased_halfplane_back_resume_end_time", 1.0e9);
        this->declare_parameter<string>("mapping.phased_halfplane_phase1_target", "back");
        this->declare_parameter<string>("mapping.phased_halfplane_phase2_target", "front");
        this->declare_parameter<string>("mapping.phased_halfplane_phase3_target", "back");
        this->declare_parameter<vector<double>>("mapping.phased_halfplane_segments_start_times", vector<double>());
        this->declare_parameter<vector<double>>("mapping.phased_halfplane_segments_end_times", vector<double>());
        this->declare_parameter<vector<string>>("mapping.phased_halfplane_segments_targets", vector<string>());
        this->declare_parameter<double>("mapping.voxel_size_normal", -1.0);
        this->declare_parameter<bool>("mapping.voxel_size_fine_enable", true);
        this->declare_parameter<double>("mapping.voxel_size_fine", -1.0);
        this->declare_parameter<double>("mapping.voxel_ramp_duration", 0.0);
        this->declare_parameter<bool>("mapping.windowed_voxel_enable", false);
        this->declare_parameter<double>("mapping.windowed_voxel_start_time", 290.0);
        this->declare_parameter<double>("mapping.windowed_voxel_end_time", 296.0);
        this->declare_parameter<double>("mapping.windowed_voxel_leaf_size", 0.2);
        this->declare_parameter<bool>("mapping.blind_window_enable", false);
        this->declare_parameter<double>("mapping.blind_window_start_time", 0.0);
        this->declare_parameter<double>("mapping.blind_window_end_time", 1.0e9);
        this->declare_parameter<double>("mapping.blind_window_value", 0.01);
        this->declare_parameter<bool>("mapping.imu_only_enable", false);
        this->declare_parameter<vector<double>>("mapping.imu_only_start_times", vector<double>());
        this->declare_parameter<vector<double>>("mapping.imu_only_end_times", vector<double>());
        this->declare_parameter<bool>("mapping.imu_trust_window_enable", false);
        this->declare_parameter<double>("mapping.imu_trust_window_start_time", 0.0);
        this->declare_parameter<double>("mapping.imu_trust_window_end_time", 1.0e9);
        this->declare_parameter<double>("mapping.imu_trust_window_value", 1.0);
        this->declare_parameter<double>("mapping.imu_degraded_dt_threshold_ms", 15.0);
        this->declare_parameter<double>("mapping.lidar_weak_rotation_threshold", -1.0);
        this->declare_parameter<bool>("mapping.tum_trajectory_log_enable", false);
        this->declare_parameter<string>("mapping.tum_trajectory_log_path", root_dir + "/Log/trajectory.tum");
        this->declare_parameter<bool>("mapping.pose_trajectory_log_enable", false);
        this->declare_parameter<string>("mapping.pose_trajectory_log_path", root_dir + "/Log/pose.txt");
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<string>("pcd_save.path", root_dir + "/PCD");
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("mapping.imu_trust_scale", imu_trust_scale, 1.0);
        if (imu_trust_scale <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "mapping.imu_trust_scale must be positive; using 1.0 instead of %.6f",
                        imu_trust_scale);
            imu_trust_scale = 1.0;
        }
        this->get_parameter_or<double>("preprocess.blind", blind_default_value, 0.01);
        p_pre->blind = blind_default_value; // overridden dynamically per-scan in the lidar callbacks if blind_window_enable
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<double>("mapping.degeneracy_direction_threshold", degeneracy_direction_threshold, 0.12);
        this->get_parameter_or<int>("mapping.degeneracy_min_valid_plane_num", degeneracy_min_valid_plane_num, 50);
        this->get_parameter_or<int>("mapping.degeneracy_temporal_window", degeneracy_temporal_window, 10);
        this->get_parameter_or<int>("mapping.degeneracy_temporal_bad_count", degeneracy_temporal_bad_count, 7);
        this->get_parameter_or<double>("mapping.warning_direction_threshold", warning_direction_threshold, 0.15);
        this->get_parameter_or<int>("mapping.warning_min_plane_num", warning_min_plane_num, 60);
        this->get_parameter_or<double>("mapping.plane_drop_threshold", plane_drop_threshold, -0.35);
        this->get_parameter_or<int>("mapping.warning_window", warning_window, 5);
        this->get_parameter_or<int>("mapping.warning_count_threshold", warning_count_threshold, 3);
        this->get_parameter_or<bool>("mapping.local_ba_enable", local_ba_enable, false);
        this->get_parameter_or<bool>("mapping.balm_export_enable", balm_export_enable, false);
        this->get_parameter_or<double>("mapping.balm_export_start_sec", balm_export_start_sec, 430.0);
        this->get_parameter_or<double>("mapping.balm_export_end_sec", balm_export_end_sec, 480.0);
        this->get_parameter_or<string>("mapping.balm_export_root", balm_export_root, "/tmp/balm_export");
        this->get_parameter_or<int>("mapping.local_ba_window_size", local_ba_window_size, 10);
        this->get_parameter_or<int>("mapping.local_ba_min_constraints", local_ba_min_constraints, 50);
        this->get_parameter_or<int>("mapping.local_ba_max_iterations", local_ba_max_iterations, 10);
        this->get_parameter_or<double>("mapping.local_ba_huber_delta", local_ba_huber_delta, 0.1);
        this->get_parameter_or<double>("mapping.local_ba_max_translation_correction", local_ba_max_translation_correction, 0.5);
        this->get_parameter_or<double>("mapping.local_ba_max_rotation_correction_deg", local_ba_max_rotation_correction_deg, 5.0);
        this->get_parameter_or<bool>("mapping.local_ba_imu_enable", local_ba_imu_enable, false);
        this->get_parameter_or<double>("mapping.local_ba_imu_rot_weight", local_ba_imu_rot_weight, 1.0);
        this->get_parameter_or<double>("mapping.local_ba_imu_vel_weight", local_ba_imu_vel_weight, 1.0);
        this->get_parameter_or<double>("mapping.local_ba_imu_pos_weight", local_ba_imu_pos_weight, 1.0);
        this->get_parameter_or<bool>("mapping.local_ba_map_feedback_enable", local_ba_map_feedback_enable, false);
        this->get_parameter_or<double>("mapping.local_ba_map_feedback_max_translation", local_ba_map_feedback_max_translation, 0.20);
        this->get_parameter_or<double>("mapping.local_ba_map_feedback_max_rotation_deg", local_ba_map_feedback_max_rotation_deg, 2.0);
        this->get_parameter_or<bool>("mapping.local_ba_ekf_pose_feedback_enable", local_ba_ekf_pose_feedback_enable, false);
        this->get_parameter_or<double>("mapping.local_ba_ekf_feedback_max_translation", local_ba_ekf_feedback_max_translation, 0.05);
        this->get_parameter_or<double>("mapping.local_ba_ekf_feedback_max_rotation_deg", local_ba_ekf_feedback_max_rotation_deg, 0.5);
        this->get_parameter_or<double>("mapping.local_ba_ekf_feedback_max_velocity", local_ba_ekf_feedback_max_velocity, 0.20);
        this->get_parameter_or<double>("mapping.local_ba_feedback_alpha_normal", local_ba_feedback_alpha_normal, 0.10);
        this->get_parameter_or<double>("mapping.local_ba_feedback_alpha_warning", local_ba_feedback_alpha_warning, 0.05);
        this->get_parameter_or<double>("mapping.local_ba_feedback_alpha_degenerate", local_ba_feedback_alpha_degenerate, 0.0);
        this->get_parameter_or<double>("mapping.local_ba_velocity_alpha_normal", local_ba_velocity_alpha_normal, 0.10);
        this->get_parameter_or<double>("mapping.local_ba_velocity_alpha_warning", local_ba_velocity_alpha_warning, 0.05);
        this->get_parameter_or<double>("mapping.local_ba_velocity_alpha_degenerate", local_ba_velocity_alpha_degenerate, 0.0);
        if (degeneracy_temporal_window < 1)
        {
            RCLCPP_WARN(this->get_logger(), "mapping.degeneracy_temporal_window must be positive; using 10");
            degeneracy_temporal_window = 10;
        }
        if (degeneracy_temporal_bad_count < 1 || degeneracy_temporal_bad_count > degeneracy_temporal_window)
        {
            RCLCPP_WARN(this->get_logger(), "mapping.degeneracy_temporal_bad_count must be in [1, temporal_window]; using %d", degeneracy_temporal_window);
            degeneracy_temporal_bad_count = degeneracy_temporal_window;
        }
        if (warning_window < 1)
        {
            RCLCPP_WARN(this->get_logger(), "mapping.warning_window must be positive; using 5");
            warning_window = 5;
        }
        if (warning_count_threshold < 1 || warning_count_threshold > warning_window)
        {
            RCLCPP_WARN(this->get_logger(), "mapping.warning_count_threshold must be in [1, warning_window]; using %d", warning_window);
            warning_count_threshold = warning_window;
        }
        if (local_ba_window_size < 2)
        {
            RCLCPP_WARN(this->get_logger(), "mapping.local_ba_window_size must be at least 2; using 2");
            local_ba_window_size = 2;
        }
        if (local_ba_min_constraints < 1 || local_ba_max_iterations < 1 ||
            local_ba_huber_delta <= 0.0 || local_ba_max_translation_correction <= 0.0 ||
            local_ba_max_rotation_correction_deg <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid Local BA parameter; using the default verification-only settings");
            local_ba_min_constraints = 50;
            local_ba_max_iterations = 10;
            local_ba_huber_delta = 0.1;
            local_ba_max_translation_correction = 0.5;
            local_ba_max_rotation_correction_deg = 5.0;
        }
        if (local_ba_imu_rot_weight <= 0.0 || local_ba_imu_vel_weight <= 0.0 || local_ba_imu_pos_weight <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid Local BA IMU weights; using unit diagonal weights");
            local_ba_imu_rot_weight = 1.0;
            local_ba_imu_vel_weight = 1.0;
            local_ba_imu_pos_weight = 1.0;
        }
        if (local_ba_map_feedback_max_translation <= 0.0 || local_ba_map_feedback_max_rotation_deg <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid Local BA map-feedback gate; using 0.20 m and 2.0 deg");
            local_ba_map_feedback_max_translation = 0.20;
            local_ba_map_feedback_max_rotation_deg = 2.0;
        }
        if (local_ba_ekf_feedback_max_translation <= 0.0 || local_ba_ekf_feedback_max_rotation_deg <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid Local BA EKF pose-feedback gate; using 0.05 m and 0.5 deg");
            local_ba_ekf_feedback_max_translation = 0.05;
            local_ba_ekf_feedback_max_rotation_deg = 0.5;
        }
        this->get_parameter_or<bool>("mapping.deskew_enable", deskew_enable, true);
        this->get_parameter_or<bool>("mapping.front_reject_enable", front_reject_enable, false);
        this->get_parameter_or<double>("mapping.front_reject_start_time", FRONT_REJECT_START_TIME, 260.0);
        this->get_parameter_or<double>("mapping.front_reject_end_time", FRONT_REJECT_END_TIME, 390.0);
        this->get_parameter_or<bool>("mapping.phased_halfplane_fine_voxel_enable", phased_halfplane_fine_voxel_enable, false);
        this->get_parameter_or<double>("mapping.phased_halfplane_fine_voxel_leaf_size", phased_halfplane_fine_voxel_leaf_size, -1.0);
        this->get_parameter_or<double>("mapping.phased_halfplane_other_leaf_size", phased_halfplane_other_leaf_size, -1.0);
        this->get_parameter_or<double>("mapping.phased_halfplane_back_start_time", phased_halfplane_back_start_time, 20.0);
        this->get_parameter_or<double>("mapping.phased_halfplane_back_end_time", phased_halfplane_back_end_time, 65.0);
        this->get_parameter_or<double>("mapping.phased_halfplane_front_start_time", phased_halfplane_front_start_time, 66.0);
        this->get_parameter_or<double>("mapping.phased_halfplane_front_end_time", phased_halfplane_front_end_time, 87.0);
        this->get_parameter_or<double>("mapping.phased_halfplane_back_resume_start_time", phased_halfplane_back_resume_start_time, 88.0);
        this->get_parameter_or<double>("mapping.phased_halfplane_back_resume_end_time", phased_halfplane_back_resume_end_time, 1.0e9);
        this->get_parameter_or<string>("mapping.phased_halfplane_phase1_target", phased_halfplane_phase1_target, "back");
        this->get_parameter_or<string>("mapping.phased_halfplane_phase2_target", phased_halfplane_phase2_target, "front");
        this->get_parameter_or<string>("mapping.phased_halfplane_phase3_target", phased_halfplane_phase3_target, "back");
        vector<double> phased_halfplane_segments_start_times;
        vector<double> phased_halfplane_segments_end_times;
        vector<string> phased_halfplane_segments_targets;
        this->get_parameter_or<vector<double>>("mapping.phased_halfplane_segments_start_times", phased_halfplane_segments_start_times, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.phased_halfplane_segments_end_times", phased_halfplane_segments_end_times, vector<double>());
        this->get_parameter_or<vector<string>>("mapping.phased_halfplane_segments_targets", phased_halfplane_segments_targets, vector<string>());
        this->get_parameter_or<double>("mapping.voxel_size_normal", voxel_size_normal, -1.0);
        this->get_parameter_or<bool>("mapping.voxel_size_fine_enable", voxel_size_fine_enable, true);
        this->get_parameter_or<double>("mapping.voxel_size_fine", voxel_size_fine, -1.0);
        this->get_parameter_or<double>("mapping.voxel_ramp_duration", voxel_ramp_duration, 0.0);
        this->get_parameter_or<bool>("mapping.windowed_voxel_enable", windowed_voxel_enable, false);
        this->get_parameter_or<double>("mapping.windowed_voxel_start_time", windowed_voxel_start_time, 290.0);
        this->get_parameter_or<double>("mapping.windowed_voxel_end_time", windowed_voxel_end_time, 296.0);
        this->get_parameter_or<double>("mapping.windowed_voxel_leaf_size", windowed_voxel_leaf_size, 0.2);
        this->get_parameter_or<bool>("mapping.blind_window_enable", blind_window_enable, false);
        this->get_parameter_or<double>("mapping.blind_window_start_time", blind_window_start_time, 0.0);
        this->get_parameter_or<double>("mapping.blind_window_end_time", blind_window_end_time, 1.0e9);
        this->get_parameter_or<double>("mapping.blind_window_value", blind_window_value, 0.01);
        this->get_parameter_or<bool>("mapping.imu_only_enable", imu_only_enable, false);
        this->get_parameter_or<vector<double>>("mapping.imu_only_start_times", imu_only_start_times, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.imu_only_end_times", imu_only_end_times, vector<double>());
        if (imu_only_enable && imu_only_start_times.size() != imu_only_end_times.size())
        {
            RCLCPP_WARN(this->get_logger(),
                        "imu_only_start_times/end_times length mismatch (starts=%zu ends=%zu) -- imu-only windows disabled",
                        imu_only_start_times.size(), imu_only_end_times.size());
            imu_only_enable = false;
        }
        this->get_parameter_or<bool>("mapping.imu_trust_window_enable", imu_trust_window_enable, false);
        this->get_parameter_or<double>("mapping.imu_trust_window_start_time", imu_trust_window_start_time, 0.0);
        this->get_parameter_or<double>("mapping.imu_trust_window_end_time", imu_trust_window_end_time, 1.0e9);
        this->get_parameter_or<double>("mapping.imu_trust_window_value", imu_trust_window_value, 1.0);
        this->get_parameter_or<double>("mapping.imu_degraded_dt_threshold_ms", imu_degraded_dt_threshold_ms, 15.0);
        this->get_parameter_or<double>("mapping.lidar_weak_rotation_threshold", lidar_weak_rotation_threshold, -1.0);
        if (lidar_weak_rotation_threshold <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "mapping.lidar_weak_rotation_threshold is unset (<=0): lidar_weak_rotation/combined_risk will be logged as nan. "
                        "Run once to inspect the r_lambda_min column, then set this parameter explicitly.");
        }
        this->get_parameter_or<bool>("mapping.tum_trajectory_log_enable", tum_trajectory_log_enable, false);
        this->get_parameter_or<string>("mapping.tum_trajectory_log_path", tum_trajectory_log_path, root_dir + "/Log/trajectory.tum");
        this->get_parameter_or<bool>("mapping.pose_trajectory_log_enable", pose_trajectory_log_enable, false);
        this->get_parameter_or<string>("mapping.pose_trajectory_log_path", pose_trajectory_log_path, root_dir + "/Log/pose.txt");
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<string>("pcd_save.path", pcd_save_path, root_dir + "/PCD");
        if (pcd_save_en && pcd_save_interval > 0)
        {
            std::error_code pcd_dir_error;
            std::filesystem::create_directories(pcd_save_path, pcd_dir_error);
            if (pcd_dir_error)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Cannot create per-scan PCD directory %s: %s",
                            pcd_save_path.c_str(), pcd_dir_error.message().c_str());
            }
        }
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        rebuild_legacy_fine_voxel_segments();
        if (!phased_halfplane_segments_start_times.empty() ||
            !phased_halfplane_segments_end_times.empty() ||
            !phased_halfplane_segments_targets.empty())
        {
            if (phased_halfplane_segments_start_times.size() == phased_halfplane_segments_end_times.size() &&
                phased_halfplane_segments_start_times.size() == phased_halfplane_segments_targets.size())
            {
                phased_halfplane_segments.clear();
                for (size_t idx = 0; idx < phased_halfplane_segments_targets.size(); idx++)
                {
                    phased_halfplane_segments.push_back(
                        {phased_halfplane_segments_start_times[idx],
                         phased_halfplane_segments_end_times[idx],
                         phased_halfplane_segments_targets[idx]});
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "Ignoring phased_halfplane_segments_* because list lengths differ: starts=%zu ends=%zu targets=%zu",
                            phased_halfplane_segments_start_times.size(),
                            phased_halfplane_segments_end_times.size(),
                            phased_halfplane_segments_targets.size());
            }
        }

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="camera_init";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
        p_imu->imu_trust_scale = imu_trust_scale;
        p_imu->deskew_enable = deskew_enable;

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        if (tum_trajectory_log_enable)
        {
            tum_trajectory_file_.open(tum_trajectory_log_path, ios::out | ios::trunc);
            if (!tum_trajectory_file_.is_open())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to open TUM trajectory file: %s", tum_trajectory_log_path.c_str());
            }
        }
        if (pose_trajectory_log_enable)
        {
            pose_trajectory_file_.open(pose_trajectory_log_path, ios::out | ios::trunc);
            if (!pose_trajectory_file_.is_open())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to open pose trajectory file: %s", pose_trajectory_log_path.c_str());
            }
        }

        /*** ROS subscribe initialization ***/
        sensor_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        processing_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sensor_sub_options;
        sensor_sub_options.callback_group = sensor_callback_group_;

        if (p_pre->lidar_type == AVIA)
        {
            std::string avia_msg_type = detect_avia_topic_type(this, lid_topic);
            if (avia_msg_type.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No publisher detected on '%s' within timeout; defaulting to livox_ros_driver2/msg/CustomMsg.", lid_topic.c_str());
                avia_msg_type = "livox_ros_driver2/msg/CustomMsg";
            }

            if (avia_msg_type == "livox_ros_driver/msg/CustomMsg")
            {
                RCLCPP_INFO(this->get_logger(), "Subscribing to '%s' as legacy livox_ros_driver/msg/CustomMsg.", lid_topic.c_str());
                sub_pcl_livox_legacy_ = this->create_generic_subscription(
                    lid_topic,
                    "livox_ros_driver/msg/CustomMsg",
                    rclcpp::QoS(20),
                    livox_legacy_pcl_cbk,
                    sensor_sub_options);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Subscribing to '%s' as livox_ros_driver2/msg/CustomMsg.", lid_topic.c_str());
                sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk, sensor_sub_options);
            }
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk, sensor_sub_options);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk, sensor_sub_options);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        pubLocalBAPath_ = this->create_publisher<nav_msgs::msg::Path>("/local_ba_path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = this->create_wall_timer(period_ms, std::bind(&LaserMappingNode::timer_callback, this), processing_callback_group_);

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = this->create_wall_timer(map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this), processing_callback_group_);

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "map_save",
            std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            processing_callback_group_);

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        finalize_balm_export();
        // pcd_save.interval > 0 accumulates scans into fixed-size PCD chunks.
        // Flush the final, partial chunk so the last scans are not lost on shutdown.
        flush_scan_pcd();
        // Save the final accumulated world-frame map on normal node shutdown.
        // This is an output-only operation after processing has stopped.
        if (pcd_save_en && pcl_wait_pub != nullptr && !pcl_wait_pub->empty())
        {
            RCLCPP_INFO(this->get_logger(), "Saving final map PCD to %s", map_file_path.c_str());
            save_to_pcd();
        }
        fout_out.close();
        fout_pre.close();
        fclose(fp);
        if (tum_trajectory_file_.is_open())
        {
            tum_trajectory_file_.close();
        }
        if (pose_trajectory_file_.is_open())
        {
            pose_trajectory_file_.close();
        }
    }

private:
    void flush_scan_pcd()
    {
        if (!pcd_save_en || pcd_save_interval <= 0 || pcl_wait_save->empty())
            return;

        ++pcd_index;
        std::ostringstream filename;
        filename << "scan_" << std::setw(6) << std::setfill('0') << pcd_index << ".pcd";
        const std::filesystem::path pcd_path = std::filesystem::path(pcd_save_path) / filename.str();
        pcl::PCDWriter pcd_writer;
        if (pcd_writer.writeBinary(pcd_path.string(), *pcl_wait_save) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Per-scan PCD write failed: %s", pcd_path.c_str());
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Saved %zu points to %s", pcl_wait_save->size(), pcd_path.c_str());
        pcl_wait_save->clear();
        scan_pcd_wait_count_ = 0;
    }

    void save_processed_scan_pcd()
    {
        if (!pcd_save_en || pcd_save_interval <= 0 || feats_undistort->empty())
            return;

        // The scan has already been deskewed.  Transform it using the final EKF
        // state for this LiDAR frame, matching the published world-frame scan.
        PointCloudXYZI world_scan;
        world_scan.reserve(feats_undistort->size());
        for (const auto &point : feats_undistort->points)
        {
            PointType world_point;
            RGBpointBodyToWorld(&point, &world_point);
            world_scan.push_back(world_point);
        }
        *pcl_wait_save += world_scan;
        ++scan_pcd_wait_count_;
        if (scan_pcd_wait_count_ >= pcd_save_interval)
            flush_scan_pcd();
    }

    void timer_callback()
    {
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            if (imu_trust_window_enable)
            {
                const double pre_scan_relative_time = Measures.lidar_beg_time - first_lidar_time;
                p_imu->imu_trust_scale = use_imu_trust_window(pre_scan_relative_time) ? imu_trust_window_value : imu_trust_scale;
            }
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            const double scan_relative_time = Measures.lidar_beg_time - first_lidar_time;
            const double active_surf_leaf_size = use_windowed_voxel_window(scan_relative_time)
                                                      ? windowed_voxel_leaf_size
                                                      : filter_size_surf_min;
            // front/back-target segments keep the pre-existing hard-step half-plane split
            // (only the targeted half-plane's leaf size moves; the other half stays at
            // active_surf_leaf_size, so this can't reuse the single-value "total" ramp
            // below). voxel_size_fine_enable also ramps the targeted half-plane's leaf
            // size in/out over voxel_ramp_duration around [start_time, end_time]; with
            // voxel_size_fine_enable off (or ramp_duration<=0) this collapses back to the
            // exact old hard step at the segment boundary.
            FineVoxelSegment active_halfplane_segment;
            bool has_active_halfplane_segment = false;
            for (const auto &seg : phased_halfplane_segments)
            {
                if (is_total_halfplane_target(seg.target))
                {
                    continue;
                }
                const double margin = voxel_size_fine_enable ? voxel_ramp_duration : 0.0;
                if (scan_relative_time >= seg.start_time - margin && scan_relative_time <= seg.end_time + margin)
                {
                    active_halfplane_segment = seg;
                    has_active_halfplane_segment = true;
                }
            }

            if (has_active_halfplane_segment)
            {
                const double halfplane_fine_target = resolve_fine_voxel_leaf_size(active_surf_leaf_size);
                double fine_leaf_size = halfplane_fine_target;
                double base_leaf_size = active_surf_leaf_size;
                if (voxel_size_fine_enable)
                {
                    const double voxel_normal_resolved = (voxel_size_normal > 0.0) ? voxel_size_normal : active_surf_leaf_size;
                    const double voxel_fine_resolved = (voxel_size_fine > 0.0) ? voxel_size_fine : resolve_fine_voxel_leaf_size(active_surf_leaf_size);
                    // The untargeted half-plane inherits any adjacent/overlapping "total"
                    // segment's ramp value (e.g. a total segment ending right where this
                    // front/back segment begins) instead of jumping straight to
                    // active_surf_leaf_size, so a total->half-plane handoff is one smooth
                    // transition rather than a hard step. With no such total segment nearby
                    // this just evaluates to voxel_normal_resolved, unchanged from before.
                    const double total_background = compute_voxel_size(scan_relative_time, phased_halfplane_segments,
                                                                        voxel_normal_resolved, voxel_fine_resolved, voxel_ramp_duration);
                    const double own_ramp = compute_voxel_size(scan_relative_time,
                                                               active_halfplane_segment.start_time, active_halfplane_segment.end_time,
                                                               voxel_normal_resolved, halfplane_fine_target, voxel_ramp_duration);
                    // Symmetrically, the targeted half-plane also can't un-fine below what
                    // an overlapping total segment demands, so the transition stays
                    // continuous at both ends of the handoff (see own_ramp vs total_background).
                    fine_leaf_size = std::min(own_ramp, total_background);

                    base_leaf_size = total_background;
                    if (phased_halfplane_other_leaf_size > 0.0)
                    {
                        // The untargeted half-plane can also be given its own explicit
                        // target (independent of voxel_size_normal), ramping toward it the
                        // same way the targeted half-plane ramps toward halfplane_fine_target.
                        const double own_other_ramp = compute_voxel_size(scan_relative_time,
                                                                         active_halfplane_segment.start_time, active_halfplane_segment.end_time,
                                                                         voxel_normal_resolved, phased_halfplane_other_leaf_size, voxel_ramp_duration);
                        base_leaf_size = std::min(own_other_ramp, total_background);
                    }
                }
                downsample_target_halfplane_with_fine_voxel(
                    feats_undistort,
                    feats_down_body,
                    base_leaf_size,
                    fine_leaf_size,
                    is_front_halfplane_target(active_halfplane_segment.target));
            }
            else
            {
                const double voxel_normal_resolved = (voxel_size_normal > 0.0) ? voxel_size_normal : active_surf_leaf_size;
                double voxel_size_now = voxel_normal_resolved;
                if (voxel_size_fine_enable)
                {
                    const double voxel_fine_resolved = (voxel_size_fine > 0.0) ? voxel_size_fine : resolve_fine_voxel_leaf_size(active_surf_leaf_size);
                    voxel_size_now = compute_voxel_size(scan_relative_time, phased_halfplane_segments,
                                                        voxel_normal_resolved, voxel_fine_resolved, voxel_ramp_duration);
                }
                // else: "total"-target segments have no effect; voxel size stays voxel_normal_resolved
                downSizeFilterSurf.setLeafSize(voxel_size_now, voxel_size_now, voxel_size_now);
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                // VoxelSLAM re-downsamples at half the voxel size whenever the first
                // pass leaves fewer than 500 points (voxelslam.cpp L1629-1633), so a
                // sparse-feature moment (doorway threshold, low-clearance corridor)
                // still gets a floor on point density. Stock FAST-LIO2 has no such
                // fallback -- it just keeps whatever the fixed-size grid produced.
                if (feats_down_body->points.size() < 500)
                {
                    downSizeFilterSurf.setLeafSize(voxel_size_now / 2, voxel_size_now / 2, voxel_size_now / 2);
                    downSizeFilterSurf.setInputCloud(feats_undistort);
                    downSizeFilterSurf.filter(*feats_down_body);
                }
            }
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            latest_degeneracy_result = DegeneracyResult{};
            if (!in_imu_only_window(scan_relative_time))
            {
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            }
            // else: skip the LiDAR measurement update entirely for this scan -- the
            // state stays exactly at the IMU-only forward-propagated value already in kf.
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            update_degeneracy_state();
            {
                static std::ofstream degeneracy_ofs;
                static bool degeneracy_ofs_tried = false;
                if (!degeneracy_ofs_tried)
                {
                    degeneracy_ofs_tried = true;
                    const char *dir = std::getenv("FASTLIO_DEGENERACY_LOG_DIR");
                    std::string path = (dir ? std::string(dir) : std::string("/tmp")) + "/fastlio2_degeneracy.csv";
                    degeneracy_ofs.open(path);
                    if (degeneracy_ofs.is_open())
                        degeneracy_ofs << "t,valid_plane_num,lambda_min,lambda_min_norm,is_degenerate_voxelslam,persistent_degenerate,early_warning\n";
                }
                if (degeneracy_ofs.is_open())
                {
                    degeneracy_ofs << std::fixed << std::setprecision(6) << Measures.lidar_beg_time << ","
                                   << latest_degeneracy_result.valid_plane_num << ","
                                   << std::setprecision(6) << latest_degeneracy_result.lambda_min << ","
                                   << latest_degeneracy_result.lambda_min_norm << ","
                                   << (latest_degeneracy_result.is_degenerate_voxelslam ? 1 : 0) << ","
                                   << (latest_degeneracy_result.persistent_degenerate ? 1 : 0) << ","
                                   << (latest_degeneracy_result.early_warning ? 1 : 0) << "\n";
                    degeneracy_ofs.flush();
                }
            }
            if (g_use_voxel_matching)
            {
                // LIVE integration: the voxel map was already used for real
                // matching inside h_share_model (gated by g_use_voxel_matching
                // there too). Here we just grow it, using FAST-LIO2's own
                // final converged pose for this scan (not GT) -- this is the
                // actual closed-loop system now, same as VoxelSLAM's own
                // odom_only would build its map from its own estimate.
                if (!g_real_vp_inited) init_real_vp();

                vp::PVecPtr pptr(new vp::PVec);
                pptr->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++)
                {
                    const PointType &pi = feats_down_body->points[i];
                    Eigen::Vector3d p_lidar(pi.x, pi.y, pi.z);
                    Eigen::Matrix3d pvar;
                    Eigen::Vector3d p_lidar_for_var = p_lidar;
                    calc_body_var_port(p_lidar_for_var, 0.02, 0.05, pvar);
                    Eigen::Matrix3d Rli = state_point.offset_R_L_I.matrix();
                    (*pptr)[i].pnt = Rli * p_lidar + state_point.offset_T_L_I;
                    (*pptr)[i].var = Rli * pvar * Rli.transpose();
                }

                vp::IMUST xcurr;
                xcurr.t = Measures.lidar_beg_time;
                xcurr.R = state_point.rot.matrix();
                xcurr.p = state_point.pos;
                xcurr.v.setZero(); xcurr.bg.setZero(); xcurr.ba.setZero();
                xcurr.g = Eigen::Vector3d(0, 0, -9.81);

                g_real_win_count++;
                g_real_xbuf.push_back(xcurr);
                g_real_pvecbuf.push_back(pptr);

                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pwld(pptr->size());
                for (size_t i = 0; i < pptr->size(); i++)
                    pwld[i] = xcurr.R * (*pptr)[i].pnt + xcurr.p;

                std::unordered_map<vp::VOXEL_LOC, vp::OctoTree *> touched;
                vp::cut_voxel_multi(g_real_surf_map, g_real_pvecbuf.back(), g_real_win_count - 1,
                                    touched, g_real_win_size, pwld, g_real_sws);

                for (auto &kv : touched) kv.second->recut(g_real_win_count, g_real_xbuf, g_real_sws[0]);

                if (g_real_win_count >= g_real_win_size)
                {
                    for (auto &kv : touched) kv.second->margi(g_real_win_count, 1, g_real_xbuf, *g_real_voxhess);
                    g_real_xbuf.erase(g_real_xbuf.begin());
                    g_real_pvecbuf.erase(g_real_pvecbuf.begin());
                    for (int i = 0; i < g_real_win_size; i++) { vp::mp[i]++; if (vp::mp[i] >= g_real_win_size) vp::mp[i] = 0; }
                    g_real_win_count--;
                }
            }
            else
            {
                // Stage-0b shadow comparison: same adaptive voxel-plane matching,
                // but points are placed in world frame using GT pose instead of
                // FAST-LIO2's own (possibly drifting) state_point, so the
                // matching algorithm is evaluated in isolation from FAST-LIO2's
                // own pose error. Does NOT touch state_point/kf in any way.
                if (!g_gt_loaded)
                {
                    g_gt_loaded = true;
                    const char *gt_path = std::getenv("SHADOW_GT_TRAJ_PATH");
                    if (gt_path) load_gt_poses(gt_path);
                    else std::cerr << "[shadow-gt] SHADOW_GT_TRAJ_PATH not set; shadow-gt disabled" << std::endl;
                }

                Eigen::Matrix3d shadow_normal_matrix = Eigen::Matrix3d::Zero();
                int shadow_valid = 0;
                bool have_gt = false;
                std::vector<Eigen::Vector3d> shadow_pts_world;
                shadow_pts_world.reserve(feats_down_size);

                Eigen::Vector3d gt_p; Eigen::Matrix3d gt_R;
                have_gt = lookup_gt_pose(Measures.lidar_beg_time, gt_p, gt_R);

                if (have_gt)
                {
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        const PointType &pi = feats_down_body->points[i];
                        Eigen::Vector3d p_body(pi.x, pi.y, pi.z);
                        Eigen::Vector3d pw = gt_R * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + gt_p;
                        shadow_pts_world.push_back(pw);
                        Eigen::Vector3d nrm;
                        if (g_shadow_voxel_map.query(pw, nrm))
                        {
                            shadow_normal_matrix.noalias() += nrm * nrm.transpose();
                            shadow_valid++;
                        }
                    }
                }
                double shadow_lambda_min = 0.0;
                if (shadow_valid > 0)
                {
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(shadow_normal_matrix);
                    if (es.info() == Eigen::Success) shadow_lambda_min = es.eigenvalues()(0);
                }
                for (const auto &pw : shadow_pts_world) g_shadow_voxel_map.insert(pw);

                static std::ofstream shadow_ofs;
                static bool shadow_ofs_tried = false;
                if (!shadow_ofs_tried)
                {
                    shadow_ofs_tried = true;
                    const char *dir = std::getenv("FASTLIO_DEGENERACY_LOG_DIR");
                    std::string path = (dir ? std::string(dir) : std::string("/tmp")) + "/fastlio2_shadow_voxel_gtpose.csv";
                    shadow_ofs.open(path);
                    if (shadow_ofs.is_open())
                        shadow_ofs << "t,have_gt,shadow_valid_plane_num,shadow_lambda_min,shadow_is_degenerate\n";
                }
                if (shadow_ofs.is_open())
                {
                    shadow_ofs << std::fixed << std::setprecision(6) << Measures.lidar_beg_time << ","
                               << (have_gt ? 1 : 0) << ","
                               << shadow_valid << "," << shadow_lambda_min << ","
                               << (shadow_lambda_min < DEGENERACY_LAMBDA_MIN_THRESHOLD ? 1 : 0) << "\n";
                    shadow_ofs.flush();
                }
            }
            if (!g_use_voxel_matching)
            {
                // Stage-0c: real VoxelSLAM OctoTree matching, GT-pose driven.
                if (!g_real_vp_inited) init_real_vp();
                Eigen::Vector3d gt_p2; Eigen::Matrix3d gt_R2;
                if (lookup_gt_pose(Measures.lidar_beg_time, gt_p2, gt_R2))
                {
                    vp::PVecPtr pptr(new vp::PVec);
                    pptr->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        const PointType &pi = feats_down_body->points[i];
                        Eigen::Vector3d p_lidar(pi.x, pi.y, pi.z);
                        Eigen::Matrix3d pvar;
                        Eigen::Vector3d p_lidar_for_var = p_lidar;
                        calc_body_var_port(p_lidar_for_var, 0.02, 0.05, pvar);
                        Eigen::Matrix3d Rli = state_point.offset_R_L_I.matrix();
                        (*pptr)[i].pnt = Rli * p_lidar + state_point.offset_T_L_I;
                        (*pptr)[i].var = Rli * pvar * Rli.transpose();
                    }

                    // 1) match against the EXISTING map (built from past scans only)
                    Eigen::Matrix3d real_normal_matrix = Eigen::Matrix3d::Zero();
                    int real_valid = 0;
                    for (auto &pv : *pptr)
                    {
                        Eigen::Vector3d wld = gt_R2 * pv.pnt + gt_p2;
                        vp::Plane *pla = nullptr;
                        double sigma_d = 0;
                        Eigen::Matrix3d var_world = gt_R2 * pv.var * gt_R2.transpose();
                        vp::OctoTree *oc = nullptr;
                        int flag = vp::match(g_real_surf_map, wld, pla, var_world, sigma_d, oc);
                        if (flag && pla)
                        {
                            real_normal_matrix.noalias() += pla->normal * pla->normal.transpose();
                            real_valid++;
                        }
                    }
                    double real_lambda_min = 0.0;
                    if (real_valid > 0)
                    {
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(real_normal_matrix);
                        if (es.info() == Eigen::Success) real_lambda_min = es.eigenvalues()(0);
                    }

                    // 2) insert this scan into the windowed map (grows it for future scans)
                    vp::IMUST xcurr;
                    xcurr.t = Measures.lidar_beg_time;
                    xcurr.R = gt_R2; xcurr.p = gt_p2;
                    xcurr.v.setZero(); xcurr.bg.setZero(); xcurr.ba.setZero();
                    xcurr.g = Eigen::Vector3d(0, 0, -9.81);

                    g_real_win_count++;
                    g_real_xbuf.push_back(xcurr);
                    g_real_pvecbuf.push_back(pptr);

                    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pwld(pptr->size());
                    for (size_t i = 0; i < pptr->size(); i++)
                        pwld[i] = xcurr.R * (*pptr)[i].pnt + xcurr.p;

                    std::unordered_map<vp::VOXEL_LOC, vp::OctoTree *> touched;
                    vp::cut_voxel_multi(g_real_surf_map, g_real_pvecbuf.back(), g_real_win_count - 1,
                                        touched, g_real_win_size, pwld, g_real_sws);

                    for (auto &kv : touched) kv.second->recut(g_real_win_count, g_real_xbuf, g_real_sws[0]);

                    if (g_real_win_count >= g_real_win_size)
                    {
                        for (auto &kv : touched) kv.second->margi(g_real_win_count, 1, g_real_xbuf, *g_real_voxhess);
                        g_real_xbuf.erase(g_real_xbuf.begin());
                        g_real_pvecbuf.erase(g_real_pvecbuf.begin());
                        for (int i = 0; i < g_real_win_size; i++) { vp::mp[i]++; if (vp::mp[i] >= g_real_win_size) vp::mp[i] = 0; }
                        g_real_win_count--;
                    }

                    static std::ofstream real_ofs;
                    static bool real_ofs_tried = false;
                    if (!real_ofs_tried)
                    {
                        real_ofs_tried = true;
                        const char *dir = std::getenv("FASTLIO_DEGENERACY_LOG_DIR");
                        std::string path = (dir ? std::string(dir) : std::string("/tmp")) + "/fastlio2_real_vslam_gtpose.csv";
                        real_ofs.open(path);
                        if (real_ofs.is_open())
                            real_ofs << "t,real_valid_plane_num,real_lambda_min,real_is_degenerate\n";
                    }
                    if (real_ofs.is_open())
                    {
                        real_ofs << std::fixed << std::setprecision(6) << Measures.lidar_beg_time << ","
                                 << real_valid << "," << real_lambda_min << ","
                                 << (real_lambda_min < DEGENERACY_LAMBDA_MIN_THRESHOLD ? 1 : 0) << "\n";
                        real_ofs.flush();
                    }
                }
            }
            // h_share_model() may run multiple times in the iterated update;
            // write the diagnostic only here, after the final EKF state is set
            // and immediately before any map insertion.
            LocalBAResult local_ba_result;
            if (local_ba_enable)
            {
                local_ba_result = process_local_ba();
            }

            export_balm_scan_if_requested();



            // Read the final state once, after the iterative EKF update.  These
            // streams are write-only diagnostics: no state, covariance, or map
            // value is changed by trajectory logging.
            const Eigen::Quaterniond final_state_q(state_point.rot.matrix());
            const double elapsed_time = Measures.lidar_beg_time - first_lidar_time;
            if (pose_trajectory_file_.is_open())
            {
                // pose.txt: timestamp elapsed_time px py pz qx qy qz qw
                pose_trajectory_file_ << fixed << setprecision(9)
                                      << Measures.lidar_beg_time << " "
                                      << elapsed_time << " "
                                      << state_point.pos(0) << " "
                                      << state_point.pos(1) << " "
                                      << state_point.pos(2) << " "
                                      << final_state_q.x() << " "
                                      << final_state_q.y() << " "
                                      << final_state_q.z() << " "
                                      << final_state_q.w() << "\n";
            }
            if (tum_trajectory_file_.is_open())
            {
                // Standard TUM: timestamp px py pz qx qy qz qw.
                tum_trajectory_file_ << fixed << setprecision(9)
                                     << Measures.lidar_beg_time << " "
                                     << state_point.pos(0) << " "
                                     << state_point.pos(1) << " "
                                     << state_point.pos(2) << " "
                                     << final_state_q.x() << " "
                                     << final_state_q.y() << " "
                                     << final_state_q.z() << " "
                                     << final_state_q.w() << "\n";
            }
            save_processed_scan_pcd();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            // Runtime timings can be skewed by host load during ros2 bag play; keep them as diagnostics only.
            map_incremental();
            t5 = omp_get_wtime();

            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    static double local_ba_rmse(const deque<LocalBAFrame> &frames,
                                const vector<Eigen::Quaterniond> &quaternions,
                                const vector<Eigen::Vector3d> &translations)
    {
        double squared_error_sum = 0.0;
        size_t count = 0;
        for (size_t frame_index = 0; frame_index < frames.size(); frame_index++)
        {
            for (const PlaneConstraint &constraint : frames[frame_index].constraints)
            {
                const double residual = constraint.plane_normal.dot(
                    quaternions[frame_index] * constraint.point_body + translations[frame_index]) + constraint.plane_d;
                squared_error_sum += residual * residual;
                count++;
            }
        }
        return count == 0 ? std::numeric_limits<double>::quiet_NaN() :
                            std::sqrt(squared_error_sum / static_cast<double>(count));
    }

    static LocalBAImuSample interpolate_imu(const LocalBAImuSample &a,
                                              const LocalBAImuSample &b, double timestamp)
    {
        const double alpha = (timestamp - a.timestamp) / std::max(1e-12, b.timestamp - a.timestamp);
        LocalBAImuSample out;
        out.timestamp = timestamp;
        out.gyro = (1.0 - alpha) * a.gyro + alpha * b.gyro;
        out.accel = (1.0 - alpha) * a.accel + alpha * b.accel;
        return out;
    }

    static bool make_imu_preintegration(const LocalBAFrame &from, const LocalBAFrame &to,
                                        IMUPreintegration &out)
    {
        if (to.timestamp <= from.timestamp || to.imu_samples.size() < 2)
        {
            return false;
        }
        const double begin = from.timestamp;
        const double end = to.timestamp;
        out = IMUPreintegration{};
        out.delta_q = Eigen::Quaterniond::Identity();
        out.from_frame = from.frame_id;
        out.to_frame = to.frame_id;
        out.accel_scale = from.accel_scale;
        out.accel_bias_norm = from.accel_bias.norm();
        Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_scaled_accel = Eigen::Vector3d::Zero();
        double last_stamp = begin;
        bool have_input = false;

        // This deliberately mirrors ImuProcess::UndistortPcl(): the saved
        // previous IMU sample is paired with the current sample, midpoint
        // averaging is used, and a pair crossing scan start is time-clipped.
        for (size_t i = 1; i < to.imu_samples.size(); ++i)
        {
            const LocalBAImuSample &head = to.imu_samples[i - 1];
            const LocalBAImuSample &tail = to.imu_samples[i];
            if (tail.timestamp < begin)
            {
                continue;
            }
            if (head.timestamp >= end)
            {
                break;
            }
            const double integration_end = std::min(tail.timestamp, end);
            const double dt = head.timestamp < begin ? integration_end - begin : integration_end - head.timestamp;
            if (dt <= 0.0)
            {
                continue;
            }
            const Eigen::Vector3d raw_accel = 0.5 * (head.accel + tail.accel);
            const Eigen::Vector3d scaled_accel = raw_accel * from.accel_scale;
            const Eigen::Vector3d gyro = 0.5 * (head.gyro + tail.gyro) - from.gyro_bias;
            const Eigen::Vector3d accel = scaled_accel - from.accel_bias;
            const Eigen::Vector3d accel_i = out.delta_q * accel;
            out.delta_p += out.delta_v * dt + 0.5 * accel_i * dt * dt;
            out.delta_v += accel_i * dt;
            const double angle = gyro.norm() * dt;
            Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
            if (angle > 1e-12)
            {
                dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, gyro.normalized()));
            }
            out.delta_q = (out.delta_q * dq).normalized();
            out.dt += dt;
            out.raw_accel_norm += raw_accel.norm();
            out.scaled_accel_norm += scaled_accel.norm();
            ++out.imu_sample_count;
            last_gyro = gyro;
            last_scaled_accel = scaled_accel;
            last_stamp = integration_end;
            have_input = true;
            if (tail.timestamp >= end)
            {
                break;
            }
        }
        // FAST-LIO predicts the scan-end remainder with the last midpoint
        // input.  Do exactly the same instead of creating a new scale value.
        if (have_input && last_stamp < end)
        {
            const double dt = end - last_stamp;
            const Eigen::Vector3d accel_i = out.delta_q * (last_scaled_accel - from.accel_bias);
            out.delta_p += out.delta_v * dt + 0.5 * accel_i * dt * dt;
            out.delta_v += accel_i * dt;
            const double angle = last_gyro.norm() * dt;
            if (angle > 1e-12)
            {
                out.delta_q = (out.delta_q * Eigen::Quaterniond(Eigen::AngleAxisd(angle, last_gyro.normalized()))).normalized();
            }
            out.dt += dt;
        }
        if (out.imu_sample_count > 0)
        {
            out.raw_accel_norm /= out.imu_sample_count;
            out.scaled_accel_norm /= out.imu_sample_count;
        }
        return out.dt > 1e-4;
    }

    static vector<IMUPreintegration> build_imu_factors(const deque<LocalBAFrame> &frames)
    {
        vector<IMUPreintegration> factors;
        for (size_t i = 1; i < frames.size(); ++i)
        {
            IMUPreintegration preint;
            if (make_imu_preintegration(frames[i - 1], frames[i], preint))
            {
                factors.push_back(preint);
            }
        }
        return factors;
    }

    struct ImuRmse { double rot = std::numeric_limits<double>::quiet_NaN(); double vel = std::numeric_limits<double>::quiet_NaN(); double pos = std::numeric_limits<double>::quiet_NaN(); };

    static ImuRmse local_ba_imu_rmse(const deque<LocalBAFrame> &frames,
                                      const vector<IMUPreintegration> &factors,
                                      const vector<Eigen::Quaterniond> &q,
                                      const vector<Eigen::Vector3d> &p,
                                      const vector<Eigen::Vector3d> &v)
    {
        ImuRmse rmse;
        double sum_rot = 0.0, sum_vel = 0.0, sum_pos = 0.0;
        int count = 0;
        for (const IMUPreintegration &f : factors)
        {
            const size_t i = static_cast<size_t>(f.from_frame - frames.front().frame_id);
            const size_t j = static_cast<size_t>(f.to_frame - frames.front().frame_id);
            if (i >= q.size() || j >= q.size()) continue;
            const Eigen::Quaterniond q_err = f.delta_q.conjugate() * (q[i].conjugate() * q[j]);
            const double rot = Eigen::AngleAxisd(q_err.normalized()).angle();
            const Eigen::Vector3d vel = q[i].conjugate() * (v[j] - v[i] - frames[i].gravity * f.dt) - f.delta_v;
            const Eigen::Vector3d pos = q[i].conjugate() * (p[j] - p[i] - v[i] * f.dt - 0.5 * frames[i].gravity * f.dt * f.dt) - f.delta_p;
            sum_rot += rot * rot;
            sum_vel += vel.squaredNorm();
            sum_pos += pos.squaredNorm();
            ++count;
        }
        if (count > 0)
        {
            rmse.rot = std::sqrt(sum_rot / (3.0 * count));
            rmse.vel = std::sqrt(sum_vel / (3.0 * count));
            rmse.pos = std::sqrt(sum_pos / (3.0 * count));
        }
        return rmse;
    }

    static Eigen::Vector3d local_ba_preint_residual_norms(const LocalBAFrame &from,
                                                            const IMUPreintegration &f,
                                                            const Eigen::Quaterniond &q_i,
                                                            const Eigen::Vector3d &p_i,
                                                            const Eigen::Vector3d &v_i,
                                                            const Eigen::Quaterniond &q_j,
                                                            const Eigen::Vector3d &p_j,
                                                            const Eigen::Vector3d &v_j)
    {
        const Eigen::Quaterniond q_err = f.delta_q.conjugate() * (q_i.conjugate() * q_j);
        const double r_rot = Eigen::AngleAxisd(q_err.normalized()).angle();
        const double r_vel = (q_i.conjugate() * (v_j - v_i - from.gravity * f.dt) - f.delta_v).norm();
        const double r_pos = (q_i.conjugate() * (p_j - p_i - v_i * f.dt - 0.5 * from.gravity * f.dt * f.dt) - f.delta_p).norm();
        return Eigen::Vector3d(r_rot, r_vel, r_pos);
    }

    void capture_local_ba_frame()
    {
        LocalBAFrame frame;
        frame.frame_id = local_ba_next_frame_id_;
        // The propagated/filter state is at the scan end; use the same time
        // boundary for the following scan-to-scan IMU preintegration factor.
        frame.timestamp = Measures.lidar_end_time;
        frame.q = Eigen::Quaterniond(state_point.rot.matrix()).normalized();
        frame.t = state_point.pos;
        frame.v = state_point.vel;
        frame.gyro_bias = state_point.bg;
        frame.accel_bias = state_point.ba;
        frame.gravity = state_point.grav;
        frame.accel_scale = p_imu->acceleration_scale();
        if (local_ba_have_last_imu_sample_)
        {
            frame.imu_samples.push_back(local_ba_last_imu_sample_);
        }
        for (const auto &imu : Measures.imu)
        {
            LocalBAImuSample sample;
            sample.timestamp = get_time_sec(imu->header.stamp);
            sample.gyro = Eigen::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
            sample.accel = Eigen::Vector3d(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
            if (frame.imu_samples.empty() || sample.timestamp > frame.imu_samples.back().timestamp)
            {
                frame.imu_samples.push_back(sample);
            }
        }
        if (!frame.imu_samples.empty())
        {
            local_ba_last_imu_sample_ = frame.imu_samples.back();
            local_ba_have_last_imu_sample_ = true;
        }
        frame.scan.reset(new PointCloudXYZI(*feats_down_body));
        frame.constraints = latest_local_ba_constraints;
        for (PlaneConstraint &constraint : frame.constraints)
        {
            constraint.frame_id = local_ba_next_frame_id_;
        }
        local_ba_next_frame_id_++;
        local_ba_frames_.push_back(std::move(frame));
        while (static_cast<int>(local_ba_frames_.size()) > local_ba_window_size)
        {
            local_ba_frames_.pop_front();
        }
    }

    LocalBAResult run_local_ba()
    {
        LocalBAResult result;
        result.window_size = static_cast<int>(local_ba_frames_.size());
        if (local_ba_frames_.empty())
        {
            return result;
        }

        const LocalBAFrame &latest_frame = local_ba_frames_.back();
        result.optimized_q = latest_frame.q;
        result.optimized_t = latest_frame.t;
        result.optimized_v = latest_frame.v;
        result.accel_scale = latest_frame.accel_scale;
        for (const LocalBAFrame &frame : local_ba_frames_)
        {
            result.num_plane_constraints += static_cast<int>(frame.constraints.size());
        }
        if (local_ba_frames_.size() < 2 || result.num_plane_constraints < local_ba_min_constraints)
        {
            return result;
        }
        result.ba_executed = true;

        vector<array<double, 4>> quaternion_parameters(local_ba_frames_.size());
        vector<array<double, 3>> translation_parameters(local_ba_frames_.size());
        vector<array<double, 3>> velocity_parameters(local_ba_frames_.size());
        vector<Eigen::Quaterniond> original_quaternions;
        vector<Eigen::Vector3d> original_translations;
        vector<Eigen::Vector3d> original_velocities;
        original_quaternions.reserve(local_ba_frames_.size());
        original_translations.reserve(local_ba_frames_.size());
        original_velocities.reserve(local_ba_frames_.size());
        for (size_t i = 0; i < local_ba_frames_.size(); i++)
        {
            const LocalBAFrame &frame = local_ba_frames_[i];
            quaternion_parameters[i] = {{frame.q.x(), frame.q.y(), frame.q.z(), frame.q.w()}};
            translation_parameters[i] = {{frame.t.x(), frame.t.y(), frame.t.z()}};
            velocity_parameters[i] = {{frame.v.x(), frame.v.y(), frame.v.z()}};
            original_quaternions.push_back(frame.q);
            original_translations.push_back(frame.t);
            original_velocities.push_back(frame.v);
        }
        result.lidar_rmse_before = local_ba_rmse(local_ba_frames_, original_quaternions, original_translations);
        const vector<IMUPreintegration> imu_factors = local_ba_imu_enable ? build_imu_factors(local_ba_frames_) : vector<IMUPreintegration>();
        result.imu_factor_count = static_cast<int>(imu_factors.size());
        if (!imu_factors.empty())
        {
            const ImuRmse imu_before = local_ba_imu_rmse(local_ba_frames_, imu_factors, original_quaternions, original_translations, original_velocities);
            result.imu_rot_rmse_before = imu_before.rot;
            result.imu_vel_rmse_before = imu_before.vel;
            result.imu_pos_rmse_before = imu_before.pos;
            const IMUPreintegration &latest_preint = imu_factors.back();
            const size_t i = static_cast<size_t>(latest_preint.from_frame - local_ba_frames_.front().frame_id);
            const size_t j = static_cast<size_t>(latest_preint.to_frame - local_ba_frames_.front().frame_id);
            const Eigen::Vector3d sanity = local_ba_preint_residual_norms(
                local_ba_frames_[i], latest_preint, original_quaternions[i], original_translations[i], original_velocities[i],
                original_quaternions[j], original_translations[j], original_velocities[j]);
            result.preint_rot_residual_before = sanity.x();
            result.preint_vel_residual_before = sanity.y();
            result.preint_pos_residual_before = sanity.z();
            result.accel_scale = latest_preint.accel_scale;
            result.raw_accel_norm = latest_preint.raw_accel_norm;
            result.scaled_accel_norm = latest_preint.scaled_accel_norm;
            result.accel_bias_norm = latest_preint.accel_bias_norm;
            result.delta_v_norm = latest_preint.delta_v.norm();
            result.delta_p_norm = latest_preint.delta_p.norm();
            result.imu_dt = latest_preint.dt;
            result.imu_sample_count = latest_preint.imu_sample_count;
        }

        ceres::Problem problem;
        for (size_t i = 0; i < local_ba_frames_.size(); i++)
        {
            problem.AddParameterBlock(quaternion_parameters[i].data(), 4,
                                      new ceres::EigenQuaternionParameterization());
            problem.AddParameterBlock(translation_parameters[i].data(), 3);
            problem.AddParameterBlock(velocity_parameters[i].data(), 3);
        }
        // Fix the oldest pose to remove the pose-only window's gauge freedom.
        problem.SetParameterBlockConstant(quaternion_parameters.front().data());
        problem.SetParameterBlockConstant(translation_parameters.front().data());
        problem.SetParameterBlockConstant(velocity_parameters.front().data());

        for (size_t i = 0; i < local_ba_frames_.size(); i++)
        {
            for (const PlaneConstraint &constraint : local_ba_frames_[i].constraints)
            {
                auto *cost = new ceres::AutoDiffCostFunction<PointToPlaneCost, 1, 4, 3>(
                    new PointToPlaneCost(constraint.point_body, constraint.plane_normal, constraint.plane_d));
                problem.AddResidualBlock(cost, new ceres::HuberLoss(local_ba_huber_delta),
                                         quaternion_parameters[i].data(), translation_parameters[i].data());
            }
        }
        for (const IMUPreintegration &preint : imu_factors)
        {
            const size_t i = static_cast<size_t>(preint.from_frame - local_ba_frames_.front().frame_id);
            const size_t j = static_cast<size_t>(preint.to_frame - local_ba_frames_.front().frame_id);
            if (i >= local_ba_frames_.size() || j >= local_ba_frames_.size()) continue;
            auto *cost = new ceres::AutoDiffCostFunction<IMUPreintegrationCost, 9, 4, 3, 3, 4, 3, 3>(
                new IMUPreintegrationCost(preint, local_ba_frames_[i].gravity,
                                          local_ba_imu_rot_weight, local_ba_imu_vel_weight, local_ba_imu_pos_weight));
            problem.AddResidualBlock(cost, nullptr,
                                     quaternion_parameters[i].data(), translation_parameters[i].data(), velocity_parameters[i].data(),
                                     quaternion_parameters[j].data(), translation_parameters[j].data(), velocity_parameters[j].data());
        }

        ceres::Problem::EvaluateOptions evaluate_options;
        problem.Evaluate(evaluate_options, &result.cost_before, nullptr, nullptr, nullptr);
        ceres::Solver::Options options;
        options.max_num_iterations = local_ba_max_iterations;
        options.linear_solver_type = ceres::DENSE_QR;
        options.num_threads = 1;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        result.ba_converged = summary.IsSolutionUsable();
        problem.Evaluate(evaluate_options, &result.cost_after, nullptr, nullptr, nullptr);

        vector<Eigen::Quaterniond> optimized_quaternions;
        vector<Eigen::Vector3d> optimized_translations;
        vector<Eigen::Vector3d> optimized_velocities;
        optimized_quaternions.reserve(local_ba_frames_.size());
        optimized_translations.reserve(local_ba_frames_.size());
        optimized_velocities.reserve(local_ba_frames_.size());
        for (size_t i = 0; i < local_ba_frames_.size(); i++)
        {
            optimized_quaternions.emplace_back(quaternion_parameters[i][3], quaternion_parameters[i][0],
                                                quaternion_parameters[i][1], quaternion_parameters[i][2]);
            optimized_quaternions.back().normalize();
            optimized_translations.emplace_back(translation_parameters[i][0], translation_parameters[i][1],
                                                 translation_parameters[i][2]);
            optimized_velocities.emplace_back(velocity_parameters[i][0], velocity_parameters[i][1], velocity_parameters[i][2]);
        }
        result.lidar_rmse_after = local_ba_rmse(local_ba_frames_, optimized_quaternions, optimized_translations);
        if (!imu_factors.empty())
        {
            const ImuRmse imu_after = local_ba_imu_rmse(local_ba_frames_, imu_factors, optimized_quaternions, optimized_translations, optimized_velocities);
            result.imu_rot_rmse_after = imu_after.rot;
            result.imu_vel_rmse_after = imu_after.vel;
            result.imu_pos_rmse_after = imu_after.pos;
        }
        result.optimized_q = optimized_quaternions.back();
        result.optimized_t = optimized_translations.back();
        result.optimized_v = optimized_velocities.back();
        result.latest_delta_translation = (result.optimized_t - latest_frame.t).norm();
        const Eigen::Quaterniond delta_q = latest_frame.q.conjugate() * result.optimized_q;
        result.latest_delta_rotation_deg = Eigen::AngleAxisd(delta_q.normalized()).angle() * 180.0 / M_PI;
        result.ba_translation_correction = result.latest_delta_translation;
        result.ba_rotation_correction_deg = result.latest_delta_rotation_deg;
        result.ba_velocity_correction = (result.optimized_v - latest_frame.v).norm();

        const bool reject = !result.ba_converged ||
                            !std::isfinite(result.cost_before) || !std::isfinite(result.cost_after) ||
                            !std::isfinite(result.lidar_rmse_before) || !std::isfinite(result.lidar_rmse_after) ||
                            result.cost_after >= result.cost_before ||
                            result.lidar_rmse_after > result.lidar_rmse_before ||
                            result.latest_delta_translation > local_ba_max_translation_correction ||
                            result.latest_delta_rotation_deg > local_ba_max_rotation_correction_deg;
        if (reject)
        {
            result.optimized_q = latest_frame.q;
            result.optimized_t = latest_frame.t;
            result.optimized_v = latest_frame.v;
            return result;
        }
        result.ba_accepted = true;
        return result;
    }

    void publish_local_ba_path(const LocalBAResult &result)
    {
        geometry_msgs::msg::PoseStamped optimized_pose;
        optimized_pose.header.stamp = get_ros_time(Measures.lidar_beg_time);
        optimized_pose.header.frame_id = "camera_init";
        optimized_pose.pose.position.x = result.optimized_t.x();
        optimized_pose.pose.position.y = result.optimized_t.y();
        optimized_pose.pose.position.z = result.optimized_t.z();
        optimized_pose.pose.orientation.x = result.optimized_q.x();
        optimized_pose.pose.orientation.y = result.optimized_q.y();
        optimized_pose.pose.orientation.z = result.optimized_q.z();
        optimized_pose.pose.orientation.w = result.optimized_q.w();
        local_ba_path_.header = optimized_pose.header;
        local_ba_path_.poses.push_back(optimized_pose);
        pubLocalBAPath_->publish(local_ba_path_);
    }

    void select_local_ba_map_pose(const LocalBAResult &result)
    {
        local_ba_map_pose_valid = false;
        local_ba_map_pose_source = "FAST_LIO";
        local_ba_map_q = Eigen::Quaterniond(state_point.rot.matrix()).normalized();
        local_ba_map_t = state_point.pos;
        local_ba_feedback_translation_gate_pass = false;
        local_ba_feedback_rotation_gate_pass = false;
        local_ba_feedback_reject_reason = "DISABLED";

        if (!local_ba_map_feedback_enable)
        {
            return;
        }
        if (!result.ba_executed)
        {
            local_ba_feedback_reject_reason = "BA_NOT_EXECUTED";
            return;
        }
        if (!result.ba_converged)
        {
            local_ba_feedback_reject_reason = "BA_NOT_CONVERGED";
            return;
        }
        if (!result.ba_accepted)
        {
            local_ba_feedback_reject_reason = "BA_REJECTED";
            return;
        }

        local_ba_feedback_translation_gate_pass =
            result.ba_translation_correction <= local_ba_map_feedback_max_translation;
        local_ba_feedback_rotation_gate_pass =
            result.ba_rotation_correction_deg <= local_ba_map_feedback_max_rotation_deg;
        if (!local_ba_feedback_translation_gate_pass)
        {
            local_ba_feedback_reject_reason = "TRANSLATION_TOO_LARGE";
            return;
        }
        if (!local_ba_feedback_rotation_gate_pass)
        {
            local_ba_feedback_reject_reason = "ROTATION_TOO_LARGE";
            return;
        }

        local_ba_feedback_reject_reason = "ACCEPTED";
        local_ba_map_pose_valid = true;
        local_ba_map_pose_source = "LOCAL_BA";
        local_ba_map_q = result.optimized_q;
        local_ba_map_t = result.optimized_t;
    }

    // Replace the EKF mean with the newest accepted BA pose and velocity.
    // Biases, gravity, and covariance are deliberately untouched: state mean
    // is updated without covariance relinearization/update.
    void apply_local_ba_ekf_pose_velocity_feedback(LocalBAResult &result)
    {
        local_ba_ekf_pose_feedback_used = false;
        result.fast_pose_before = state_point.pos;
        result.fast_velocity_before = state_point.vel;
        result.feedback_reject_reason = "NO_VALID_WINDOW";
        result.feedback_mode = "BA_NOT_READY";

        if (!local_ba_ekf_pose_feedback_enable)
        {
            return;
        }
        if (!result.ba_accepted)
        {
            result.feedback_reject_reason = "BA_NOT_ACCEPTED";
            result.feedback_mode = "BA_NOT_ACCEPTED";
            return;
        }
        if (result.ba_translation_correction >= local_ba_ekf_feedback_max_translation)
        {
            result.feedback_reject_reason = "TRANSLATION_TOO_LARGE";
            return;
        }
        if (result.ba_rotation_correction_deg >= local_ba_ekf_feedback_max_rotation_deg)
        {
            result.feedback_reject_reason = "ROTATION_TOO_LARGE";
            return;
        }
        if (result.ba_velocity_correction >= local_ba_ekf_feedback_max_velocity)
        {
            result.feedback_reject_reason = "VELOCITY_TOO_LARGE";
            return;
        }
        result.feedback_allowed = true;

        if (latest_degeneracy_result.persistent_degenerate)
        {
            result.feedback_mode = "PERSISTENT_DEGENERATE";
            result.feedback_reject_reason = "PERSISTENT_DEGENERACY";
            result.feedback_alpha_pose = local_ba_feedback_alpha_degenerate;
            result.feedback_alpha_velocity = local_ba_velocity_alpha_degenerate;
            return;
        }
        if (latest_degeneracy_result.early_warning)
        {
            result.feedback_mode = "WARNING";
            result.feedback_alpha_pose = local_ba_feedback_alpha_warning;
            result.feedback_alpha_velocity = local_ba_velocity_alpha_warning;
        }
        else
        {
            result.feedback_mode = "NORMAL";
            result.feedback_alpha_pose = local_ba_feedback_alpha_normal;
            result.feedback_alpha_velocity = local_ba_velocity_alpha_normal;
        }
        if (result.feedback_alpha_pose <= 0.0 && result.feedback_alpha_velocity <= 0.0)
        {
            result.feedback_reject_reason = "PERSISTENT_DEGENERACY";
            return;
        }

        const Eigen::Quaterniond q_fast(state_point.rot.matrix());
        const Eigen::Quaterniond q_ba = result.optimized_q.normalized();
        // Eigen::Quaternion::slerp follows the shortest path (including sign handling).
        const Eigen::Quaterniond q_new = q_fast.normalized().slerp(result.feedback_alpha_pose, q_ba).normalized();
        state_point.pos += result.feedback_alpha_pose * (result.optimized_t - state_point.pos);
        state_point.rot = SO3(q_new.toRotationMatrix());
        state_point.vel += result.feedback_alpha_velocity * (result.optimized_v - state_point.vel);
        // State mean is partially corrected by Local BA without covariance relinearization/update.
        kf.change_x(state_point);  // Biases and gravity remain unchanged; no kf.change_P().

        // Keep the window's newest initial pose consistent with the state that
        // will be propagated into the next scan.
        if (!local_ba_frames_.empty())
        {
            local_ba_frames_.back().q = q_new;
            local_ba_frames_.back().t = state_point.pos;
            local_ba_frames_.back().v = state_point.vel;
        }

        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];
        local_ba_ekf_pose_feedback_used = true;
        result.feedback_applied = true;
        result.feedback_reject_reason = "NONE";
        local_ba_ekf_feedback_reject_reason = result.feedback_reject_reason;
    }

    LocalBAResult process_local_ba()
    {
        capture_local_ba_frame();
        const V3D ekf_pos_before_feedback = state_point.pos;
        LocalBAResult result;
        result.window_size = static_cast<int>(local_ba_frames_.size());
        result.optimized_q = Eigen::Quaterniond(state_point.rot.matrix()).normalized();
        result.optimized_t = state_point.pos;
        for (const LocalBAFrame &frame : local_ba_frames_)
        {
            result.num_plane_constraints += static_cast<int>(frame.constraints.size());
        }
        // Always refine once the sliding window is valid; degeneracy affects
        // only the feedback gain, never whether BA is executed.
        result = run_local_ba();
        result.triggered = result.ba_executed;
        apply_local_ba_ekf_pose_velocity_feedback(result);
        const bool previous_map_pose_used_ba = local_ba_previous_map_pose_used_ba_;
        // Never use a separate map-only pose. map_incremental() therefore
        // transforms with state_point, which is the actual post-feedback EKF state.
        local_ba_map_pose_valid = false;
        local_ba_previous_map_pose_used_ba_ = false;

        publish_local_ba_path(result);
        return result;
    }

    bool initialize_balm_export()
    {
        if (balm_export_initialized_)
        {
            return balm_export_ready_;
        }
        balm_export_initialized_ = true;
        namespace fs = std::filesystem;
        const fs::path root(balm_export_root);
        std::error_code ec;
        if (fs::exists(root, ec) && !fs::is_empty(root, ec))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "BALM export directory is not empty; refusing to overwrite: %s",
                         balm_export_root.c_str());
            return false;
        }
        fs::create_directories(root / "scans", ec);
        if (ec)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create BALM export directory: %s", ec.message().c_str());
            return false;
        }
        balm_poses_file_.open((root / "poses.txt").string(), ios::out | ios::trunc);
        balm_timestamps_file_.open((root / "timestamps.txt").string(), ios::out | ios::trunc);
        balm_export_ready_ = balm_poses_file_.is_open() && balm_timestamps_file_.is_open();
        if (!balm_export_ready_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open BALM pose/timestamp files in: %s", balm_export_root.c_str());
        }
        return balm_export_ready_;
    }

    void export_balm_scan_if_requested()
    {
        if (!balm_export_enable)
        {
            return;
        }
        const double elapsed_time = Measures.lidar_beg_time - first_lidar_time;
        if (elapsed_time < balm_export_start_sec || elapsed_time > balm_export_end_sec)
        {
            return;
        }
        if (!initialize_balm_export() || feats_down_body->empty())
        {
            return;
        }

        // feats_down_body is the deskewed, LiDAR-frame cloud actually used by
        // scan-to-map registration after the surface voxel filter.
        const Eigen::Matrix3d R_WL = state_point.rot.matrix() * state_point.offset_R_L_I.matrix();
        const Eigen::Vector3d p_WL = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;
        const Eigen::Quaterniond q_WL(R_WL);
        std::ostringstream index_stream;
        index_stream << std::setw(6) << std::setfill('0') << balm_export_index_ << ".pcd";
        const std::filesystem::path pcd_path = std::filesystem::path(balm_export_root) / "scans" / index_stream.str();
        if (pcl::io::savePCDFileBinary(pcd_path.string(), *feats_down_body) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "BALM export PCD write failed: %s", pcd_path.c_str());
            return;
        }
        // Quaternion ordering is qx qy qz qw. The local scan therefore pairs
        // with world_T_lidar, not FAST-LIO's internal world_T_imu state.
        balm_poses_file_ << balm_export_index_ << " " << fixed << setprecision(9)
                         << Measures.lidar_beg_time << " " << elapsed_time << " "
                         << p_WL.x() << " " << p_WL.y() << " " << p_WL.z() << " "
                         << q_WL.x() << " " << q_WL.y() << " " << q_WL.z() << " " << q_WL.w() << "\n";
        balm_timestamps_file_ << balm_export_index_ << " " << fixed << setprecision(9)
                              << Measures.lidar_beg_time << " " << elapsed_time << "\n";
        if (!balm_poses_file_ || !balm_timestamps_file_)
        {
            std::error_code ec;
            std::filesystem::remove(pcd_path, ec);
            RCLCPP_WARN(this->get_logger(), "BALM export pose/timestamp write failed; discarded scan index %zu", balm_export_index_);
            return;
        }
        if (balm_export_count_ == 0) balm_first_export_timestamp_ = Measures.lidar_beg_time;
        balm_last_export_timestamp_ = Measures.lidar_beg_time;
        ++balm_export_count_;
        ++balm_export_index_;
    }

    void finalize_balm_export()
    {
        if (!balm_export_ready_) return;
        balm_poses_file_.flush();
        balm_timestamps_file_.flush();
        balm_poses_file_.close();
        balm_timestamps_file_.close();
        std::ofstream metadata(std::filesystem::path(balm_export_root) / "metadata.txt", ios::out | ios::trunc);
        metadata << fixed << setprecision(9)
                 << "export_start_sec " << balm_export_start_sec << "\n"
                 << "export_end_sec " << balm_export_end_sec << "\n"
                 << "first_lidar_timestamp " << first_lidar_time << "\n"
                 << "num_exported_scans " << balm_export_count_ << "\n"
                 << "cloud_type DESKEWED_LIDAR_FRAME\n"
                 << "cloud_source feats_down_body_registration_downsampled\n"
                 << "pose_represents world_T_lidar\n"
                 << "extrinsic_translation " << state_point.offset_T_L_I.transpose() << "\n"
                 << "extrinsic_rotation_matrix " << state_point.offset_R_L_I.matrix().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ";")) << "\n";
        RCLCPP_INFO(this->get_logger(),
                    "[BALM Export Summary] range = %.1f ~ %.1f sec num_scans = %zu num_poses = %zu first_export_timestamp = %.9f last_export_timestamp = %.9f output_dir = %s",
                    balm_export_start_sec, balm_export_end_sec, balm_export_count_, balm_export_count_,
                    balm_first_export_timestamp_, balm_last_export_timestamp_, balm_export_root.c_str());
    }

    void update_degeneracy_state()
    {
        DegeneracyResult &result = latest_degeneracy_result;
        result.direction_bad = std::isfinite(result.lambda_min_norm) &&
                               result.lambda_min_norm < degeneracy_direction_threshold;
        result.support_bad = result.valid_plane_num < degeneracy_min_valid_plane_num;
        result.frame_bad = result.direction_bad || result.support_bad;
        degeneracy_frame_bad_history.push_back(result.frame_bad);
        while (static_cast<int>(degeneracy_frame_bad_history.size()) > degeneracy_temporal_window)
        {
            degeneracy_frame_bad_history.pop_front();
        }
        result.bad_count_10 = static_cast<int>(std::count(
            degeneracy_frame_bad_history.begin(), degeneracy_frame_bad_history.end(), true));
        result.persistent_degenerate =
            static_cast<int>(degeneracy_frame_bad_history.size()) == degeneracy_temporal_window &&
            result.bad_count_10 >= degeneracy_temporal_bad_count;

        // The history holds previous scan counts before the current one is appended.
        // Its element ten positions from the back is exactly t-10.
        result.plane_drop_ratio = 0.0;
        result.trend_warning = false;
        if (degeneracy_plane_count_history.size() >= 10)
        {
            const int plane_count_t_minus_10 =
                degeneracy_plane_count_history[degeneracy_plane_count_history.size() - 10];
            if (plane_count_t_minus_10 > 0)
            {
                result.plane_drop_ratio =
                    static_cast<double>(result.valid_plane_num - plane_count_t_minus_10) /
                    plane_count_t_minus_10;
                result.trend_warning = result.plane_drop_ratio < plane_drop_threshold;
            }
        }
        degeneracy_plane_count_history.push_back(result.valid_plane_num);
        while (degeneracy_plane_count_history.size() > 11)
        {
            degeneracy_plane_count_history.pop_front();
        }

        result.direction_warning = std::isfinite(result.lambda_min_norm) &&
                                   result.lambda_min_norm < warning_direction_threshold;
        result.support_warning = result.valid_plane_num < warning_min_plane_num;
        result.warning_frame = result.direction_warning || result.support_warning || result.trend_warning;
        degeneracy_warning_history.push_back(result.warning_frame);
        while (static_cast<int>(degeneracy_warning_history.size()) > warning_window)
        {
            degeneracy_warning_history.pop_front();
        }
        result.warning_count_5 = static_cast<int>(std::count(
            degeneracy_warning_history.begin(), degeneracy_warning_history.end(), true));
        result.early_warning =
            static_cast<int>(degeneracy_warning_history.size()) == warning_window &&
            result.warning_count_5 >= warning_count_threshold;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Degeneracy: frame_bad=%d persistent=%d early_warning=%d plane_drop=%.3f",
                             result.frame_bad ? 1 : 0, result.persistent_degenerate ? 1 : 0,
                             result.early_warning ? 1 : 0, result.plane_drop_ratio);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLocalBAPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::GenericSubscription::SharedPtr sub_pcl_livox_legacy_;
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
    rclcpp::CallbackGroup::SharedPtr processing_callback_group_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int scan_pcd_wait_count_ = 0;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
    ofstream tum_trajectory_file_;
    ofstream pose_trajectory_file_;
    ofstream balm_poses_file_;
    ofstream balm_timestamps_file_;
    bool balm_export_initialized_ = false;
    bool balm_export_ready_ = false;
    size_t balm_export_index_ = 0;
    size_t balm_export_count_ = 0;
    double balm_first_export_timestamp_ = 0.0;
    double balm_last_export_timestamp_ = 0.0;
    deque<LocalBAFrame> local_ba_frames_;
    LocalBAImuSample local_ba_last_imu_sample_;
    bool local_ba_have_last_imu_sample_ = false;
    nav_msgs::msg::Path local_ba_path_;
    int local_ba_next_frame_id_ = 0;
    bool local_ba_previous_map_pose_used_ba_ = false;
};

inline bool parse_cpu_list(const std::string &text, std::vector<int> &cpus)
{
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        if (token.empty())
        {
            continue;
        }
        const auto dash = token.find('-');
        if (dash == std::string::npos)
        {
            cpus.push_back(std::stoi(token));
        }
        else
        {
            const int lo = std::stoi(token.substr(0, dash));
            const int hi = std::stoi(token.substr(dash + 1));
            for (int cpu = lo; cpu <= hi; cpu++)
            {
                cpus.push_back(cpu);
            }
        }
    }
    return !cpus.empty();
}

// Heterogeneous P-core/E-core CPUs (Intel 12th gen+) can produce different
// floating-point results for the same Eigen H^T*H reduction depending on
// which core type executes it, since the scheduler is free to move threads
// between P- and E-cores. Pinning affinity here, before any threads exist,
// makes every thread this process later spawns (executor threads, OMP
// workers, the ikd-Tree rebuild thread) inherit the same P-core-only mask,
// since Linux threads inherit their creator's CPU affinity.
inline void pin_to_performance_cores()
{
    std::ifstream p_core_file("/sys/devices/cpu_core/cpus");
    if (!p_core_file.is_open())
    {
        // Not a hybrid CPU (or not exposed by this kernel) -- leave the
        // default affinity untouched.
        return;
    }
    std::string text;
    std::getline(p_core_file, text);

    std::vector<int> p_cores;
    if (!parse_cpu_list(text, p_cores))
    {
        return;
    }

    cpu_set_t mask;
    CPU_ZERO(&mask);
    for (int cpu : p_cores)
    {
        CPU_SET(cpu, &mask);
    }

    if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) == 0)
    {
        std::cout << "[fast_lio] Pinned process to " << p_cores.size()
                  << " P-core logical CPUs (" << text << ") for deterministic Eigen reductions." << std::endl;
    }
    else
    {
        std::cout << "[fast_lio] Warning: sched_setaffinity to P-cores failed (errno=" << errno << ")" << std::endl;
    }
}

int main(int argc, char** argv)
{
    pin_to_performance_cores();
    rclcpp::init(argc, argv);

    // Different Ouster driver/firmware versions name this field 'ambient' or 'noise';
    // it isn't used by FAST-LIO, so silence the PCL field-mismatch warning instead of
    // hardcoding one name.
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    signal(SIGINT, SigHandle);

    auto node = std::make_shared<LaserMappingNode>();
    const auto hw_threads = std::max(2u, std::thread::hardware_concurrency());
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), hw_threads);
    executor.add_node(node);
    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
