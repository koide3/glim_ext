// FAST-LIO2 Odometry Estimation Module for GLIM
// Wraps the FAST-LIO2 algorithm (ESIKF + ikd-Tree) behind the GLIM odometry interface.
//
// Key FAST-LIO2 components used:
//   - IKFoM ESIKF (Error-State Iterated Kalman Filter) for state estimation
//   - ikd-Tree for incremental map management
//   - IMU forward propagation and point cloud undistortion
//
// This implementation avoids all ROS dependencies by directly using the
// algorithmic headers (esekfom, ikd-Tree, so3_math) and re-implementing
// the IMU/LiDAR processing pipeline with GLIM data types.

#include <glim_ext/odometry_estimation_fastlio2.hpp>

#include <deque>
#include <cmath>
#include <numeric>
#include <algorithm>

#include <omp.h>

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

// FAST-LIO2 algorithmic headers (ROS-independent)
#include <so3_math.h>
#include <use-ikfom.hpp>
#include <ikd-Tree/ikd_Tree.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

// Gravity constant
#ifndef G_m_s2
#define G_m_s2 (9.81)
#endif

#ifndef NUM_MATCH_POINTS
#define NUM_MATCH_POINTS (5)
#endif

#ifndef LASER_POINT_COV
#define LASER_POINT_COV (0.001)
#endif

#ifndef INIT_TIME
#define INIT_TIME (0.1)
#endif

#ifndef MAX_INI_COUNT
#define MAX_INI_COUNT (10)
#endif

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

// PointType used by ikd-Tree (must match FAST-LIO2 convention)
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using V3F = Eigen::Vector3f;

// Pose6D struct (replaces ROS message)
struct Pose6D {
  double offset_time;
  double acc[3];
  double gyr[3];
  double vel[3];
  double pos[3];
  double rot[9];
};

static Pose6D set_pose6d(double t, const V3D& a, const V3D& g, const V3D& v, const V3D& p, const M3D& R) {
  Pose6D kp;
  kp.offset_time = t;
  for (int i = 0; i < 3; i++) {
    kp.acc[i] = a(i);
    kp.gyr[i] = g(i);
    kp.vel[i] = v(i);
    kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++) {
      kp.rot[i * 3 + j] = R(i, j);
    }
  }
  return kp;
}

// IMU data struct (replaces sensor_msgs::Imu)
struct ImuData {
  double stamp;
  V3D linear_acceleration;
  V3D angular_velocity;
};

// Plane estimation for ICP
template <typename T>
static bool esti_plane(Eigen::Matrix<T, 4, 1>& pca_result, const PointVector& point, const T& threshold) {
  Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
  Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (std::fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold) {
      return false;
    }
  }
  return true;
}

// h_share_model function for ESIKF
// We use a void pointer to route the static callback to the Impl instance
static thread_local void* g_impl_ptr = nullptr;
static void h_share_model_static(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

// ============================================================
// Implementation struct (PIMPL)
// ============================================================
struct OdometryEstimationFastLIO2::Impl {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Params = OdometryEstimationFastLIO2Params;
  Params params;

  // Logger
  std::shared_ptr<spdlog::logger> logger;

  // Sensor extrinsics
  Eigen::Isometry3d T_lidar_imu;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;

  // ESIKF
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state_point;

  // ikd-Tree
  KD_TREE<PointType> ikdtree;

  // IMU processing state
  std::deque<ImuData> imu_buffer;
  ImuData last_imu;
  bool imu_need_init;
  bool b_first_frame;
  bool flg_first_scan;
  bool flg_EKF_inited;
  int init_iter_num;
  V3D mean_acc;
  V3D mean_gyr;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  V3D angvel_last;
  V3D acc_s_last;
  double first_lidar_time;
  double last_lidar_end_time;
  Eigen::Matrix<double, 12, 12> Q;

  // Point cloud processing
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr normvec_cloud;
  PointCloudXYZI::Ptr laserCloudOri;
  PointCloudXYZI::Ptr corr_normvect;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  int feats_down_size;
  int effct_feat_num;
  std::vector<bool> point_selected_surf;
  std::vector<PointVector> Nearest_Points;

  // IMU-rate propagated poses from undistortion (absolute timestamps, world-frame IMU poses)
  std::vector<std::pair<double, Eigen::Isometry3d>> imu_propagated_poses;

  // Frame counter and covariance estimation
  long frame_count;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  // Active frames for callbacks
  std::vector<EstimationFrame::Ptr> frames;
  int marginalized_cursor;

  // Local map keyframe for on_marginalized_keyframes output
  EstimationFrame::Ptr target_localmap_frame;

  // Map management
  double cube_side_length;
  double det_range;
  bool localmap_initialized;
  BoxPointType local_map_points;

  // Extrinsic estimation enable
  bool extrinsic_est_en;

  Impl(const Params& p, std::shared_ptr<spdlog::logger> logger_in) : params(p), logger(logger_in) {
    // Load sensor config
    Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
    T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());

    // T_lidar_imu transforms points from IMU frame to LiDAR frame
    // FAST-LIO2 uses Lidar_R_wrt_IMU and Lidar_T_wrt_IMU which is T_imu_lidar
    // i.e., the LiDAR pose expressed in the IMU frame
    Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();
    Lidar_R_wrt_IMU = T_imu_lidar.rotation();
    Lidar_T_wrt_IMU = T_imu_lidar.translation();

    // Init IMU state
    imu_need_init = true;
    b_first_frame = true;
    flg_first_scan = true;
    flg_EKF_inited = false;
    init_iter_num = 1;
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D::Zero();
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    cov_acc_scale = V3D(p.acc_cov, p.acc_cov, p.acc_cov);
    cov_gyr_scale = V3D(p.gyr_cov, p.gyr_cov, p.gyr_cov);
    cov_bias_gyr = V3D(p.b_gyr_cov, p.b_gyr_cov, p.b_gyr_cov);
    cov_bias_acc = V3D(p.b_acc_cov, p.b_acc_cov, p.b_acc_cov);
    angvel_last = V3D::Zero();
    acc_s_last = V3D::Zero();
    first_lidar_time = 0.0;
    last_lidar_end_time = 0.0;
    Q = process_noise_cov();
    last_imu.stamp = 0.0;
    last_imu.linear_acceleration.setZero();
    last_imu.angular_velocity.setZero();

    // Point clouds
    feats_undistort.reset(new PointCloudXYZI());
    feats_down_body.reset(new PointCloudXYZI());
    feats_down_world.reset(new PointCloudXYZI());
    normvec_cloud.reset(new PointCloudXYZI(100000, 1));
    laserCloudOri.reset(new PointCloudXYZI(100000, 1));
    corr_normvect.reset(new PointCloudXYZI(100000, 1));

    downSizeFilterSurf.setLeafSize(p.filter_size_surf, p.filter_size_surf, p.filter_size_surf);
    downSizeFilterMap.setLeafSize(p.filter_size_map, p.filter_size_map, p.filter_size_map);
    feats_down_size = 0;
    effct_feat_num = 0;

    // Frame management
    frame_count = 0;
    covariance_estimation = std::make_unique<CloudCovarianceEstimation>(p.num_threads);
    marginalized_cursor = 0;

    // Map management
    cube_side_length = p.cube_side_length;
    det_range = p.det_range;
    localmap_initialized = false;

    extrinsic_est_en = p.extrinsic_est_en;

    // Init ESIKF
    double epsi[23] = {0.001};
    std::fill(epsi, epsi + 23, 0.001);
    g_impl_ptr = this;
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model_static, p.max_iterations, epsi);
  }

  // ---- IMU Initialization ----
  void imu_init(const std::deque<ImuData>& imu_data, int& N) {
    if (b_first_frame) {
      Reset();
      N = 1;
      b_first_frame = false;
      mean_acc = imu_data.front().linear_acceleration;
      mean_gyr = imu_data.front().angular_velocity;
    }

    for (const auto& imu : imu_data) {
      V3D cur_acc = imu.linear_acceleration;
      V3D cur_gyr = imu.angular_velocity;
      mean_acc += (cur_acc - mean_acc) / N;
      mean_gyr += (cur_gyr - mean_gyr) / N;
      cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
      cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
      N++;
    }

    state_ikfom init_state = kf.get_x();

    // Compute initial rotation so that gravity aligns with world -Z (i.e., +Z is up).
    // The accelerometer measures -gravity in the IMU frame, so mean_acc points upward in IMU frame.
    // We find R_world_imu that maps mean_acc direction to [0,0,1] (world +Z).
    V3D mean_acc_normalized = mean_acc / mean_acc.norm();
    Eigen::Quaterniond q_init = Eigen::Quaterniond::FromTwoVectors(mean_acc_normalized, V3D(0, 0, 1));
    init_state.rot = q_init.normalized();
    init_state.grav = S2(V3D(0, 0, -G_m_s2));

    init_state.bg = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf.change_P(init_P);
    last_imu = imu_data.back();
  }

  void Reset() {
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D::Zero();
    angvel_last = V3D::Zero();
    imu_need_init = true;
    init_iter_num = 1;
    imu_buffer.clear();
    last_imu.stamp = 0.0;
    last_imu.linear_acceleration.setZero();
    last_imu.angular_velocity.setZero();
  }

  // ---- Point cloud undistortion ----
  void undistort_pcl(const std::deque<ImuData>& v_imu_in, double pcl_beg_time, double pcl_end_time, PointCloudXYZI& pcl_out) {
    // Prepend last IMU
    std::deque<ImuData> v_imu = v_imu_in;
    v_imu.push_front(last_imu);

    state_ikfom imu_state = kf.get_x();
    std::vector<Pose6D> IMUpose;
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;
    double dt = 0;
    input_ikfom in;

    for (auto it = v_imu.begin(); it < (v_imu.end() - 1); it++) {
      auto& head = *it;
      auto& tail = *(it + 1);

      if (tail.stamp < last_lidar_end_time) continue;

      angvel_avr << 0.5 * (head.angular_velocity.x() + tail.angular_velocity.x()), 0.5 * (head.angular_velocity.y() + tail.angular_velocity.y()),
        0.5 * (head.angular_velocity.z() + tail.angular_velocity.z());

      acc_avr << 0.5 * (head.linear_acceleration.x() + tail.linear_acceleration.x()), 0.5 * (head.linear_acceleration.y() + tail.linear_acceleration.y()),
        0.5 * (head.linear_acceleration.z() + tail.linear_acceleration.z());

      acc_avr = acc_avr * G_m_s2 / mean_acc.norm();

      if (head.stamp < last_lidar_end_time) {
        dt = tail.stamp - last_lidar_end_time;
      } else {
        dt = tail.stamp - head.stamp;
      }

      in.acc = acc_avr;
      in.gyro = angvel_avr;
      Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
      Q.block<3, 3>(3, 3).diagonal() = cov_acc;
      Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
      Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
      kf.predict(dt, Q, in);

      imu_state = kf.get_x();
      angvel_last = angvel_avr - imu_state.bg;
      acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
      for (int i = 0; i < 3; i++) {
        acc_s_last[i] += imu_state.grav[i];
      }
      double offs_t = tail.stamp - pcl_beg_time;
      IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
    }

    // Predict to pcl_end_time
    double note = pcl_end_time > v_imu.back().stamp ? 1.0 : -1.0;
    dt = note * (pcl_end_time - v_imu.back().stamp);
    kf.predict(dt, Q, in);

    imu_state = kf.get_x();
    last_imu = v_imu.back();
    last_lidar_end_time = pcl_end_time;

    // Save IMU-rate propagated poses (world-frame IMU poses with absolute timestamps)
    imu_propagated_poses.clear();
    imu_propagated_poses.reserve(IMUpose.size() + 1);
    for (const auto& pose : IMUpose) {
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.linear() << pose.rot[0], pose.rot[1], pose.rot[2], pose.rot[3], pose.rot[4], pose.rot[5], pose.rot[6], pose.rot[7], pose.rot[8];
      T.translation() << pose.pos[0], pose.pos[1], pose.pos[2];
      imu_propagated_poses.emplace_back(pcl_beg_time + pose.offset_time, T);
    }
    // Add the final predicted pose at pcl_end_time
    {
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.linear() = imu_state.rot.toRotationMatrix();
      T.translation() = imu_state.pos;
      imu_propagated_poses.emplace_back(pcl_end_time, T);
    }

    // Backward propagation to undistort each point
    if (pcl_out.points.empty()) return;
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << head->rot[0], head->rot[1], head->rot[2], head->rot[3], head->rot[4], head->rot[5], head->rot[6], head->rot[7], head->rot[8];
      vel_imu << head->vel[0], head->vel[1], head->vel[2];
      pos_imu << head->pos[0], head->pos[1], head->pos[2];
      acc_imu << tail->acc[0], tail->acc[1], tail->acc[2];
      angvel_avr << tail->gyr[0], tail->gyr[1], tail->gyr[2];

      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        M3D R_i(R_imu * Exp(angvel_avr, dt));
        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
        V3D P_compensate =
          imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);

        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_out.points.begin()) break;
      }
    }
  }

  // ---- IMU process ----
  bool process_imu(const std::deque<ImuData>& imu_data, double pcl_beg_time, double pcl_end_time, PointCloudXYZI::Ptr pcl_undistort) {
    if (imu_data.empty()) return false;

    if (imu_need_init) {
      imu_init(imu_data, init_iter_num);
      imu_need_init = true;
      last_imu = imu_data.back();

      if (init_iter_num > MAX_INI_COUNT) {
        cov_acc *= std::pow(G_m_s2 / mean_acc.norm(), 2);
        imu_need_init = false;
        cov_acc = cov_acc_scale;
        cov_gyr = cov_gyr_scale;
        // Set time reference so next frame's undistort_pcl uses correct dt
        last_lidar_end_time = pcl_end_time;
        logger->info("IMU Initial Done");
      }
      return false;
    }

    undistort_pcl(imu_data, pcl_beg_time, pcl_end_time, *pcl_undistort);
    return true;
  }

  // ---- Map FOV management ----
  void lasermap_fov_segment() {
    state_point = kf.get_x();
    V3D pos_LiD = state_point.pos + state_point.rot * state_point.offset_T_L_I;

    if (!localmap_initialized) {
      for (int i = 0; i < 3; i++) {
        local_map_points.vertex_min[i] = pos_LiD(i) - cube_side_length / 2.0;
        local_map_points.vertex_max[i] = pos_LiD(i) + cube_side_length / 2.0;
      }
      localmap_initialized = true;
      return;
    }

    const float MOV_THRESHOLD = 1.5f;
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
      dist_to_map_edge[i][0] = std::fabs(pos_LiD(i) - local_map_points.vertex_min[i]);
      dist_to_map_edge[i][1] = std::fabs(pos_LiD(i) - local_map_points.vertex_max[i]);
      if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range || dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range) {
        need_move = true;
      }
    }
    if (!need_move) return;

    std::vector<BoxPointType> cub_needrm;
    BoxPointType new_local_map = local_map_points;
    float mov_dist = std::max((cube_side_length - 2.0 * MOV_THRESHOLD * det_range) * 0.5 * 0.9, double(det_range * (MOV_THRESHOLD - 1)));

    for (int i = 0; i < 3; i++) {
      BoxPointType tmp = local_map_points;
      if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range) {
        new_local_map.vertex_max[i] -= mov_dist;
        new_local_map.vertex_min[i] -= mov_dist;
        tmp.vertex_min[i] = local_map_points.vertex_max[i] - mov_dist;
        cub_needrm.push_back(tmp);
      } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range) {
        new_local_map.vertex_max[i] += mov_dist;
        new_local_map.vertex_min[i] += mov_dist;
        tmp.vertex_max[i] = local_map_points.vertex_min[i] + mov_dist;
        cub_needrm.push_back(tmp);
      }
    }
    local_map_points = new_local_map;

    if (!cub_needrm.empty()) {
      ikdtree.Delete_Point_Boxes(cub_needrm);
    }
  }

  // ---- h_share_model for ESIKF ----
  void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    laserCloudOri->clear();
    corr_normvect->clear();
    double total_residual = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
      PointType& point_body = feats_down_body->points[i];
      PointType& point_world = feats_down_world->points[i];

      V3D p_body(point_body.x, point_body.y, point_body.z);
      V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
      point_world.x = p_global(0);
      point_world.y = p_global(1);
      point_world.z = p_global(2);
      point_world.intensity = point_body.intensity;

      std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
      auto& points_near = Nearest_Points[i];

      if (ekfom_data.converge) {
        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        point_selected_surf[i] = points_near.size() >= NUM_MATCH_POINTS && pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5;
      }

      if (!point_selected_surf[i]) continue;

      Eigen::Vector4f pabcd;
      point_selected_surf[i] = false;
      if (esti_plane(pabcd, points_near, 0.1f)) {
        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
        float score = 1 - 0.9 * std::fabs(pd2) / std::sqrt(p_body.norm());

        if (score > 0.9) {
          point_selected_surf[i] = true;
          normvec_cloud->points[i].x = pabcd(0);
          normvec_cloud->points[i].y = pabcd(1);
          normvec_cloud->points[i].z = pabcd(2);
          normvec_cloud->points[i].intensity = pd2;
        }
      }
    }

    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++) {
      if (point_selected_surf[i]) {
        laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
        corr_normvect->points[effct_feat_num] = normvec_cloud->points[i];
        effct_feat_num++;
      }
    }

    if (effct_feat_num < 1) {
      ekfom_data.valid = false;
      logger->warn("No Effective Points!");
      return;
    }

    // Compute Jacobian H and measurement vector h
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
      const PointType& laser_p = laserCloudOri->points[i];
      V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
      M3D point_be_crossmat;
      point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
      V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      const PointType& norm_p = corr_normvect->points[i];
      V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

      V3D C(s.rot.conjugate() * norm_vec);
      V3D A(point_crossmat * C);
      if (extrinsic_est_en) {
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, A(0), A(1), A(2), B(0), B(1), B(2), C(0), C(1), C(2);
      } else {
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, A(0), A(1), A(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }

      ekfom_data.h(i) = -norm_p.intensity;
    }
  }

  // ---- Map incremental update ----
  void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
      pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));

      if (!Nearest_Points[i].empty() && flg_EKF_inited) {
        const PointVector& points_near = Nearest_Points[i];
        bool need_add = true;
        PointType mid_point;
        float filter_map = downSizeFilterMap.getLeafSize()[0];
        mid_point.x = std::floor(feats_down_world->points[i].x / filter_map) * filter_map + 0.5 * filter_map;
        mid_point.y = std::floor(feats_down_world->points[i].y / filter_map) * filter_map + 0.5 * filter_map;
        mid_point.z = std::floor(feats_down_world->points[i].z / filter_map) * filter_map + 0.5 * filter_map;

        if (
          std::fabs(points_near[0].x - mid_point.x) > 0.5 * filter_map && std::fabs(points_near[0].y - mid_point.y) > 0.5 * filter_map &&
          std::fabs(points_near[0].z - mid_point.z) > 0.5 * filter_map) {
          PointNoNeedDownsample.push_back(feats_down_world->points[i]);
          continue;
        }

        for (int j = 0; j < NUM_MATCH_POINTS; j++) {
          if (static_cast<int>(points_near.size()) < NUM_MATCH_POINTS) break;
          float d1 = (points_near[j].x - mid_point.x) * (points_near[j].x - mid_point.x) + (points_near[j].y - mid_point.y) * (points_near[j].y - mid_point.y) +
                     (points_near[j].z - mid_point.z) * (points_near[j].z - mid_point.z);
          float d2 = (feats_down_world->points[i].x - mid_point.x) * (feats_down_world->points[i].x - mid_point.x) +
                     (feats_down_world->points[i].y - mid_point.y) * (feats_down_world->points[i].y - mid_point.y) +
                     (feats_down_world->points[i].z - mid_point.z) * (feats_down_world->points[i].z - mid_point.z);
          if (d1 < d2) {
            need_add = false;
            break;
          }
        }
        if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
      } else {
        PointToAdd.push_back(feats_down_world->points[i]);
      }
    }

    ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
  }

  void pointBodyToWorld(const PointType* pi, PointType* po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
  }
};

// Static callback that routes to the Impl instance
static void h_share_model_static(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
  static_cast<OdometryEstimationFastLIO2::Impl*>(g_impl_ptr)->h_share_model(s, ekfom_data);
}

// ============================================================
// Params
// ============================================================
OdometryEstimationFastLIO2Params::OdometryEstimationFastLIO2Params() {
  Config config(GlobalConfig::get_config_path("config_odometry"));

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
  max_iterations = config.param<int>("odometry_estimation", "max_iterations", 4);

  filter_size_surf = config.param<double>("odometry_estimation", "filter_size_surf", 0.5);
  filter_size_map = config.param<double>("odometry_estimation", "filter_size_map", 0.5);
  cube_side_length = config.param<double>("odometry_estimation", "cube_side_length", 1000.0);
  det_range = config.param<double>("odometry_estimation", "det_range", 300.0);

  gyr_cov = config.param<double>("odometry_estimation", "gyr_cov", 0.1);
  acc_cov = config.param<double>("odometry_estimation", "acc_cov", 0.1);
  b_gyr_cov = config.param<double>("odometry_estimation", "b_gyr_cov", 0.0001);
  b_acc_cov = config.param<double>("odometry_estimation", "b_acc_cov", 0.0001);

  extrinsic_est_en = config.param<bool>("odometry_estimation", "extrinsic_est_en", false);
}

OdometryEstimationFastLIO2Params::~OdometryEstimationFastLIO2Params() {}

// ============================================================
// OdometryEstimationFastLIO2
// ============================================================
OdometryEstimationFastLIO2::OdometryEstimationFastLIO2(const OdometryEstimationFastLIO2Params& params) {
  logger = spdlog::default_logger()->clone("fastlio2");
  impl = std::make_unique<Impl>(params, logger);
  logger->info("FAST-LIO2 odometry module initialized");
}

OdometryEstimationFastLIO2::~OdometryEstimationFastLIO2() {}

void OdometryEstimationFastLIO2::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  ImuData imu;
  imu.stamp = stamp;
  imu.linear_acceleration = linear_acc;
  imu.angular_velocity = angular_vel;
  impl->imu_buffer.push_back(imu);
}

EstimationFrame::ConstPtr OdometryEstimationFastLIO2::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  Callbacks::on_insert_frame(raw_frame);

  const double pcl_beg_time = raw_frame->stamp;
  const double pcl_end_time = raw_frame->scan_end_time;

  // Skip first scan (need a full IMU sweep before & after)
  if (impl->flg_first_scan) {
    impl->first_lidar_time = pcl_beg_time;
    impl->flg_first_scan = false;

    // Drain IMU buffer of pre-scan data
    while (!impl->imu_buffer.empty() && impl->imu_buffer.front().stamp < pcl_end_time) {
      impl->imu_buffer.pop_front();
    }
    return nullptr;
  }

  // Gather IMU data between last scan end and this scan end
  std::deque<ImuData> imu_for_frame;
  while (!impl->imu_buffer.empty() && impl->imu_buffer.front().stamp < pcl_end_time) {
    imu_for_frame.push_back(impl->imu_buffer.front());
    impl->imu_buffer.pop_front();
  }

  if (imu_for_frame.empty()) {
    logger->warn("No IMU data for frame at t={:.4f}, skipping", pcl_beg_time);
    return nullptr;
  }

  // Convert PreprocessedFrame points to PCL format
  PointCloudXYZI::Ptr pcl_in(new PointCloudXYZI());
  pcl_in->resize(raw_frame->points.size());
  for (size_t i = 0; i < raw_frame->points.size(); i++) {
    auto& p = pcl_in->points[i];
    p.x = raw_frame->points[i].x();
    p.y = raw_frame->points[i].y();
    p.z = raw_frame->points[i].z();
    p.intensity = (i < raw_frame->intensities.size()) ? raw_frame->intensities[i] : 0.0;
    // Store per-point relative time in curvature field (milliseconds)
    p.curvature = (i < raw_frame->times.size()) ? raw_frame->times[i] * 1000.0 : 0.0;
  }

  // Sort by time
  std::sort(pcl_in->points.begin(), pcl_in->points.end(), [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });

  // IMU forward propagation + undistortion
  *(impl->feats_undistort) = *pcl_in;
  bool imu_ready = impl->process_imu(imu_for_frame, pcl_beg_time, pcl_end_time, impl->feats_undistort);
  if (!imu_ready) {
    if (impl->imu_need_init) {
      logger->info("IMU initialization in progress...");
    }
    return nullptr;
  }

  if (impl->feats_undistort->empty()) {
    logger->warn("Empty undistorted cloud, skipping");
    return nullptr;
  }

  impl->state_point = impl->kf.get_x();
  impl->flg_EKF_inited = (pcl_beg_time - impl->first_lidar_time) >= INIT_TIME;

  // Map FOV management
  impl->lasermap_fov_segment();

  // Downsample
  impl->downSizeFilterSurf.setInputCloud(impl->feats_undistort);
  impl->downSizeFilterSurf.filter(*(impl->feats_down_body));
  impl->feats_down_size = impl->feats_down_body->points.size();

  // Initialize the map kdtree if needed
  if (impl->ikdtree.Root_Node == nullptr) {
    if (impl->feats_down_size > 5) {
      impl->ikdtree.set_downsample_param(impl->params.filter_size_map);
      impl->feats_down_world->resize(impl->feats_down_size);
      for (int i = 0; i < impl->feats_down_size; i++) {
        impl->pointBodyToWorld(&(impl->feats_down_body->points[i]), &(impl->feats_down_world->points[i]));
      }
      impl->ikdtree.Build(impl->feats_down_world->points);
    }
    // Return a basic frame even for initial build
    // (Fall through to frame creation below)
  }

  if (impl->feats_down_size < 5) {
    logger->warn("Too few downsampled points ({}), skipping", impl->feats_down_size);
    return nullptr;
  }

  // Prepare for ESIKF update
  impl->normvec_cloud->resize(impl->feats_down_size);
  impl->feats_down_world->resize(impl->feats_down_size);
  impl->Nearest_Points.resize(impl->feats_down_size);
  impl->point_selected_surf.resize(impl->feats_down_size, true);
  std::fill(impl->point_selected_surf.begin(), impl->point_selected_surf.end(), true);

  // Iterated Kalman filter update
  if (impl->ikdtree.Root_Node != nullptr && impl->ikdtree.validnum() > 0) {
    g_impl_ptr = impl.get();
    double solve_H_time = 0;
    impl->kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    if (impl->frame_count % 100 == 0) {
      logger->debug("frame {}: down={} effct={} map_pts={}", impl->frame_count, impl->feats_down_size, impl->effct_feat_num, impl->ikdtree.validnum());
    }
  }

  impl->state_point = impl->kf.get_x();

  // Add points to the map
  impl->map_incremental();

  // ---- Create EstimationFrame ----
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = impl->frame_count++;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu = impl->T_lidar_imu;
  new_frame->raw_frame = raw_frame;

  // Extract state: T_world_imu from ESIKF
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  T_world_imu.linear() = impl->state_point.rot.toRotationMatrix();
  T_world_imu.translation() = impl->state_point.pos;

  new_frame->set_T_world_sensor(FrameID::IMU, T_world_imu);

  // Velocity
  new_frame->v_world_imu = impl->state_point.vel;

  // IMU bias: [acc_bias; gyro_bias]
  new_frame->imu_bias.head<3>() = impl->state_point.ba;
  new_frame->imu_bias.tail<3>() = impl->state_point.bg;

  // Create PointCloudCPU from undistorted points
  // FAST-LIO2's undistorted points are in the LiDAR frame.
  // Transform them to IMU frame for compatibility with sub_mapping.
  Eigen::Isometry3d T_imu_lidar = impl->T_lidar_imu.inverse();
  std::vector<Eigen::Vector4d> undistorted_points(impl->feats_undistort->size());
  for (size_t i = 0; i < impl->feats_undistort->size(); i++) {
    Eigen::Vector4d p_lidar(impl->feats_undistort->points[i].x, impl->feats_undistort->points[i].y, impl->feats_undistort->points[i].z, 1.0);
    undistorted_points[i] = T_imu_lidar.matrix() * p_lidar;
  }

  gtsam_points::PointCloudCPU::Ptr frame_cpu(new gtsam_points::PointCloudCPU(undistorted_points));

  // Estimate covariances and normals
  impl->covariance_estimation->estimate(undistorted_points, raw_frame->neighbors, frame_cpu->normals_storage, frame_cpu->covs_storage);
  frame_cpu->normals = frame_cpu->normals_storage.data();
  frame_cpu->covs = frame_cpu->covs_storage.data();

  new_frame->frame = frame_cpu;
  new_frame->frame_id = FrameID::IMU;

  // Set IMU-rate trajectory from forward propagated poses, corrected for ESIKF update.
  // The propagated poses are pre-update; T_world_imu is post-update.
  // Apply the correction so the trajectory endpoint matches the final pose.
  if (!impl->imu_propagated_poses.empty()) {
    const int N = impl->imu_propagated_poses.size();
    // The last propagated pose is at pcl_end_time (pre-update)
    const Eigen::Isometry3d& T_predicted_end = impl->imu_propagated_poses.back().second;
    // Correction: T_world_imu_corrected * T_predicted_end^{-1}
    Eigen::Isometry3d T_correction = T_world_imu * T_predicted_end.inverse();

    new_frame->imu_rate_trajectory.resize(8, N);
    for (int i = 0; i < N; i++) {
      const auto& [t, pose] = impl->imu_propagated_poses[i];
      Eigen::Isometry3d corrected_pose = T_correction * pose;
      Eigen::Quaterniond q(corrected_pose.linear());
      new_frame->imu_rate_trajectory.col(i) << t, corrected_pose.translation(), q.x(), q.y(), q.z(), q.w();
    }
  }

  // Fire callbacks
  Callbacks::on_new_frame(new_frame);

  impl->frames.push_back(new_frame);

  // Marginalize old frames (keep last 10)
  const int max_active_frames = 10;
  while (static_cast<int>(impl->frames.size()) > max_active_frames) {
    marginalized_frames.push_back(impl->frames.front());
    impl->frames.erase(impl->frames.begin());
  }

  if (!marginalized_frames.empty()) {
    Callbacks::on_marginalized_frames(marginalized_frames);
  }

  std::vector<EstimationFrame::ConstPtr> active_frames(impl->frames.begin(), impl->frames.end());
  Callbacks::on_update_new_frame(active_frames.back());
  Callbacks::on_update_frames(active_frames);

  // Periodically output local map as keyframes (for sub_mapping consumption)
  if (impl->frame_count % 50 == 0 && impl->ikdtree.Root_Node != nullptr) {
    PointVector all_points;
    impl->ikdtree.flatten(impl->ikdtree.Root_Node, all_points, NOT_RECORD);

    if (!all_points.empty()) {
      // Convert ikd-Tree points (world frame) to PointCloudCPU
      std::vector<Eigen::Vector4d> map_points(all_points.size());
      for (size_t i = 0; i < all_points.size(); i++) {
        map_points[i] = Eigen::Vector4d(all_points[i].x, all_points[i].y, all_points[i].z, 1.0);
      }

      EstimationFrame::Ptr localmap_frame(new EstimationFrame);
      localmap_frame->id = impl->frame_count - 1;
      localmap_frame->stamp = new_frame->stamp;
      localmap_frame->T_lidar_imu = impl->T_lidar_imu;
      localmap_frame->T_world_lidar = impl->T_lidar_imu.inverse();
      localmap_frame->T_world_imu.setIdentity();
      localmap_frame->v_world_imu.setZero();
      localmap_frame->imu_bias.setZero();
      localmap_frame->frame_id = FrameID::IMU;
      localmap_frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(map_points);

      std::vector<EstimationFrame::ConstPtr> keyframes = {localmap_frame};
      Callbacks::on_update_keyframes(keyframes);

      if (impl->target_localmap_frame) {
        std::vector<EstimationFrame::ConstPtr> marginalized_keyframes = {impl->target_localmap_frame};
        Callbacks::on_marginalized_keyframes(marginalized_keyframes);
      }

      impl->target_localmap_frame = localmap_frame;
    }
  }

  return new_frame;
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimationFastLIO2::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> remaining;
  for (const auto& f : impl->frames) {
    remaining.push_back(f);
  }
  return remaining;
}

}  // namespace glim
