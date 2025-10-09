/*
 * neo_localization_node.cpp
 *
 *  Created on: Apr 8, 2020
 *      Author: mad
 */

#include <neo_localization/Convert.h>
#include <neo_localization/GridMap.h>
#include <neo_localization/Solver.h>
#include <neo_localization/Util.h>
#include <neo_localization/LocalizationStats.h>
#include "error_monitor/client.h"
#include <std_srvs/Empty.h>

#include <angles/angles.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <array>
#include <cmath>
#include <mutex>
#include <random>
#include <thread>

class KalmanFilter {
public:
  KalmanFilter(double q, double r, double init=0.0, double p0=1.0)
    : Q(q), R(r), X(init), P(p0), initialized(false), last_update_time(ros::Time(0)), outlier_threshold(5.0), min_P(1e-6) {}

  // 设置滤波器噪声参数
  void setParams(double q, double r) { Q = q; R = r; }
  // 设置异常观测抑制阈值
  void setOutlierThreshold(double th) { outlier_threshold = th; }
  // 设置最小协方差下限
  void setMinP(double p) { min_P = p; }

  // 滤波主入口
  double update(double measurement, ros::Time stamp = ros::Time::now()) {
    if (!initialized) {
      X = measurement;
      P = 1.0;
      initialized = true;
      last_update_time = stamp;
      return X;
    }
    // 时间相关过程噪声自适应
    double dt = (stamp - last_update_time).toSec();
    last_update_time = stamp;
    double Q_eff = Q * (dt > 0.0 ? dt : 1.0);
    // 预测步骤
    P = P + Q_eff;
    // 异常值检测
    double innovation = measurement - X;
    double S = P + R;
    double sigma = sqrt(S);
    if (fabs(innovation) > outlier_threshold * sigma) {
      // 异常值检测：超出阈值直接返回预测值
      return X;
    }
    // 更新
    double K = P / S;
    X = X + K * innovation;
    P = (1 - K) * P;
    if (P < min_P) P = min_P;
    return X;
  }
  void reset(double init) {
    X = init;
    P = 1.0;
    initialized = false;
    last_update_time = ros::Time(0);
  }
  double get() const { return X; }
  double getCov() const { return P; }
  bool isInitialized() const { return initialized; }
private:
  double Q, R, X, P;
  bool initialized;
  ros::Time last_update_time;
  double outlier_threshold;
  double min_P;
};

/*
 * Coordinate systems:
 * - Sensor in [meters, rad], aka. "laserX"
 * - Base Link in [meters, rad], aka. "base_link"
 * - Odometry in [meters, rad], aka. "odom"
 * - Map in [meters, rad], aka. "map"
 * - World Grid in [meters, rad], aka. "world"
 * - Tile Grid in [meters, rad], aka. "grid"
 * - World Grid in [pixels]
 * - Tile Grid in [pixels]
 *
 */
class NeoLocalizationNode {
public:
  NeoLocalizationNode()
      : m_node_handle("~"),
        kf_score(0.001, 0.01, 0.55),
        kf_uvw0(0.001, 0.01, 0.4),
        kf_uvw1(0.001, 0.01, 0.25),
        kf_stdxy(0.0001, 0.001, 0.025),
        kf_stdyaw(0.0001, 0.001, 0.025)
  {
    m_node_handle.param("broadcast_tf", m_broadcast_tf, true);

    // Load all other parameters using the private node handle
    m_node_handle.param<std::string>("base_frame", m_base_frame, "base_link");
    m_node_handle.param<std::string>("odom_frame", m_odom_frame, "odom");
    m_node_handle.param<std::string>("map_frame", m_map_frame, "map");
    m_node_handle.param<std::string>("odom_pub_frame", m_odom_pub_frame, "odom_neo");

    m_node_handle.param("map_size", m_map_size, 1000);
    m_node_handle.param("map_downscale", m_map_downscale, 0);
    m_node_handle.param("num_smooth", m_num_smooth, 5);
    m_node_handle.param("solver_iterations", m_solver_iterations, 20);
    m_node_handle.param("sample_rate", m_sample_rate, 10);
    m_node_handle.param("min_points", m_min_points, 20);

    m_node_handle.param("update_gain", m_update_gain, 0.5);
    m_node_handle.param("confidence_gain", m_confidence_gain, 0.01);
    m_node_handle.param("min_score", m_min_score, 0.25);
    m_node_handle.param("odometry_std_xy", m_odometry_std_xy, 0.01);
    m_node_handle.param("odometry_std_yaw", m_odometry_std_yaw, 0.01);
    m_node_handle.param("min_sample_std_xy", m_min_sample_std_xy, 0.025);
    m_node_handle.param("min_sample_std_yaw", m_min_sample_std_yaw, 0.025);
    m_node_handle.param("max_sample_std_xy", m_max_sample_std_xy, 0.5);
    m_node_handle.param("max_sample_std_yaw", m_max_sample_std_yaw, 0.5);
    m_node_handle.param("constrain_threshold", m_constrain_threshold, 0.1);
    m_node_handle.param("constrain_threshold_yaw", m_constrain_threshold_yaw, 0.13); 
    m_node_handle.param("loc_update_rate", m_loc_update_rate, 10.0);
    m_node_handle.param("map_update_rate", m_map_update_rate, 0.5);
    m_node_handle.param("transform_timeout", m_transform_timeout, 0.2);

    // 主要风险评估阈值
    m_node_handle.param("th_score_warn", m_th_score_warn, 0.55);
    m_node_handle.param("th_score_err", m_th_score_err, 0.25);
    m_node_handle.param("th_uvw0_warn", m_th_uvw0_warn, 0.25);
    m_node_handle.param("th_uvw0_err", m_th_uvw0_err, 0.10);
    m_node_handle.param("th_uvw1_warn", m_th_uvw1_warn, 0.20);
    m_node_handle.param("th_uvw1_err", m_th_uvw1_err, 0.08);

    m_node_handle.param("th_stdxy_warn", m_th_stdxy_warn, 0.20);
    m_node_handle.param("th_stdxy_err", m_th_stdxy_err, 0.30);
    m_node_handle.param("th_stdyaw_warn", m_th_stdyaw_warn, 0.20);
    m_node_handle.param("th_stdyaw_err", m_th_stdyaw_err, 0.30);
    
    m_node_handle.param("th_stdxy_warn_3d", m_th_stdxy_warn_3d, 0.20);
    m_node_handle.param("th_stdxy_err_3d", m_th_stdxy_err_3d, 0.30);
    m_node_handle.param("th_stdyaw_warn_3d", m_th_stdyaw_warn_3d, 0.20);
    m_node_handle.param("th_stdyaw_err_3d", m_th_stdyaw_err_3d, 0.30);
    
    m_node_handle.param("th_stdxy_warn_2d", m_th_stdxy_warn_2d, 0.30);
    m_node_handle.param("th_stdxy_err_2d", m_th_stdxy_err_2d, 0.40);
    m_node_handle.param("th_stdyaw_warn_2d", m_th_stdyaw_warn_2d, 0.30);
    m_node_handle.param("th_stdyaw_err_2d", m_th_stdyaw_err_2d, 0.40);
    
    m_node_handle.param("th_stdxy_warn_1d", m_th_stdxy_warn_1d, 0.40);
    m_node_handle.param("th_stdxy_err_1d", m_th_stdxy_err_1d, 0.50);
    m_node_handle.param("th_stdyaw_warn_1d", m_th_stdyaw_warn_1d, 0.40);
    m_node_handle.param("th_stdyaw_err_1d", m_th_stdyaw_err_1d, 0.50);
    
    m_node_handle.param("th_stdxy_warn_0d", m_th_stdxy_warn_0d, 0.50);
    m_node_handle.param("th_stdxy_err_0d", m_th_stdxy_err_0d, 0.60);
    m_node_handle.param("th_stdyaw_warn_0d", m_th_stdyaw_warn_0d, 0.50);
    m_node_handle.param("th_stdyaw_err_0d", m_th_stdyaw_err_0d, 0.60);

    m_node_handle.param("w_score", m_w_score, 0.6);
    m_node_handle.param("w_uvw0", m_w_uvw0, 0.15);
    m_node_handle.param("w_uvw1", m_w_uvw1, 0.15);
    m_node_handle.param("w_stdxy", m_w_stdxy, 0.05);
    m_node_handle.param("w_stdyaw", m_w_stdyaw, 0.05);

    m_node_handle.param("risk_clear", m_risk_clear, 0.20);
    m_node_handle.param("risk_warn", m_risk_warn, 0.3);
    m_node_handle.param("risk_err", m_risk_err, 0.6);

    // 证据积分系统参数
    m_node_handle.param("evidence_up",   m_e_up,   0.10); // 风险增长率
    m_node_handle.param("evidence_down", m_e_down, 0.10); // 风险衰减率
    m_node_handle.param("evidence_warn", m_e_warn, 0.30); // 警告阈值
    m_node_handle.param("evidence_err",  m_e_err,  0.70); // 错误阈值
    
    // 模式稳定性控制
    m_node_handle.param("stable_mode_threshold", m_stable_mode_threshold, 2);  // 减少模式升级所需帧数
    m_node_handle.param("mode_hysteresis", m_mode_hysteresis, 0.04);          // 降低滞后阈值

		// Read initial pose parameters
    double initial_pose_x, initial_pose_y, initial_pose_a;
    bool has_initial_x =
        m_node_handle.getParam("initial_pose_x", initial_pose_x);
    bool has_initial_y =
        m_node_handle.getParam("initial_pose_y", initial_pose_y);
    bool has_initial_a =
        m_node_handle.getParam("initial_pose_a", initial_pose_a);

    // Set initial pose if all parameters are available
    if (has_initial_x && has_initial_y && has_initial_a) {
      ROS_INFO("Setting initial pose from parameters: x=%.3f, y=%.3f, yaw=%.3f",
               initial_pose_x, initial_pose_y, initial_pose_a);
      set_initial_pose(initial_pose_x, initial_pose_y, initial_pose_a);
    } else {
      // 等待手动初始化的信息
      // ROS_INFO("Initial pose parameters not found or incomplete. Waiting for "
      //          "manual initialization.");
    }

    m_sub_scan_topic = m_node_handle.subscribe(
        "/scan", 10, &NeoLocalizationNode::scan_callback, this);
    m_sub_map_topic = m_node_handle.subscribe(
        "/map", 1, &NeoLocalizationNode::map_callback, this);
    m_sub_pose_estimate = m_node_handle.subscribe(
        "/initialpose", 1, &NeoLocalizationNode::pose_callback, this);

		m_pub_map_tile =
        m_node_handle.advertise<nav_msgs::OccupancyGrid>("/map_tile", 1);
    m_pub_loc_pose =
        m_node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/amcl_pose", 10);
    m_pub_loc_pose_2 =
        m_node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/map_pose", 10);
    m_pub_pose_array =
        m_node_handle.advertise<geometry_msgs::PoseArray>("/particlecloud", 10);

		m_pub_stats = m_node_handle.advertise<neo_localization::LocalizationStats>("localization_stats", 1);
    m_pub_stats_filtered = m_node_handle.advertise<neo_localization::LocalizationStats>("localization_stats_filtered", 1);

    m_loc_update_timer = m_node_handle.createTimer(
        ros::Rate(m_loc_update_rate), &NeoLocalizationNode::loc_update, this);

    m_map_update_thread = std::thread(&NeoLocalizationNode::update_loop, this);

    // 初始化错误监控客户端 & 服务
    try {
      m_err_client = std::make_shared<cyanine_os::error_monitor::ErrorMonitorClient>("error_monitor");
    } catch (const std::exception &e) {
      ROS_WARN_STREAM("NeoLocalizationNode: init error_monitor client failed: " << e.what());
    }
    m_srv_fix_mon = m_node_handle.advertiseService("fix_mon", &NeoLocalizationNode::fix_mon_cb, this);
  }

  ~NeoLocalizationNode() {
    if (m_map_update_thread.joinable()) {
      m_map_update_thread.join();
    }
  }

protected:
  /*
   * Computes localization update for a single laser scan.
   */
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan) {
    std::lock_guard<std::mutex> lock(m_node_mutex);

    if (!m_map) {
      return;
    }
    m_scan_buffer[scan->header.frame_id] = scan;
  }

  /*
   * Convert/Transform a scan from ROS format to a specified base frame.
   */
  std::vector<scan_point_t>
  convert_scan(const sensor_msgs::LaserScan::ConstPtr &scan,
               const Matrix<double, 4, 4> &odom_to_base) {
    std::vector<scan_point_t> points;

    tf::StampedTransform sensor_to_base;
    try {
      m_tf.lookupTransform(m_base_frame, scan->header.frame_id, ros::Time(0),
                           sensor_to_base);
    } catch (const std::exception &ex) {
      ROS_WARN_STREAM(
          "NeoLocalizationNode: lookupTransform(scan->header.frame_id, "
          "m_base_frame) failed: "
          << ex.what());
      return points;
    }

    tf::StampedTransform base_to_odom;
    try {
      m_tf.waitForTransform(m_odom_frame, m_base_frame, scan->header.stamp,
                            ros::Duration(m_transform_timeout));
      m_tf.lookupTransform(m_odom_frame, m_base_frame, scan->header.stamp,
                           base_to_odom);
    } catch (const std::exception &ex) {
      ROS_WARN_STREAM("NeoLocalizationNode: lookupTransform(m_base_frame, "
                      "m_odom_frame) failed: "
                      << ex.what());
      return points;
    }

    const Matrix<double, 4, 4> S = convert_transform_3(sensor_to_base);
    const Matrix<double, 4, 4> L = convert_transform_25(base_to_odom);

    // precompute transformation matrix from sensor to requested base
    const Matrix<double, 4, 4> T = odom_to_base * L * S;

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      if (scan->ranges[i] <= scan->range_min ||
          scan->ranges[i] >= scan->range_max) {
        continue; // no actual measurement
      }

      // transform sensor points into base coordinate system
      const Matrix<double, 3, 1> scan_pos =
          (T * rotate3_z<double>(scan->angle_min + i * scan->angle_increment) *
           Matrix<double, 4, 1>{scan->ranges[i], 0, 0, 1})
              .project();
      scan_point_t point;
      point.x = scan_pos[0];
      point.y = scan_pos[1];
      points.emplace_back(point);
    }
    return points;
  }

  void loc_update(const ros::TimerEvent &event) {
    double best_score_snapshot = 0.0;
    Matrix<double, 3, 1> grad_std_uvw_snapshot;
    double std_xy_snapshot = 0.0, std_yaw_snapshot = 0.0;
    int mode_snapshot = 0;
    ros::Time offset_time_snapshot;
    bool have_snapshot = false;

    {
      std::lock_guard<std::mutex> lock(m_node_mutex);
      if (!m_map || m_scan_buffer.empty()) {
        return;
      }

      tf::StampedTransform base_to_odom;
      try {
        m_tf.lookupTransform(m_odom_frame, m_base_frame, ros::Time(0),
                             base_to_odom);
      } catch (const std::exception &ex) {
        ROS_WARN_STREAM("NeoLocalizationNode: lookupTransform(m_base_frame, "
                        "m_odom_frame) failed: "
                        << ex.what());
        return;
      }

      const Matrix<double, 4, 4> L = convert_transform_25(base_to_odom);
      const Matrix<double, 4, 4> T = translate25(m_offset_x, m_offset_y) *
                                     rotate25_z(m_offset_yaw); // odom to map

      const Matrix<double, 3, 1> odom_pose =
          (L * Matrix<double, 4, 1>{0, 0, 0, 1}).project();
      const double dist_moved = (odom_pose - m_last_odom_pose).get<2>().norm();
      const double rad_rotated =
          fabs(angles::normalize_angle(odom_pose[2] - m_last_odom_pose[2]));

      std::vector<scan_point_t> points;

      // convert all scans to current base frame
      for (const auto &scan : m_scan_buffer) {
        auto scan_points = convert_scan(scan.second, L.inverse());
        points.insert(points.end(), scan_points.begin(), scan_points.end());
      }

      // check for number of points
      if (points.size() < m_min_points) {
        ROS_WARN_STREAM(
            "NeoLocalizationNode: Number of points too low: " << points.size());
        return;
      }

      auto pose_array = boost::make_shared<geometry_msgs::PoseArray>();
      pose_array->header.stamp = base_to_odom.stamp_;
      pose_array->header.frame_id = m_map_frame;

      // calc predicted grid pose based on odometry
      const Matrix<double, 3, 1> grid_pose =
          (m_grid_to_map.inverse() * T * L * Matrix<double, 4, 1>{0, 0, 0, 1})
              .project();

      // setup distributions
      std::normal_distribution<double> dist_x(grid_pose[0], m_sample_std_xy);
      std::normal_distribution<double> dist_y(grid_pose[1], m_sample_std_xy);
      std::normal_distribution<double> dist_yaw(grid_pose[2], m_sample_std_yaw);

      // solve odometry prediction first
      m_solver.pose_x = grid_pose[0];
      m_solver.pose_y = grid_pose[1];
      m_solver.pose_yaw = grid_pose[2];

      for (int iter = 0; iter < m_solver_iterations; ++iter) {
        m_solver.solve<float>(*m_map, points);
      }

      double best_x = m_solver.pose_x;
      double best_y = m_solver.pose_y;
      double best_yaw = m_solver.pose_yaw;
      double best_score = m_solver.r_norm;

      std::vector<Matrix<double, 3, 1>> seeds(m_sample_rate);
      std::vector<Matrix<double, 3, 1>> samples(m_sample_rate);
      std::vector<double> sample_errors(m_sample_rate);

      for (int i = 0; i < m_sample_rate; ++i) {
        // generate new sample
        m_solver.pose_x = dist_x(m_generator);
        m_solver.pose_y = dist_y(m_generator);
        m_solver.pose_yaw = dist_yaw(m_generator);

        seeds[i] = Matrix<double, 3, 1>{m_solver.pose_x, m_solver.pose_y,
                                        m_solver.pose_yaw};

        // solve sample
        for (int iter = 0; iter < m_solver_iterations; ++iter) {
          m_solver.solve<float>(*m_map, points);
        }

        // save sample
        const auto sample = Matrix<double, 3, 1>{m_solver.pose_x, m_solver.pose_y,
                                                 m_solver.pose_yaw};
        samples[i] = sample;
        sample_errors[i] = m_solver.r_norm;

        // check if sample is better
        if (m_solver.r_norm > best_score) {
          best_x = m_solver.pose_x;
          best_y = m_solver.pose_y;
          best_yaw = m_solver.pose_yaw;
          best_score = m_solver.r_norm;
        }

        // add to visualization
        {
          const Matrix<double, 3, 1> map_pose =
              (m_grid_to_map * sample.extend()).project();
          tf::Pose pose;
          pose.setOrigin(tf::Vector3(map_pose[0], map_pose[1], 0));
          pose.setRotation(tf::createQuaternionFromYaw(map_pose[2]));
          geometry_msgs::Pose tmp;
          tf::poseTFToMsg(pose, tmp);
          pose_array->poses.push_back(tmp);
        }
      }

      // compute covariances
      double mean_score = 0;
      Matrix<double, 3, 1> mean_xyw;
      Matrix<double, 3, 1> seed_mean_xyw;
      const double var_error = compute_variance(sample_errors, mean_score);
      const Matrix<double, 3, 3> var_xyw = compute_covariance(samples, mean_xyw);
      const Matrix<double, 3, 3> grad_var_xyw =
          compute_virtual_scan_covariance_xyw(
              m_map, points, Matrix<double, 3, 1>{best_x, best_y, best_yaw});

      // compute gradient characteristic
      std::array<Matrix<double, 2, 1>, 2> grad_eigen_vectors;
      const Matrix<double, 2, 1> grad_eigen_values =
          compute_eigenvectors_2(grad_var_xyw.get<2, 2>(), grad_eigen_vectors);
      const Matrix<double, 3, 1> grad_std_uvw{sqrt(grad_eigen_values[0]),
                                              sqrt(grad_eigen_values[1]),
                                              sqrt(grad_var_xyw(2, 2))};

      int potential_mode = 0;
      int new_mode = m_mode;  

      // 1. 分数判断
      if (best_score <= m_min_score) {
        potential_mode = 0;
      } 
      // 2. 根据约束强度判定潜在模式
      else {
        bool has_x_constraint = grad_std_uvw[0] > m_constrain_threshold;
        bool has_y_constraint = grad_std_uvw[1] > m_constrain_threshold;
        bool has_yaw_constraint = grad_std_uvw[2] > m_constrain_threshold_yaw;

        if (has_x_constraint) {
          if (has_y_constraint) {
            potential_mode = 3;  
          } else if (has_yaw_constraint) {
            potential_mode = 2;  
          } else {
            potential_mode = 1;  
          }
        } else {
          potential_mode = 0;    
        }
      }

      // 3. 应用滞后逻辑和稳定性判断
      static int mode_downgrade_cooldown = 0; 
      const int mode_downgrade_cooldown_threshold = 5; 
      
      if (mode_downgrade_cooldown > 0) {
        mode_downgrade_cooldown--;
      }

      if (potential_mode != m_mode) {
        if (potential_mode > m_mode) {
          bool strong_evidence = true;
          
          if (potential_mode == 3) {
            strong_evidence = (grad_std_uvw[0] > m_constrain_threshold * (1.0 + m_mode_hysteresis)) && 
                              (grad_std_uvw[1] > m_constrain_threshold * (1.0 + m_mode_hysteresis)) &&
                              (best_score > m_min_score * 1.2);  
          } 
          else if (potential_mode == 2) {
            strong_evidence = (grad_std_uvw[0] > m_constrain_threshold * (1.0 + m_mode_hysteresis)) && 
                              (grad_std_uvw[2] > m_constrain_threshold_yaw * (1.0 + m_mode_hysteresis));
          }
          else if (potential_mode == 1) {
            strong_evidence = (grad_std_uvw[0] > m_constrain_threshold * (1.0 + m_mode_hysteresis));
          }
          
          if (strong_evidence) {
            m_stable_mode_count++;
            if (m_stable_mode_count >= m_stable_mode_threshold) {
              new_mode = potential_mode;
              m_stable_mode_count = 0;  
              ROS_INFO("Mode upgraded: %d -> %d (strong evidence)", m_mode, new_mode);
            }
          } else {
            m_stable_mode_count = 0;  
          }
        }
        else {
          if (mode_downgrade_cooldown == 0) { 
            bool weak_evidence = false;
            
            if (m_mode == 3) {
              weak_evidence = (grad_std_uvw[0] < m_constrain_threshold * (1.0 - m_mode_hysteresis)) || 
                              (grad_std_uvw[1] < m_constrain_threshold * (1.0 - m_mode_hysteresis)) ||
                              (best_score < m_min_score * 1.1);
                   if (grad_std_uvw[2] > m_constrain_threshold_yaw * 1.2) {
            if (grad_std_uvw[0] > m_constrain_threshold) {
              potential_mode = std::max(2, potential_mode);
              weak_evidence = false;  
              ROS_INFO("Keep 2D mode - strong X and Yaw");
            } else if (best_score > m_min_score * 0.9) {
              // 极度信任Yaw：当Yaw约束极好时限制模式降级
              potential_mode = std::max(2, m_mode - 1);
              weak_evidence = (m_mode > 3);  
              ROS_INFO("Limited downgrade - strong Yaw");
            }
          }
            }
            else if (m_mode == 2) {
              weak_evidence = (grad_std_uvw[0] < m_constrain_threshold * (1.0 - m_mode_hysteresis)) || 
                              (grad_std_uvw[2] < m_constrain_threshold_yaw * (1.0 - m_mode_hysteresis));
              
              // 强化Yaw信任：降低维持2D模式的Yaw阈值，延长冷却时间
              if ((grad_std_uvw[2] > m_constrain_threshold_yaw * 0.85) && (best_score > m_min_score * 0.8)) {
                m_stable_mode_count = 0;  
                weak_evidence = false;
                ROS_INFO("Keep 2D mode - yaw strong (%.3f > %.3f)", 
                         grad_std_uvw[2], m_constrain_threshold_yaw * 0.85);
                
                mode_downgrade_cooldown = std::max(mode_downgrade_cooldown, 15); 
              }
            }
            else if (m_mode == 1) {
              weak_evidence = (grad_std_uvw[0] < m_constrain_threshold * (1.0 - m_mode_hysteresis));
            }
            
            if (weak_evidence) {
              m_stable_mode_count++;
              if (m_stable_mode_count >= (m_stable_mode_threshold + 2)) { 
                new_mode = potential_mode;
                m_stable_mode_count = 0;
                mode_downgrade_cooldown = mode_downgrade_cooldown_threshold; 
                ROS_INFO("Mode downgraded: %d -> %d (weak evidence)", m_mode, new_mode);
              }
            } else {
              m_stable_mode_count = 0;
            }
          } else {
            m_stable_mode_count = 0;
          }
        }
      } else {
        m_stable_mode_count = 0;
      }

      // 低得分时的模式保持逻辑
      if (best_score < m_min_score * 0.8) {
        if (grad_std_uvw[2] > m_constrain_threshold_yaw * 0.75) {  
          if (m_mode >= 2) {
            new_mode = 2;  
            ROS_WARN_THROTTLE(1.0, "Maintain 2D mode (%.2f), strong yaw: (%.3f)", best_score, grad_std_uvw[2]);
            
            mode_downgrade_cooldown = std::max(mode_downgrade_cooldown, 20);
          } else if (grad_std_uvw[0] > m_constrain_threshold * 0.6) {
            // X方向约束尚可，保持1D
            new_mode = 1;
            ROS_WARN_THROTTLE(1.0, "Maintain 1D mode (%.2f)", best_score);
          } else {
            // 即使降级到0D，仍极度信任yaw
            new_mode = 0;
            ROS_WARN_THROTTLE(1.0, "Mode 0D - maximum yaw trust, score: %.2f", best_score);
          }
        } else {
          // yaw约束较弱但仍保持较高信任度
          new_mode = 0;
          ROS_WARN_THROTTLE(1.0, "Mode 0D - elevated yaw trust, score: %.2f", best_score);
        }
      }
      
      // 快速模式升级逻辑
      if (m_mode == 0 && best_score > m_min_score * 1.2) { 
        if (grad_std_uvw[0] > m_constrain_threshold * 1.1 &&
            grad_std_uvw[1] > m_constrain_threshold * 1.1) {
          new_mode = 3;
          ROS_INFO("Rapid mode upgrade: 0D to 3D (score=%.2f, X=%.3f, Y=%.3f)", 
                   best_score, grad_std_uvw[0], grad_std_uvw[1]);
        } else if (grad_std_uvw[0] > m_constrain_threshold * 1.0 &&
                   grad_std_uvw[2] > m_constrain_threshold_yaw * 1.0) {
          // X和Yaw都良好时升级到2D
          new_mode = 2;
          ROS_INFO("Rapid mode upgrade: 0D to 2D (X=%.3f, Yaw=%.3f)", 
                   grad_std_uvw[0], grad_std_uvw[2]);
        } else if (grad_std_uvw[2] > m_constrain_threshold_yaw * 1.1) {
          // 极度信任Yaw：当Yaw约束良好时直接升级到1D
          new_mode = 1; 
          ROS_INFO("Rapid mode upgrade: 0D to 1D (strong yaw=%.3f)", grad_std_uvw[2]);
        }
      }
      
      int mode = new_mode;
      m_mode = new_mode; 

      if (mode > 0) {
        double new_grid_x = best_x;
        double new_grid_y = best_y;
        double new_grid_yaw = best_yaw;

        if (mode < 3) {
          // constrain update to the good direction (ie. in direction of the eigen
          // vector with the smaller sigma)
          const auto delta = Matrix<double, 2, 1>{best_x, best_y} -
                             Matrix<double, 2, 1>{grid_pose[0], grid_pose[1]};
          const auto dist = grad_eigen_vectors[0].dot(delta);
          new_grid_x = grid_pose[0] + dist * grad_eigen_vectors[0][0];
          new_grid_y = grid_pose[1] + dist * grad_eigen_vectors[0][1];
        }
        
        if (mode < 2) {
          // 允许部分更新yaw，仅在低得分或极低yaw约束时完全保持旧方向
          if (grad_std_uvw[2] > m_constrain_threshold_yaw * 0.5 && best_score > m_min_score * 0.6) {
            // 仍有一定yaw约束时采用部分更新
            const double yaw_blend = 0.3;  // 弱yaw约束时的部分更新率
            new_grid_yaw = angles::normalize_angle(
              grid_pose[2] + angles::shortest_angular_distance(grid_pose[2], best_yaw) * yaw_blend);
            ROS_DEBUG_THROTTLE(1.0, "Partial yaw update in mode %d: %.3f -> %.3f", 
                              mode, grid_pose[2], new_grid_yaw);
          } else {
            new_grid_yaw = grid_pose[2]; 
          }
        }

        // use best sample for update
        Matrix<double, 4, 4> grid_pose_new =
            translate25(new_grid_x, new_grid_y) * rotate25_z(new_grid_yaw);

        // compute new odom to map offset from new grid pose
        const Matrix<double, 3, 1> new_offset =
            (m_grid_to_map * grid_pose_new * L.inverse() *
             Matrix<double, 4, 1>{0, 0, 0, 1})
                .project();

        // apply new offset with an exponential low pass filter
        m_offset_x += (new_offset[0] - m_offset_x) * m_update_gain;
        m_offset_y += (new_offset[1] - m_offset_y) * m_update_gain;
        m_offset_yaw +=
            angles::shortest_angular_distance(m_offset_yaw, new_offset[2]) *
            m_update_gain;
      }
      m_offset_time = base_to_odom.stamp_;

      double th_stdxy_warn_active, th_stdxy_err_active;
      double th_stdyaw_warn_active, th_stdyaw_err_active;

      // 选择当前模式的阈值
      switch(mode) {
        case 3:
          th_stdxy_warn_active = m_th_stdxy_warn_3d;
          th_stdxy_err_active = m_th_stdxy_err_3d;
          th_stdyaw_warn_active = m_th_stdyaw_warn_3d;
          th_stdyaw_err_active = m_th_stdyaw_err_3d;
          break;
        case 2:
          th_stdxy_warn_active = m_th_stdxy_warn_2d;
          th_stdxy_err_active = m_th_stdxy_err_2d;
          th_stdyaw_warn_active = m_th_stdyaw_warn_2d;
          th_stdyaw_err_active = m_th_stdyaw_err_2d;
          break;
        case 1:
          th_stdxy_warn_active = m_th_stdxy_warn_1d;
          th_stdxy_err_active = m_th_stdxy_err_1d;
          th_stdyaw_warn_active = m_th_stdyaw_warn_1d;
          th_stdyaw_err_active = m_th_stdyaw_err_1d;
          break;
        case 0:
        default:
          th_stdxy_warn_active = m_th_stdxy_warn_0d;
          th_stdxy_err_active = m_th_stdxy_err_0d;
          th_stdyaw_warn_active = m_th_stdyaw_warn_0d;
          th_stdyaw_err_active = m_th_stdyaw_err_0d;
          break;
      }

      // 基于目标值和平滑率的统一扩散控制
      static double target_std_xy = 0.0;
      static double target_std_yaw = 0.0;
      static int downgrade_cooldown = 0;
      
      if (downgrade_cooldown > 0) {
        downgrade_cooldown--;
      }
      
      // 根据当前模式计算目标扩散值
      switch(mode) {
        case 3: 
          target_std_xy = std::max(m_min_sample_std_xy, th_stdxy_warn_active * 0.65);
          target_std_yaw = std::max(m_min_sample_std_yaw, th_stdyaw_warn_active * 0.15); 
          break;
          
        case 2:
          target_std_xy = std::max(m_min_sample_std_xy, th_stdxy_warn_active * 0.75);
          target_std_yaw = std::max(m_min_sample_std_yaw, th_stdyaw_warn_active * 0.15); 
          break;
          
        case 1:
          target_std_xy = std::max(m_min_sample_std_xy, th_stdxy_warn_active * 0.8);
          target_std_yaw = std::max(m_min_sample_std_yaw, th_stdyaw_warn_active * 0.15); 
          break;
          
        case 0:
        default:
          target_std_xy = std::max(m_min_sample_std_xy, th_stdxy_warn_active * 0.9);
          target_std_yaw = std::max(m_min_sample_std_yaw, th_stdyaw_warn_active * 0.2); 
          break;
      }
      
      double odom_factor_xy = 0.0;
      double odom_factor_yaw = 0.0;
      
      if (mode <= 1) {
        // 低模式下，位置和yaw都受里程计影响极大
        odom_factor_xy = mode == 0 ? 1.0 : 0.5; 
        odom_factor_yaw = mode == 0 ? 1.0 : 0.9; 
      } else {
        odom_factor_xy = 0.0;
        odom_factor_yaw = mode == 2 ? 0.7 : 0.5; 
      }
      
      target_std_xy += dist_moved * m_odometry_std_xy * odom_factor_xy;
      target_std_yaw += rad_rotated * m_odometry_std_yaw * odom_factor_yaw;
      
      // 限制最大目标值
      target_std_xy = fmin(target_std_xy, m_max_sample_std_xy * 0.9);
      target_std_yaw = fmin(target_std_yaw, m_max_sample_std_yaw * 0.9);
      
      double alpha_xy = 0.08;  
      double alpha_yaw = 0.08;
      if (best_score > 0.6) {  
        alpha_xy = 0.12;
        alpha_yaw = 0.12;
      }
      
      // 针对增加和减少使用不同系数，防止震荡
      double alpha_xy_up = 0.06;  
      double alpha_xy_down = 0.12; 
      
      // yaw方向使用极度保守的参数，确保最稳定的yaw方向估计
      double alpha_yaw_up = 0.02;   
      double alpha_yaw_down = 0.15; 
      
      // 如果分数较高，可以更快速地收敛yaw方向（意味着更信任当前测量）
      if (best_score > 0.6) {  
        alpha_yaw_down = 0.2;  
      }
      
      // 使用合适的系数计算增量
      double delta_xy = 0.0;
      if (target_std_xy > m_sample_std_xy) {
        delta_xy = (target_std_xy - m_sample_std_xy) * alpha_xy_up;
      } else {
        delta_xy = (target_std_xy - m_sample_std_xy) * alpha_xy_down;
      }
      
      double delta_yaw = 0.0;
      if (target_std_yaw > m_sample_std_yaw) {
        delta_yaw = (target_std_yaw - m_sample_std_yaw) * alpha_yaw_up;
      } else {
        delta_yaw = (target_std_yaw - m_sample_std_yaw) * alpha_yaw_down;
      }
      
      // 单步方差变化限制
      const double max_step_xy = 0.02;  // xy方向每步最大方差变化
      const double max_step_yaw = 0.025; // yaw方向允许更大变化以加速收敛
      
      if (std::abs(delta_xy) > max_step_xy) {
        delta_xy = delta_xy > 0 ? max_step_xy : -max_step_xy;
      }
      
      if (std::abs(delta_yaw) > max_step_yaw) {
        delta_yaw = delta_yaw > 0 ? max_step_yaw : -max_step_yaw;
      }
      
      // 更新方差
      m_sample_std_xy += delta_xy;
      m_sample_std_yaw += delta_yaw;
      
      // 设置方差安全范围
      double min_std_xy = std::max(m_min_sample_std_xy, th_stdxy_warn_active * 0.4);
      // 极度信任yaw: 允许极低的yaw方差下限，特别是在模式0D和1D
      double min_std_yaw = std::max(m_min_sample_std_yaw, 
                                    mode <= 1 ? th_stdyaw_warn_active * 0.1 : th_stdyaw_warn_active * 0.15);
      
      m_sample_std_xy = fmin(fmax(m_sample_std_xy, min_std_xy), m_max_sample_std_xy);
      m_sample_std_yaw = fmin(fmax(m_sample_std_yaw, min_std_yaw), m_max_sample_std_yaw);
      
      ROS_DEBUG_THROTTLE(2.0, "Spread control: mode=%d, std_xy=%.3f->%.3f (d=%.3f), std_yaw=%.3f->%.3f (d=%.3f)",
                       mode, m_sample_std_xy, target_std_xy, delta_xy, 
                       m_sample_std_yaw, target_std_yaw, delta_yaw);

      // publish new transform
      broadcast();

      const Matrix<double, 3, 1> new_map_pose =
          (translate25(m_offset_x, m_offset_y) * rotate25_z(m_offset_yaw) * L *
           Matrix<double, 4, 1>{0, 0, 0, 1})
              .project();

      // publish localization pose
      auto loc_pose =
          boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
      loc_pose->header.stamp = m_offset_time;
      loc_pose->header.frame_id = m_map_frame;
      loc_pose->pose.pose.position.x = new_map_pose[0];
      loc_pose->pose.pose.position.y = new_map_pose[1];
      loc_pose->pose.pose.position.z = 0;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(new_map_pose[2]),
                            loc_pose->pose.pose.orientation);
      for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < 3; ++i) {
          const int i_ = (i == 2 ? 5 : i);
          const int j_ = (j == 2 ? 5 : j);
          loc_pose->pose.covariance[j_ * 6 + i_] = var_xyw(i, j);
        }
      }
      m_pub_loc_pose.publish(loc_pose);
      m_pub_loc_pose_2.publish(loc_pose);

      // publish visualization
      m_pub_pose_array.publish(pose_array);

      // keep last odom pose
      m_last_odom_pose = odom_pose;

      if (update_counter++ % 10 == 0) {
        // ROS_INFO_STREAM(
        //     "NeoLocalizationNode: score="
        //     << float(best_score) << ", grad_uvw=[" << float(grad_std_uvw[0])
        //     << ", " << float(grad_std_uvw[1]) << ", " << float(grad_std_uvw[2])
        //     << "], std_xy=" << float(m_sample_std_xy)
        //     << " m, std_yaw=" << float(m_sample_std_yaw) << " rad, mode=" << mode
        //     << "D, " << m_scan_buffer.size() << " scans");
      }

      // clear scan buffer
      m_scan_buffer.clear();

      // 保存当前结果到快照，用于锁外处理
      best_score_snapshot = best_score;
      grad_std_uvw_snapshot = grad_std_uvw;
      std_xy_snapshot = m_sample_std_xy;
      std_yaw_snapshot = m_sample_std_yaw;
      mode_snapshot = mode;
      offset_time_snapshot = m_offset_time;
      have_snapshot = true;
    }
    if (!have_snapshot) {
      return;
    }
    
    // 卡尔曼滤波处理
    double filtered_score = kf_score.update(best_score_snapshot, offset_time_snapshot);
    double filtered_uvw0 = kf_uvw0.update(grad_std_uvw_snapshot[0], offset_time_snapshot);
    double filtered_uvw1 = kf_uvw1.update(grad_std_uvw_snapshot[1], offset_time_snapshot);
    double filtered_stdxy = kf_stdxy.update(std_xy_snapshot, offset_time_snapshot);
    double filtered_stdyaw = kf_stdyaw.update(std_yaw_snapshot, offset_time_snapshot);

    // 发布自定义消息
    neo_localization::LocalizationStats stats_msg;
    stats_msg.header.stamp = offset_time_snapshot; 
    stats_msg.header.frame_id = m_map_frame;
    stats_msg.score = best_score_snapshot;
    stats_msg.grad_uvw[0] = grad_std_uvw_snapshot[0];
    stats_msg.grad_uvw[1] = grad_std_uvw_snapshot[1];
    stats_msg.grad_uvw[2] = grad_std_uvw_snapshot[2];
    stats_msg.std_xy = std_xy_snapshot;
    stats_msg.std_yaw = std_yaw_snapshot;
    stats_msg.mode = mode_snapshot;
    m_pub_stats.publish(stats_msg);

    // 发布滤波后的消息
    neo_localization::LocalizationStats filtered_msg = stats_msg;
    filtered_msg.score = filtered_score;
    filtered_msg.grad_uvw[0] = filtered_uvw0;
    filtered_msg.grad_uvw[1] = filtered_uvw1;
    filtered_msg.std_xy = filtered_stdxy;
    filtered_msg.std_yaw = filtered_stdyaw;

    // 指标归一化并计算风险分
    auto clamp01 = [](double x){ return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); };
    
    double r_score = clamp01((m_th_score_warn - filtered_score) / std::max(1e-6, m_th_score_warn));
    double r_uvw0  = clamp01((m_th_uvw0_warn  - filtered_uvw0) / std::max(1e-6, m_th_uvw0_warn));
    double r_uvw1  = clamp01((m_th_uvw1_warn  - filtered_uvw1) / std::max(1e-6, m_th_uvw1_warn));
    
    double th_stdxy_warn_active = m_th_stdxy_warn;
    double th_stdxy_err_active = m_th_stdxy_err;
    double th_stdyaw_warn_active = m_th_stdyaw_warn;
    double th_stdyaw_err_active = m_th_stdyaw_err;
    
    switch(mode_snapshot) {
        case 3: 
            th_stdxy_warn_active = m_th_stdxy_warn_3d;
            th_stdxy_err_active = m_th_stdxy_err_3d;
            th_stdyaw_warn_active = m_th_stdyaw_warn_3d;
            th_stdyaw_err_active = m_th_stdyaw_err_3d;
            break;
        case 2:
            th_stdxy_warn_active = m_th_stdxy_warn_2d;
            th_stdxy_err_active = m_th_stdxy_err_2d;
            th_stdyaw_warn_active = m_th_stdyaw_warn_2d;
            th_stdyaw_err_active = m_th_stdyaw_err_2d;
            break;
        case 1: 
            th_stdxy_warn_active = m_th_stdxy_warn_1d;
            th_stdxy_err_active = m_th_stdxy_err_1d;
            th_stdyaw_warn_active = m_th_stdyaw_warn_1d;
            th_stdyaw_err_active = m_th_stdyaw_err_1d;
            break;
        case 0: 
            th_stdxy_warn_active = m_th_stdxy_warn_0d;
            th_stdxy_err_active = m_th_stdxy_err_0d;
            th_stdyaw_warn_active = m_th_stdyaw_warn_0d;
            th_stdyaw_err_active = m_th_stdyaw_err_0d;
            break;
    }
    
    double r_stdxy = clamp01((filtered_stdxy - th_stdxy_warn_active) / std::max(1e-6, th_stdxy_warn_active));
    double r_stdyaw= clamp01((filtered_stdyaw - th_stdyaw_warn_active) / std::max(1e-6, th_stdyaw_warn_active));

    double risk = m_w_score*r_score + m_w_uvw0*r_uvw0 + m_w_uvw1*r_uvw1 + m_w_stdxy*r_stdxy + m_w_stdyaw*r_stdyaw;

    bool hard_error = (filtered_score < m_th_score_err) || 
                     (filtered_uvw0 < m_th_uvw0_err) || 
                     (filtered_uvw1 < m_th_uvw1_err) ||
                     (filtered_stdxy > th_stdxy_err_active) || 
                     (filtered_stdyaw > th_stdyaw_err_active);
    if (hard_error) risk = std::max(risk, m_risk_err);

    // 使用局部锁保护证据积分器状态更新
    {
      std::lock_guard<std::mutex> light_lock(m_node_mutex);
      
      const double over  = std::max(0.0, risk - m_risk_clear) / std::max(1e-6, m_risk_err - m_risk_clear);
      const double under = std::max(0.0, m_risk_clear - risk) / std::max(1e-6, m_risk_clear);
      
      // 强制模式降级
      if ((risk > m_risk_warn * 1.2 || m_evidence > m_e_warn * 1.1) && mode_snapshot > 0) {
        int target_mode = mode_snapshot - 1;
        
        if (risk > m_risk_err * 0.9 || m_evidence > m_e_err * 0.9 || hard_error) {
          target_mode = 0;
        }
        
        if (target_mode < m_mode) {
          m_mode = target_mode;
          ROS_WARN("Forced mode downgrade to %dD due to high risk/evidence: risk=%.2f, evidence=%.2f", 
                  m_mode, risk, m_evidence);
        }
      }
      
      // 加强恢复能力
      if (risk < m_risk_clear * 0.7 && filtered_score > m_th_score_warn) {
        if (mode_snapshot >= 2 || grad_std_uvw_snapshot[2] > m_constrain_threshold_yaw * 1.1) {
          // 良好的yaw约束也可触发快速恢复
          m_recovery_count++;
          
          if (m_recovery_count >= 15) { // 缩短恢复所需帧数
            m_evidence *= 0.15; // 更快的恢复速度
            ROS_INFO_THROTTLE(2.0, "Strong recovery: evidence reduced to %.3f", m_evidence);
            m_recovery_count = 0;
          }
        }
      } else {
        m_recovery_count = 0;
      }
      
      m_evidence += m_e_up * over - m_e_down * under;
      m_evidence = clamp01(m_evidence);
                
      // 重定位与恢复
      if ((mode_snapshot == 3 || (mode_snapshot == 2 && grad_std_uvw_snapshot[2] > m_constrain_threshold_yaw * 1.2))
          && m_mode < 2 && filtered_score > m_th_score_warn * 1.2) {
        // 强yaw约束的2D模式也能触发重定位检测
        m_evidence *= 0.2; // 更快的恢复
        ROS_WARN("Relocation detected: fast recovery to evidence = %.3f", m_evidence);
      }
    }

    int level_val = 1; // 1=正常, 2=警告, 3=错误
    if (m_evidence >= m_e_err)      level_val = 3;
    else if (m_evidence >= m_e_warn) level_val = 2;

    filtered_msg.abnormal = (level_val != 1);
    filtered_msg.level = static_cast<uint8_t>(level_val);

    char buf2[256];
    
    char error_code = '0';
    if (filtered_score < m_th_score_warn) error_code = 'S';            // Score
    else if (filtered_uvw0 < m_th_uvw0_warn || filtered_uvw1 < m_th_uvw1_warn) error_code = 'G';  // Gradient
    else if (filtered_stdxy > th_stdxy_warn_active) error_code = 'P';  // Position
    else if (filtered_stdyaw > th_stdyaw_warn_active) error_code = 'Y'; // Yaw
    
    if (level_val == 3) {
      snprintf(buf2, sizeof(buf2), 
               "[KF] ERR(M%d|%c): r=%.2f e=%.2f (score=%.2f, uvw=[%.2f,%.2f], xy=%.2f/%.2f, yaw=%.2f/%.2f)",
               mode_snapshot, error_code, risk, m_evidence, 
               filtered_score, filtered_uvw0, filtered_uvw1, 
               filtered_stdxy, th_stdxy_err_active, filtered_stdyaw, th_stdyaw_err_active);
      filtered_msg.message = std::string(buf2);
    } else if (level_val == 2) {
      snprintf(buf2, sizeof(buf2), 
               "[KF] WARN(M%d|%c): r=%.2f e=%.2f (score=%.2f, uvw=[%.2f,%.2f], xy=%.2f/%.2f, yaw=%.2f/%.2f)",
               mode_snapshot, error_code, risk, m_evidence, 
               filtered_score, filtered_uvw0, filtered_uvw1, 
               filtered_stdxy, th_stdxy_warn_active, filtered_stdyaw, th_stdyaw_warn_active);
      filtered_msg.message = std::string(buf2);
    } else {
      filtered_msg.message = "";
    }

    filtered_msg.risk = static_cast<float>(m_evidence);
    m_pub_stats_filtered.publish(filtered_msg);

    // 错误监控处理
    if (m_err_client && level_val != m_last_level) {
      if (level_val == 1) {
        // 恢复正常: 注销34100003
        m_err_client->unregisterErrorMsg(34100003);
      } else if (level_val == 2) {
        // 警告级别: 不上报错误码，仅准备后续切换odom源
        // 如果之前有错误码，先注销
        m_err_client->unregisterErrorMsg(34100003);
      } else if (level_val == 3) {
        m_err_client->unregisterErrorMsg(34100003);
        m_err_client->registerErrorMsg(34100003, 1, "localization error");
      }
      
      // 安全更新状态等级
      std::lock_guard<std::mutex> light_lock(m_node_mutex);
      m_last_level = level_val;
    }
  }

  /*
   * Resets localization to given position.
   */
  void pose_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    {
      std::lock_guard<std::mutex> lock(m_node_mutex);

      if (pose->header.frame_id != m_map_frame) {
        ROS_WARN_STREAM("NeoLocalizationNode: Invalid pose estimate frame: "
                        << pose->header.frame_id);
        return;
      }

      tf::Transform map_pose;
      tf::poseMsgToTF(pose->pose.pose, map_pose);

      // ROS_INFO_STREAM("NeoLocalizationNode: Got new map pose estimate: x="
      //                 << map_pose.getOrigin()[0]
      //                 << " m, y=" << map_pose.getOrigin()[1] << " m, yaw="
      //                 << tf::getYaw(map_pose.getRotation()) << " rad");

      tf::StampedTransform base_to_odom;
      try {
        m_tf.lookupTransform(m_odom_frame, m_base_frame, ros::Time(),
                             base_to_odom);
      } catch (const std::exception &ex) {
        ROS_WARN_STREAM("NeoLocalizationNode: lookupTransform(m_base_frame, "
                        "m_odom_frame) failed: "
                        << ex.what());
        return;
      }

      const Matrix<double, 4, 4> L = convert_transform_25(base_to_odom);

      // compute new odom to map offset
      const Matrix<double, 3, 1> new_offset =
          (convert_transform_25(map_pose) * L.inverse() *
           Matrix<double, 4, 1>{0, 0, 0, 1})
              .project();

      // set new offset based on given position
      m_offset_x = new_offset[0];
      m_offset_y = new_offset[1];
      m_offset_yaw = new_offset[2];

      // reset particle spread to maximum
      m_sample_std_xy = m_max_sample_std_xy;
      m_sample_std_yaw = m_max_sample_std_yaw;

      broadcast();
    }

    // get a new map tile immediately
    update_map();
  }

  /*
   * Sets initial pose from parameters (x, y, yaw in map frame)
   */
  void set_initial_pose(double x, double y, double yaw) {
    std::lock_guard<std::mutex> lock(m_node_mutex);

    // Reset sample standard deviations to maximum
    m_sample_std_xy = m_max_sample_std_xy;
    m_sample_std_yaw = m_max_sample_std_yaw;

    // Set initial offset values (these represent the transform from odom to
    // map)
    m_offset_x = x;
    m_offset_y = y;
    m_offset_yaw = yaw;

    // Reset odometry pose tracking
    m_last_odom_pose = Matrix<double, 3, 1>{0, 0, 0};

    // Reset update counter
    update_counter = 0;

    ROS_INFO("Initial pose set to: x=%.3f, y=%.3f, yaw=%.3f", x, y, yaw);
  }

  /*
   * Stores the given map.
   */
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &ros_map) {
    std::lock_guard<std::mutex> lock(m_node_mutex);

    ROS_INFO_STREAM("NeoLocalizationNode: Got new map with dimensions "
                    << ros_map->info.width << " x " << ros_map->info.height
                    << " and cell size " << ros_map->info.resolution);

    {
      tf::Transform tmp;
      tf::poseMsgToTF(ros_map->info.origin, tmp);
      m_world_to_map = convert_transform_25(tmp);
    }
    m_world = ros_map;

    // reset particle spread to maximum
    m_sample_std_xy = m_max_sample_std_xy;
    m_sample_std_yaw = m_max_sample_std_yaw;
  }

  /*
   * Extracts a new map tile around current position.
   */
  void update_map() {
    Matrix<double, 4, 4> world_to_map; // transformation from original grid map
                                       // (integer coords) to "map frame"
    Matrix<double, 3, 1> world_pose;   // pose in the original (integer coords)
                                       // grid map (not map tile)
    tf::StampedTransform base_to_odom;
    nav_msgs::OccupancyGrid::ConstPtr world;
    {
      std::lock_guard<std::mutex> lock(m_node_mutex);
      if (!m_world) {
        return;
      }

      try {
        m_tf.lookupTransform(m_odom_frame, m_base_frame, ros::Time(),
                             base_to_odom);
      } catch (const std::exception &ex) {
        ROS_WARN_STREAM("NeoLocalizationNode: lookupTransform(m_base_frame, "
                        "m_odom_frame) failed: "
                        << ex.what());
        return;
      }

      const Matrix<double, 4, 4> L = convert_transform_25(base_to_odom);
      const Matrix<double, 4, 4> T = translate25(m_offset_x, m_offset_y) *
                                     rotate25_z(m_offset_yaw); // odom to map
      world_pose =
          (m_world_to_map.inverse() * T * L * Matrix<double, 4, 1>{0, 0, 0, 1})
              .project();

      world = m_world;
      world_to_map = m_world_to_map;
    }

    // compute tile origin in pixel coords
    const double world_scale = world->info.resolution;
    const int tile_x = int(world_pose[0] / world_scale) - m_map_size / 2;
    const int tile_y = int(world_pose[1] / world_scale) - m_map_size / 2;

    auto map =
        std::make_shared<GridMap<float>>(m_map_size, m_map_size, world_scale);

    // extract tile and convert to our format (occupancy between 0 and 1)
    for (int y = 0; y < map->size_y(); ++y) {
      for (int x = 0; x < map->size_x(); ++x) {
        const int x_ =
            std::min(std::max(tile_x + x, 0), int(world->info.width) - 1);
        const int y_ =
            std::min(std::max(tile_y + y, 0), int(world->info.height) - 1);
        const auto cell = world->data[y_ * world->info.width + x_];
        if (cell >= 0) {
          (*map)(x, y) = fminf(cell / 100.f, 1.f);
        } else {
          (*map)(x, y) = 0;
        }
      }
    }

    // optionally downscale map
    for (int i = 0; i < m_map_downscale; ++i) {
      map = map->downscale();
    }

    // smooth map
    for (int i = 0; i < m_num_smooth; ++i) {
      map->smooth_33_1();
    }

    // update map
    {
      std::lock_guard<std::mutex> lock(m_node_mutex);
      m_map = map;
      m_grid_to_map = world_to_map * translate25<double>(tile_x * world_scale,
                                                         tile_y * world_scale);
    }

    const auto tile_origin =
        (m_grid_to_map * Matrix<double, 4, 1>{0, 0, 0, 1}).project();
    const auto tile_center =
        (m_grid_to_map * Matrix<double, 4, 1>{map->scale() * map->size_x() / 2,
                                              map->scale() * map->size_y() / 2,
                                              0, 1})
            .project();

    // publish new map tile for visualization
    auto ros_grid = boost::make_shared<nav_msgs::OccupancyGrid>();
    ros_grid->info.resolution = map->scale();
    ros_grid->info.width = map->size_x();
    ros_grid->info.height = map->size_y();
    ros_grid->info.origin.position.x = tile_origin[0];
    ros_grid->info.origin.position.y = tile_origin[1];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(tile_origin[2]),
                          ros_grid->info.origin.orientation);
    ros_grid->data.resize(map->num_cells());
    for (int y = 0; y < map->size_y(); ++y) {
      for (int x = 0; x < map->size_x(); ++x) {
        ros_grid->data[y * map->size_x() + x] = (*map)(x, y) * 100.f;
      }
    }
    m_pub_map_tile.publish(ros_grid);

    ROS_DEBUG_STREAM("NeoLocalizationNode: Got new grid at offset ("
                     << tile_x << ", " << tile_y
                     << ") [iworld], "
                        "center = ("
                     << tile_center[0] << ", " << tile_center[1] << ") [map]");
  }

  /*
   * Asynchronous map update loop, running in separate thread.
   */
  void update_loop() {
    ros::Rate rate(m_map_update_rate);
    while (ros::ok()) {
      try {
        update_map(); // get a new map tile periodically
      } catch (const std::exception &ex) {
        ROS_WARN_STREAM(
            "NeoLocalizationNode: update_map() failed: " << ex.what());
      }
      rate.sleep();
    }
  }

  /*
   * Publishes "map" frame on tf.
   */
  void broadcast() {
    if (m_broadcast_tf) {
      // compose and publish transform for tf package
      geometry_msgs::TransformStamped pose;
      // compose header
      pose.header.stamp = m_offset_time;
      pose.header.frame_id = m_map_frame;
      pose.child_frame_id = m_odom_pub_frame;
      // compose data container
      pose.transform.translation.x = m_offset_x;
      pose.transform.translation.y = m_offset_y;
      pose.transform.translation.z = 0;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(m_offset_yaw),
                            pose.transform.rotation);

      // publish the transform
      m_tf_broadcaster.sendTransform(pose);
    }
  }

  bool fix_mon_cb(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
    std::lock_guard<std::mutex> lock(m_node_mutex);
    m_evidence = 0.0;
    m_last_level = 1;
    // 注销定位风险错误码 34100003
    if (m_err_client) {
      m_err_client->unregisterErrorMsg(34100003);
    }
    if (kf_score.isInitialized()) kf_score.reset(0.0);
    if (kf_uvw0.isInitialized()) kf_uvw0.reset(0.0);
    if (kf_uvw1.isInitialized()) kf_uvw1.reset(0.0);
    if (kf_stdxy.isInitialized()) kf_stdxy.reset(0.0);
    if (kf_stdyaw.isInitialized()) kf_stdyaw.reset(0.0);
    ROS_INFO("neo_localization fix_mon: reset risk state");
    return true;
  }

private:
  std::mutex m_node_mutex;

  ros::NodeHandle m_node_handle; // Now this is a private node handle

  ros::Publisher m_pub_map_tile;
  ros::Publisher m_pub_loc_pose;
  ros::Publisher m_pub_loc_pose_2;
  ros::Publisher m_pub_pose_array;
	ros::Publisher m_pub_stats;
  ros::Publisher m_pub_stats_filtered;

  ros::Subscriber m_sub_map_topic;
  ros::Subscriber m_sub_scan_topic;
  ros::Subscriber m_sub_pose_estimate;

  ros::Timer m_loc_update_timer;

  tf::TransformListener m_tf;
  tf::TransformBroadcaster m_tf_broadcaster;

  bool m_broadcast_tf = false;
  std::string m_base_frame;
  std::string m_odom_frame;
  std::string m_odom_pub_frame;
  std::string m_map_frame;

  int m_map_size = 0;
  int m_map_downscale = 0;
  int m_num_smooth = 0;
  int m_solver_iterations = 0;
  int m_sample_rate = 0;
  int m_min_points = 0;
  double m_update_gain = 0;
  double m_confidence_gain = 0;
  double m_min_score = 0;
  double m_odometry_std_xy = 0;  double m_odometry_std_yaw = 0;  double m_min_sample_std_xy = 0;
  double m_min_sample_std_yaw = 0;
  double m_max_sample_std_xy = 0;
  double m_max_sample_std_yaw = 0;
  double m_constrain_threshold = 0;
  double m_constrain_threshold_yaw = 0;
  double m_loc_update_rate = 0;
  double m_map_update_rate = 0;
  double m_transform_timeout = 0;
  
  // 阈值参数
  double m_th_score_warn = 0.55, m_th_score_err = 0.25;
  double m_th_uvw0_warn = 0.25, m_th_uvw0_err = 0.10;
  double m_th_uvw1_warn = 0.20, m_th_uvw1_err = 0.08;
  // 基础阈值
  double m_th_stdxy_warn = 0.20, m_th_stdxy_err = 0.30;
  double m_th_stdyaw_warn = 0.20, m_th_stdyaw_err = 0.30;
  
  // 模式特定阈值 - 调整以避免0.25附近的抖动
  double m_th_stdxy_warn_3d = 0.15, m_th_stdxy_err_3d = 0.27;  // 3D模式下严格
  double m_th_stdyaw_warn_3d = 0.15, m_th_stdyaw_err_3d = 0.27;
  
  double m_th_stdxy_warn_2d = 0.19, m_th_stdxy_err_2d = 0.29;  // 2D模式
  double m_th_stdyaw_warn_2d = 0.18, m_th_stdyaw_err_2d = 0.28;
  
  double m_th_stdxy_warn_1d = 0.235, m_th_stdxy_err_1d = 0.34;  // 1D模式 - 避开0.25临界点
  double m_th_stdyaw_warn_1d = 0.235, m_th_stdyaw_err_1d = 0.34;
  
  double m_th_stdxy_warn_0d = 0.32, m_th_stdxy_err_0d = 0.42;  // 0D模式最宽松
  double m_th_stdyaw_warn_0d = 0.32, m_th_stdyaw_err_0d = 0.42;

  // 指标权重与风险阈值
  double m_w_score = 0.6, m_w_uvw0 = 0.15, m_w_uvw1 = 0.15, m_w_stdxy = 0.05, m_w_stdyaw = 0.05;
  double m_risk_clear = 0.20;
  double m_risk_warn = 0.3, m_risk_err = 0.6;

  // 证据积分状态与阈值
  double m_evidence = 0.0;   
  double m_e_up = 0.15;      // 增长速率
  double m_e_down = 0.10;    // 衰减速率
  double m_e_warn = 0.30;     // WARN触发阈值
  double m_e_err = 0.65;      // ERROR触发阈值
  
  // 模式稳定性相关
  int m_mode = 0;                 // 当前模式状态(0D-3D)
  int m_stable_mode_count = 0;    // 模式稳定计数器
  int m_recovery_count = 0;       // 恢复计数器
  int m_stable_mode_threshold = 3; // 模式稳定所需的连续帧数
  double m_mode_hysteresis = 0.05; // 模式判断滞后阈值

  ros::Time m_offset_time;
  double m_offset_x = 0;  double m_offset_y = 0;  double m_offset_yaw = 0;  double m_sample_std_xy = 0;  double m_sample_std_yaw = 0;
  Matrix<double, 3, 1> m_last_odom_pose;
  Matrix<double, 4, 4> m_grid_to_map;
  Matrix<double, 4, 4> m_world_to_map;
  std::shared_ptr<GridMap<float>> m_map;  nav_msgs::OccupancyGrid::ConstPtr m_world;
  int64_t update_counter = 0;
  std::map<std::string, sensor_msgs::LaserScan::ConstPtr> m_scan_buffer;

  Solver m_solver;
  std::mt19937 m_generator;
  std::thread m_map_update_thread;

  KalmanFilter kf_score;
  KalmanFilter kf_uvw0;
  KalmanFilter kf_uvw1;
  KalmanFilter kf_stdxy;
  KalmanFilter kf_stdyaw;

  // 错误监控
  std::shared_ptr<cyanine_os::error_monitor::ErrorMonitorClient> m_err_client;
  int m_last_level = 1; // 1=正常 2=警告 3=错误
  ros::ServiceServer m_srv_fix_mon;
};

int main(int argc, char **argv) {
  // initialize ROS
  ros::init(argc, argv, "neo_localization_node");

  try {
    NeoLocalizationNode node;

    ros::spin();
  } catch (const std::exception &ex) {
    ROS_ERROR_STREAM("NeoLocalizationNode: " << ex.what());
    return -1;
  }

  return 0;
}
