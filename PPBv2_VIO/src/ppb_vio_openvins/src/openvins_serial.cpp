// Offline ("serial") OpenVINS runner for the PPBv2 VIO bench.
//
// Reads a ROS 2 bag on the /vio/* contract in stamp order and feeds every IMU
// sample and stereo pair to OpenVINS through the same ROS2Visualizer callbacks
// the live node (run_subscribe_msckf) uses, single-threaded. Nothing goes
// through DDS, so no message can be dropped and a run is repeatable: the
// result depends on the data and the config, not on how loaded the laptop is.
//
// After every filter update the IMU state (OpenVINS public State API) is
// written to the output bag as nav_msgs/Odometry, in the same convention as
// the live /ov_msckf/odomimu topic that vio_odom_adapter expects:
//   pose        p_IinG, R_ItoG (Hamilton, x y z w)
//   covariance  [position in G, orientation error in I] (right perturbation)
//   twist       linear: IMU velocity in the IMU frame; angular: gyro - bias_g
// Per-update processing time goes to <out>/timing.csv (real-time factor).
//
// Usage:
//   openvins_serial <estimator_config.yaml> <input_bag> <output_bag_dir> [--odom-topic T]

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS2Visualizer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

namespace {

// Stereo frames are paired by identical header stamps (the /vio/* contract
// requires it); a frame without a partner is skipped, as OpenVINS' own ROS 1
// serial runner does.
struct PendingStereo {
  sensor_msgs::msg::Image::SharedPtr left, right;
};

nav_msgs::msg::Odometry state_to_odometry(const std::shared_ptr<ov_msckf::State> &state,
                                          const Eigen::Vector3d &gyro_raw) {
  const auto &imu = state->_imu;
  const Eigen::Matrix3d R_GtoI = imu->Rot();            // JPL: rotation from G to I
  const Eigen::Quaterniond q_ItoG(R_GtoI.transpose());  // Hamilton R_ItoG

  nav_msgs::msg::Odometry odom;
  const long long ns = std::llround(state->_timestamp * 1e9);
  odom.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  odom.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  odom.header.frame_id = "global";
  odom.child_frame_id = "imu";
  odom.pose.pose.position.x = imu->pos()(0);
  odom.pose.pose.position.y = imu->pos()(1);
  odom.pose.pose.position.z = imu->pos()(2);
  odom.pose.pose.orientation.x = q_ItoG.x();
  odom.pose.pose.orientation.y = q_ItoG.y();
  odom.pose.pose.orientation.z = q_ItoG.z();
  odom.pose.pose.orientation.w = q_ItoG.w();

  // OpenVINS orders the pose error state [theta (in I), p (in G)]; the
  // Odometry convention is [position, orientation].
  std::vector<std::shared_ptr<ov_type::Type>> pose_vars = {imu->pose()};
  const Eigen::MatrixXd P = ov_msckf::StateHelper::get_marginal_covariance(state, pose_vars);
  Eigen::Matrix<double, 6, 6> P_out;
  P_out.block<3, 3>(0, 0) = P.block<3, 3>(3, 3);
  P_out.block<3, 3>(0, 3) = P.block<3, 3>(3, 0);
  P_out.block<3, 3>(3, 0) = P.block<3, 3>(0, 3);
  P_out.block<3, 3>(3, 3) = P.block<3, 3>(0, 0);
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      odom.pose.covariance[6 * r + c] = P_out(r, c);

  const Eigen::Vector3d v_I = R_GtoI * imu->vel();
  const Eigen::Vector3d w_I = gyro_raw - imu->bias_g();
  odom.twist.twist.linear.x = v_I(0);
  odom.twist.twist.linear.y = v_I(1);
  odom.twist.twist.linear.z = v_I(2);
  odom.twist.twist.angular.x = w_I(0);
  odom.twist.twist.angular.y = w_I(1);
  odom.twist.twist.angular.z = w_I(2);
  std::vector<std::shared_ptr<ov_type::Type>> twist_vars = {imu->v(), imu->bg()};
  const Eigen::MatrixXd Q = ov_msckf::StateHelper::get_marginal_covariance(state, twist_vars);
  Eigen::Matrix<double, 6, 6> T = Eigen::Matrix<double, 6, 6>::Zero();
  T.block<3, 3>(0, 0) = R_GtoI * Q.block<3, 3>(0, 0) * R_GtoI.transpose();
  T.block<3, 3>(3, 3) = Q.block<3, 3>(3, 3);
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      odom.twist.covariance[6 * r + c] = T(r, c);
  return odom;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s <estimator_config.yaml> <input_bag> <output_bag_dir> [--odom-topic T]\n", argv[0]);
    return 2;
  }
  const std::string config_path = argv[1], in_bag = argv[2], out_dir = argv[3];
  std::string odom_topic = "/ov_msckf/odomimu";
  for (int i = 4; i + 1 < argc; i += 2)
    if (std::string(argv[i]) == "--odom-topic")
      odom_topic = argv[i + 1];

  rclcpp::init(1, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("openvins_serial", options);

  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  // Repeatability: everything on this thread, deterministic feature extraction.
  params.use_multi_threading_subs = false;
  params.use_multi_threading_pubs = false;
  params.num_opencv_threads = 0;
  const int num_cameras = params.state_options.num_cameras;
  if (num_cameras != 1 && num_cameras != 2) {
    std::fprintf(stderr, "openvins_serial: only mono (cam0) or stereo is supported\n");
    return 2;
  }
  std::string topic_imu = "/imu0";
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  std::string topic_cam[2] = {"/cam0/image_raw", "/cam1/image_raw"};
  for (int i = 0; i < num_cameras; i++)
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", topic_cam[i]);
  if (!parser->successful()) {
    std::fprintf(stderr, "openvins_serial: unable to parse all parameters\n");
    return 2;
  }

  auto sys = std::make_shared<ov_msckf::VioManager>(params);
  auto viz = std::make_shared<ov_msckf::ROS2Visualizer>(node, sys);

  rosbag2_cpp::Reader reader;
  reader.open(in_bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic_imu, topic_cam[0]};
  if (num_cameras == 2)
    filter.topics.push_back(topic_cam[1]);
  reader.set_filter(filter);

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions out_opts;
  out_opts.uri = out_dir + "/odom";
  out_opts.storage_id = "mcap";
  writer.open(out_opts, {"cdr", "cdr"});
  writer.create_topic({odom_topic, "nav_msgs/msg/Odometry", "cdr", ""});
  std::ofstream timing(out_dir + "/timing.csv");
  timing << "stamp_s,process_s\n";

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;
  std::map<int64_t, PendingStereo> pending;
  Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();
  double last_written = -1.0, first_data = -1.0, last_data = -1.0;
  long n_imu = 0, n_pairs = 0, n_unpaired = 0, n_written = 0;
  const auto wall0 = std::chrono::steady_clock::now();

  while (reader.has_next() && rclcpp::ok()) {
    auto bag_msg = reader.read_next();
    rclcpp::SerializedMessage ser(*bag_msg->serialized_data);
    if (bag_msg->topic_name == topic_imu) {
      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      imu_ser.deserialize_message(&ser, msg.get());
      const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      if (first_data < 0)
        first_data = t;
      last_data = t;
      last_gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
      const auto t0 = std::chrono::steady_clock::now();
      viz->callback_inertial(msg);  // runs any queued camera updates (single-threaded)
      n_imu++;
      if (sys->initialized() && sys->get_state()->_timestamp > last_written) {
        const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        auto odom = state_to_odometry(sys->get_state(), last_gyro);
        writer.write(odom, odom_topic, rclcpp::Time(odom.header.stamp));
        timing << std::fixed << sys->get_state()->_timestamp << "," << dt << "\n";
        last_written = sys->get_state()->_timestamp;
        n_written++;
      }
      continue;
    }
    const int cam = bag_msg->topic_name == topic_cam[0] ? 0 : 1;
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img_ser.deserialize_message(&ser, img.get());
    if (num_cameras == 1) {  // diagnostic mono mode: left camera only
      viz->callback_monocular(img, 0);
      n_pairs++;
      continue;
    }
    const int64_t key = static_cast<int64_t>(img->header.stamp.sec) * 1000000000LL + img->header.stamp.nanosec;
    auto &slot = pending[key];
    (cam == 0 ? slot.left : slot.right) = img;
    if (slot.left && slot.right) {
      viz->callback_stereo(slot.left, slot.right, 0, 1);
      n_pairs++;
      pending.erase(key);
    }
    // A frame whose partner never came: anything much older than this stamp.
    while (!pending.empty() && pending.begin()->first < key - 100000000LL) {
      pending.erase(pending.begin());
      n_unpaired++;
    }
  }
  n_unpaired += static_cast<long>(pending.size());

  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
  const double data = last_data - first_data;
  std::FILE *summary = std::fopen((out_dir + "/serial_summary.json").c_str(), "w");
  std::fprintf(summary,
               "{\n  \"imu_messages\": %ld,\n  \"stereo_pairs\": %ld,\n  \"unpaired_frames\": %ld,\n"
               "  \"odometry_written\": %ld,\n  \"initialized\": %s,\n  \"data_seconds\": %.3f,\n"
               "  \"wall_seconds\": %.3f,\n  \"realtime_factor\": %.3f\n}\n",
               n_imu, n_pairs, n_unpaired, n_written, sys->initialized() ? "true" : "false", data, wall,
               wall > 0 ? data / wall : 0.0);
  std::fclose(summary);
  std::printf("openvins_serial: %ld IMU, %ld stereo pairs (%ld unpaired), %ld states written, "
              "%.1f s of data in %.1f s (%.2fx real time)\n",
              n_imu, n_pairs, n_unpaired, n_written, data, wall, wall > 0 ? data / wall : 0.0);
  rclcpp::shutdown();
  return sys->initialized() ? 0 : 3;
}
