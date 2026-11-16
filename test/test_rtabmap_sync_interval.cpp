#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_sync/CommonDataSubscriber.h>
#include <rtabmap_msgs/msg/rgbd_image.hpp>

using namespace std::chrono_literals;

class Probe final : public rtabmap_sync::CommonDataSubscriber
{
public:
  explicit Probe(rclcpp::Node & node) : CommonDataSubscriber(node, false) {setupCallbacks(node);}
  std::vector<std::pair<double, double>> pairs;

protected:
  void commonMultiCameraCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &,
    const rtabmap_msgs::msg::UserData::ConstSharedPtr &,
    const std::vector<cv_bridge::CvImageConstPtr> & images,
    const std::vector<cv_bridge::CvImageConstPtr> &,
    const std::vector<sensor_msgs::msg::CameraInfo> &,
    const std::vector<sensor_msgs::msg::CameraInfo> &,
    const sensor_msgs::msg::LaserScan &,
    const sensor_msgs::msg::PointCloud2 & scan,
    const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr &,
    const std::vector<rtabmap_msgs::msg::GlobalDescriptor> &,
    const std::vector<std::vector<rtabmap_msgs::msg::KeyPoint>> &,
    const std::vector<std::vector<rtabmap_msgs::msg::Point3f>> &,
    const std::vector<cv::Mat> &) override
  {
    pairs.emplace_back(rclcpp::Time(images.at(0)->header.stamp).seconds(),
      rclcpp::Time(scan.header.stamp).seconds());
  }
  void commonLaserScanCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &,
    const rtabmap_msgs::msg::UserData::ConstSharedPtr &,
    const sensor_msgs::msg::LaserScan &,
    const sensor_msgs::msg::PointCloud2 &,
    const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr &,
    const rtabmap_msgs::msg::GlobalDescriptor &) override {}
  void commonOdomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &,
    const rtabmap_msgs::msg::UserData::ConstSharedPtr &,
    const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr &) override {}
  void commonSensorDataCallback(
    const rtabmap_msgs::msg::SensorData::ConstSharedPtr &,
    const nav_msgs::msg::Odometry::ConstSharedPtr &,
    const rtabmap_msgs::msg::OdomInfo::ConstSharedPtr &) override {}
};

rclcpp::NodeOptions options(double interval, bool exact = false)
{
  rclcpp::NodeOptions result;
  result.parameter_overrides({
    rclcpp::Parameter("subscribe_depth", false),
    rclcpp::Parameter("subscribe_rgb", false),
    rclcpp::Parameter("subscribe_rgbd", true),
    rclcpp::Parameter("subscribe_scan_cloud", true),
    rclcpp::Parameter("subscribe_odom", false),
    rclcpp::Parameter("approx_sync", !exact),
    rclcpp::Parameter("approx_sync_max_interval", interval)});
  return result;
}

bool exercise(double limit, double skew, bool expect, bool exact = false)
{
  auto node = std::make_shared<rclcpp::Node>("sync_interval_probe", options(limit, exact));
  Probe probe(*node);
  if (!node->has_parameter("approx_sync_max_interval")) {
    std::cerr << "native subscriber ignored approx_sync_max_interval\n";
    return false;
  }
  if (!node->describe_parameter("approx_sync_max_interval").read_only) {return false;}
  auto rgbd = node->create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_image", 10);
  auto cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_for = [&](std::chrono::milliseconds duration) {
      const auto until = std::chrono::steady_clock::now() + duration;
      do {executor.spin_some(); std::this_thread::sleep_for(2ms);}
      while (std::chrono::steady_clock::now() < until);
    };
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (rgbd->get_subscription_count() == 0 || cloud->get_subscription_count() == 0) {
    if (std::chrono::steady_clock::now() >= deadline) {return false;}
    spin_for(10ms);
  }
  // A continuing sequence forces approximate-sync candidates to resolve rather
  // than mistake a pending last pair for rejection. No clock or TF is needed.
  for (int i = 0; i < 12; ++i) {
    rtabmap_msgs::msg::RGBDImage image;
    image.header.stamp = rclcpp::Time(100000000000LL + i * 1000000000LL);
    image.header.frame_id = "camera";
    image.rgb.header = image.depth.header = image.header;
    image.rgb.height = image.rgb.width = image.depth.height = image.depth.width = 2;
    image.rgb.encoding = "rgb8";
    image.rgb.step = 6;
    image.rgb.data.resize(12);
    image.depth.encoding = "16UC1";
    image.depth.step = 4;
    image.depth.data.resize(8);
    image.rgb_camera_info.header = image.depth_camera_info.header = image.header;
    image.rgb_camera_info.width = image.rgb_camera_info.height = 2;
    image.rgb_camera_info.k = {1, 0, 1, 0, 1, 1, 0, 0, 1};
    image.rgb_camera_info.p = {1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0};
    image.depth_camera_info = image.rgb_camera_info;
    sensor_msgs::msg::PointCloud2 scan;
    scan.header = image.header;
    scan.header.stamp = rclcpp::Time(image.header.stamp) + rclcpp::Duration::from_seconds(skew);
    scan.header.frame_id = "scan";
    scan.height = scan.width = 1;
    scan.point_step = scan.row_step = 12;
    scan.data.resize(12);
    rgbd->publish(image);
    cloud->publish(scan);
    spin_for(30ms);
  }
  spin_for(100ms);
  const bool received = !probe.pairs.empty();
  bool correct = received == expect;
  for (const auto & pair : probe.pairs) {
    correct &= std::abs((pair.second - pair.first) - skew) < 1e-6;
    if (limit > 0.0) {correct &= std::abs(pair.second - pair.first) <= limit + 1e-9;}
  }
  std::cout << "limit=" << limit << " skew=" << skew << " exact=" << exact
            << " callbacks=" << probe.pairs.size() << " pass=" << correct << '\n';
  executor.remove_node(node);
  return correct;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool passed = true;
  for (double invalid : {-0.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    bool rejected = false;
    try {
      auto node = std::make_shared<rclcpp::Node>("invalid_sync_interval", options(invalid));
      Probe probe(*node);
    } catch (const std::invalid_argument &) {rejected = true;}
    passed &= rejected;
  }
  passed &= exercise(0.0, 0.4, true);
  passed &= exercise(0.15, 0.4, false);
  passed &= exercise(0.15, 0.05, true);
  passed &= exercise(0.15, 0.15, true);
  passed &= exercise(0.15, 0.0, true, true);
  passed &= exercise(0.15, 0.05, false, true);
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
