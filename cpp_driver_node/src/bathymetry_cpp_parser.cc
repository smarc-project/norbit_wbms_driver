#include <memory>

#include "norbit_wbms_interfaces/msg/bathymetry.hpp"
#include "norbit_wbms_interfaces/msg/bathymetry_beam.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace {

constexpr char kNodeName[] = "bathymetry_cpp_parse";

// Relevant subscription and publishing topics.
constexpr char kBathymetryTopic[] = "/lolo/sensors/mbes/bathymetry";
constexpr char kPointsTopic[] = "bathymetry/points_cpp";
constexpr char kPcdFrameId[] = "lolo/base_link";  // TODO: parse somehow.

// We have 4 fields per beam: (x, y, z) and intensity.
constexpr int kFieldsPerBeam = 4;

// For convenience and readability.
using Bathymetry = norbit_wbms_interfaces::msg::Bathymetry;
using BathymetryBeam = norbit_wbms_interfaces::msg::BathymetryBeam;

sensor_msgs::msg::PointField MakePointField(const std::string& name,
                                            uint32_t offset) {
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  return field;
}

}  // namespace

class BathymetryParser : public rclcpp::Node {
 public:
  BathymetryParser() : Node(kNodeName) {
    RCLCPP_INFO(this->get_logger(), "[BathymetryParser] Starting the C++ node");

    sub_bathymetry_ = this->create_subscription<Bathymetry>(
        kBathymetryTopic, 1,
        std::bind(&BathymetryParser::BathymetryCallback, this,
                  std::placeholders::_1));
    pub_pcd_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(kPointsTopic, 2);
  }

 private:
  void BathymetryCallback(const Bathymetry::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 pcd_msg;
    const int num_beams = msg->beams.size();

    // Build the header.
    std_msgs::msg::Header& header = pcd_msg.header;
    header.frame_id = kPcdFrameId;
    header.stamp = this->get_clock()->now();

    // Define the data layout.
    pcd_msg.height = 1;  // Unordered.
    pcd_msg.width = num_beams;
    pcd_msg.fields = {
        MakePointField(/*name=*/"x", /*offset=*/0),
        MakePointField(/*name=*/"y", /*offset=*/4),
        MakePointField(/*name=*/"z", /*offset=*/8),
        MakePointField(/*name=*/"intensity", /*offset=*/12),
    };

    // Build the point cloud data.
    std::vector<float> data(num_beams * kFieldsPerBeam);
    for (int i = 0; i < num_beams; ++i) {
      const BathymetryBeam& beam = msg->beams[i];

      const int idx_offset = i * kFieldsPerBeam;
      data[idx_offset] = 0.0;                                            // x.
      data[idx_offset + 1] = beam.range * std::sin(-beam.angle);         // y.
      data[idx_offset + 2] = -1 * (beam.range * std::cos(-beam.angle));  // z.
      data[idx_offset + 3] = beam.intensity;
    }

    // Set data layout details.
    pcd_msg.is_bigendian = false;
    pcd_msg.point_step = 16;
    pcd_msg.row_step = 16 * num_beams;
    pcd_msg.is_dense = false;

    // Set the binary blob.
    const size_t byte_size = data.size() * sizeof(float);
    pcd_msg.data.resize(byte_size);
    std::memcpy(pcd_msg.data.data(), data.data(), byte_size);

    // Publish.
    pub_pcd_->publish(pcd_msg);
    RCLCPP_INFO(this->get_logger(),
                "[BathymetryParser] Published point cloud (N=%d) at t=%f",
                num_beams, header.stamp.sec + header.stamp.nanosec * 1e-9);
  }

  rclcpp::Subscription<Bathymetry>::SharedPtr sub_bathymetry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BathymetryParser>());
  rclcpp::shutdown();
  return 0;
}
