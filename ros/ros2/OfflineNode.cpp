// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <chrono>
#include <memory>
#include <string>

#include "OfflineNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace genz_icp_ros {

OfflineNode::OfflineNode(const rclcpp::NodeOptions &options) : OdometryServer(options) {
    bag_path_ = declare_parameter<std::string>("bag_path", "");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "");

    if (bag_path_.empty()) {
        RCLCPP_FATAL(get_logger(), "Parameter 'bag_path' is required but not set");
        throw std::runtime_error("bag_path is required");
    }
    if (pointcloud_topic_.empty()) {
        RCLCPP_FATAL(get_logger(), "Parameter 'pointcloud_topic' is required but not set");
        throw std::runtime_error("pointcloud_topic is required");
    }

    // Defer ProcessBag into the executor loop so publishers are fully wired
    oneshot_timer_ = create_wall_timer(std::chrono::milliseconds(0), [this]() {
        oneshot_timer_->cancel();
        ProcessBag();
        RCLCPP_INFO(get_logger(), "Bag processing complete, shutting down");
        rclcpp::shutdown();
    });

    RCLCPP_INFO(get_logger(), "OfflineNode initialized, will process: %s", bag_path_.c_str());
}

void OfflineNode::ProcessBag() {
    rosbag2_cpp::Reader reader;
    reader.open(bag_path_);

    // Set topic filter
    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back(pointcloud_topic_);
    reader.set_filter(filter);

    // Verify the topic exists and is PointCloud2
    const auto topics = reader.get_all_topics_and_types();
    bool topic_found = false;
    for (const auto &topic : topics) {
        if (topic.name == pointcloud_topic_) {
            if (topic.type != "sensor_msgs/msg/PointCloud2") {
                RCLCPP_FATAL(get_logger(), "Topic '%s' has type '%s', expected PointCloud2",
                             pointcloud_topic_.c_str(), topic.type.c_str());
                return;
            }
            topic_found = true;
            break;
        }
    }
    if (!topic_found) {
        RCLCPP_FATAL(get_logger(), "Topic '%s' not found in bag", pointcloud_topic_.c_str());
        return;
    }

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    size_t frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (reader.has_next() && rclcpp::ok()) {
        auto bag_msg = reader.read_next();

        // Deserialize into PointCloud2
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
        serializer.deserialize_message(&serialized_msg, msg.get());

        RegisterFrame(msg);
        frame_count++;

        if (frame_count % 100 == 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed =
                std::chrono::duration<double>(now - start_time).count();
            double fps = static_cast<double>(frame_count) / elapsed;
            RCLCPP_INFO(get_logger(), "Processed %zu frames (%.1f fps)", frame_count, fps);
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    double total_elapsed = std::chrono::duration<double>(end_time - start_time).count();
    double avg_fps = (total_elapsed > 0.0) ? static_cast<double>(frame_count) / total_elapsed : 0.0;
    RCLCPP_INFO(get_logger(), "Finished: %zu frames in %.1f s (avg %.1f fps)", frame_count,
                total_elapsed, avg_fps);
}

}  // namespace genz_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(genz_icp_ros::OfflineNode)
