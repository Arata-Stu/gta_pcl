#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

class MapBuilderNode : public rclcpp::Node
{
public:
    MapBuilderNode() : Node("map_builder_node")
    {
        // PointCloud2トピックをサブスクライブ
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/gta_pcl", 10,
            std::bind(&MapBuilderNode::pointCloudCallback, this, std::placeholders::_1));

        // 生成したマップをパブリッシュするためのパブリッシャー
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/output_map", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // 3Dマップの生成
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*cloud_filtered);

        // 生成したマップをパブリッシュ
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = "map";
        output.header.stamp = this->get_clock()->now();
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapBuilderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
