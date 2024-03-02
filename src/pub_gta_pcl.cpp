#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

class CsvPublisher : public rclcpp::Node
{
public:
    CsvPublisher() : Node("csv_publisher")
    {
        this->declare_parameter<std::string>("csv_file_path", "/path/to/default.csv");
        this->declare_parameter<std::vector<std::string>>("ignore_entities", {});

        std::string csv_file_path;
        this->get_parameter("csv_file_path", csv_file_path);
        std::vector<std::string> ignore_entities;
        this->get_parameter("ignore_entities", ignore_entities);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("gta_pcl", 10);
        loadCsv(csv_file_path, ignore_entities);
        std::thread(&CsvPublisher::publishData, this).detach();
    }

private:
    struct DataPoint
    {
        float timestamp;
        float x, y, z;
        std::string entity;
    };

    std::vector<DataPoint> data_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // トリム（前後の空白を削除）するヘルパー関数
    std::string trim(const std::string& str) {
        auto start = str.find_first_not_of(" \t\r\n");
        auto end = str.find_last_not_of(" \t\r\n");

        if (start == std::string::npos || end == std::string::npos)
            return "";

        return str.substr(start, end - start + 1);
    }

    void loadCsv(const std::string& file_path, const std::vector<std::string>& ignore_entities)
{
    std::ifstream file(file_path);
    std::string line;
    std::getline(file, line); // ヘッダー行をスキップ

    while (std::getline(file, line))
    {
        std::istringstream s(line);
        std::string token;
        DataPoint data_point;

        // タイムスタンプを読み込む
        std::getline(s, token, ',');
        data_point.timestamp = std::stof(token);

        // x, y, z 座標を読み込む
        std::getline(s, token, ',');
        data_point.x = std::stof(token);
        std::getline(s, token, ',');
        data_point.y = std::stof(token);
        std::getline(s, token, ',');
        data_point.z = std::stof(token);

        // エンティティを読み込む
        std::getline(s, token, ',');
        data_point.entity = trim(token); // トリム関数を使用して不要な空白を削除

        // エンティティが無視リストに含まれていない場合のみデータポイントを追加
        if (std::find(ignore_entities.begin(), ignore_entities.end(), data_point.entity) == ignore_entities.end()) {
            data_points_.push_back(data_point);
        } else {
            //RCLCPP_INFO(this->get_logger(), "Ignoring DataPoint with entity: %s", data_point.entity.c_str());
        }
    }
}


    void publishData()
    {
        auto base_timestamp = data_points_.front().timestamp;
        auto base_time = std::chrono::steady_clock::now();
        size_t total_points = data_points_.size();
        size_t point_index = 0;

        for (auto& data_point : data_points_)
        {
            auto delay_duration = std::chrono::milliseconds(static_cast<int>((data_point.timestamp - base_timestamp) * 1000));
            auto target_time = base_time + delay_duration;
            std::this_thread::sleep_until(target_time);

            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.push_back(pcl::PointXYZ(data_point.x, data_point.y, data_point.z));

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(cloud, output);
            output.header.frame_id = "map";
            output.header.stamp = this->get_clock()->now();

            publisher_->publish(output);

            ++point_index;
            if (point_index % 100 == 0 || point_index == total_points)
            {
                float progress = (static_cast<float>(point_index) / total_points) * 100.0f;
                RCLCPP_INFO(this->get_logger(), "Progress: %.2f%% (%zu/%zu)", progress, point_index, total_points);
            }
        }

        RCLCPP_INFO(this->get_logger(), "CSV publishing completed.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CsvPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
