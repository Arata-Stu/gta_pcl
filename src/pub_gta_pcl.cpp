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
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("gta_pcl", 10);

        // CSVファイルを読み込んでデータを準備
        loadCsv("/home/arata-22/ros_ws/src/fpstest.csv");

        // タイマーを使用せずに、別のスレッドでパブリッシュ処理を開始
        std::thread(&CsvPublisher::publishData, this).detach();
    }

private:
    struct DataPoint
    {
        float timestamp; // タイムスタンプを追加
        float x, y, z;
    };


    std::vector<DataPoint> data_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    void loadCsv(const std::string& file_path)
    {
        std::ifstream file(file_path);
        std::string line;
        std::getline(file, line); // ヘッダースキップ

        while (std::getline(file, line))
        {
            std::istringstream s(line);
            DataPoint data_point;

            // タイムスタンプを読み込む
            std::getline(s, line, ',');
            data_point.timestamp = std::stof(line);

            // x, y, z 座標を読み込む
            std::getline(s, line, ',');
            data_point.x = std::stof(line);
            std::getline(s, line, ',');
            data_point.y = std::stof(line);
            std::getline(s, line, ',');
            data_point.z = std::stof(line);

            data_points_.push_back(data_point);
        }
    }


    // CSVファイルの読み込みとデータポイントの処理部分は変更なし

    void publishData() {
    // 最初のデータポイントのタイムスタンプを基準とする
        auto base_timestamp = data_points_.front().timestamp;
        auto base_time = std::chrono::steady_clock::now();

        for (auto& data_point : data_points_) {
            // 各データポイントの相対タイムスタンプを計算
            auto delay_duration = std::chrono::milliseconds(static_cast<int>((data_point.timestamp - base_timestamp) * 1000));
            auto target_time = base_time + delay_duration;
            std::this_thread::sleep_until(target_time);

            // ポイントクラウドの生成とパブリッシュ
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.push_back(pcl::PointXYZ(data_point.x, data_point.y, data_point.z));

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(cloud, output);
            output.header.frame_id = "map";
            output.header.stamp = this->get_clock()->now();

            publisher_->publish(output);
            // RCLCPP_INFO は、適宜使用してください
        }
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CsvPublisher>());
    rclcpp::shutdown();
    return 0;
}
