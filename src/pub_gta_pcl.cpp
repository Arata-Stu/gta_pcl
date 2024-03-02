#include "rclcpp/rclcpp.hpp"
#include "gta_msg/msg/gta_pcl.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

class CsvPublisher : public rclcpp::Node
{
public:
    CsvPublisher() : Node("csv_publisher"), start_time_(std::chrono::steady_clock::now())
    {
        publisher_ = this->create_publisher<gta_msg::msg::GtaPcl>("gta_pcl", 10);

        // CSVファイルを読み込んでデータを準備
        loadCsv("/home/arata-22/ros_ws/src/fpstest.csv");

        // タイマーを使用せずに、別のスレッドでパブリッシュ処理を開始
        std::thread(&CsvPublisher::publishData, this).detach();
    }

private:
    struct DataPoint
    {
        double timestamp;
        double x, y, z;
        std::string entity_type;
    };

    std::vector<DataPoint> data_points_;
    rclcpp::Publisher<gta_msg::msg::GtaPcl>::SharedPtr publisher_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    void loadCsv(const std::string& file_path)
    {
        std::ifstream file(file_path);
        std::string line;
        std::getline(file, line); // ヘッダースキップ

        while (std::getline(file, line))
        {
            std::istringstream s(line);
            DataPoint data_point;
            std::getline(s, line, ',');
            data_point.timestamp = std::stod(line);

            std::getline(s, line, ',');
            data_point.x = std::stod(line);

            std::getline(s, line, ',');
            data_point.y = std::stod(line);

            std::getline(s, line, ',');
            data_point.z = std::stod(line);

            std::getline(s, line, ',');
            data_point.entity_type = line;

            data_points_.push_back(data_point);
        }
    }

    void publishData()
    {
        auto base_time = std::chrono::steady_clock::now();
        for (auto& data_point : data_points_)
        {
            auto target_time = base_time + std::chrono::milliseconds(static_cast<int>(data_point.timestamp * 1000));
            std::this_thread::sleep_until(target_time);

            gta_msg::msg::GtaPcl msg;
            msg.timestamp = data_point.timestamp;
            msg.x = data_point.x;
            msg.y = data_point.y;
            msg.z = data_point.z;
            msg.entity_type = data_point.entity_type;

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: timestamp = '%f', x = '%f', y = '%f', z = '%f', entity_type = '%s'",
                        msg.timestamp, msg.x, msg.y, msg.z, msg.entity_type.c_str());
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
