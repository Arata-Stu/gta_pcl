#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <signal.h>

class PcdSaver : public rclcpp::Node
{
public:
    // コンストラクタにargcとargvを追加
    PcdSaver(int argc, char *argv[]) : Node("pcd_saver"), pcd_file_path_("/home/arata-22/study/GTA/data/pcd/final_cloud.pcd")
    {
        if (argc > 1) {
            pcd_file_path_ = argv[1]; // 実行時の第一引数をPCDファイルの保存先パスとして使用
        }
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "gta_pcl", 10,
            std::bind(&PcdSaver::pointCloudCallback, this, std::placeholders::_1));
    }

    ~PcdSaver()
    {
        saveCloudToFile();
    }

    void saveCloudToFile()
    {
        if (!cloud_->points.empty()) {
            // PCDファイルとして保存
            pcl::io::savePCDFileASCII(pcd_file_path_, *cloud_);
            RCLCPP_INFO(this->get_logger(), "Saved %zu points to %s", cloud_->size(), pcd_file_path_.c_str());
        }
    }

private:
    std::string pcd_file_path_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ROSメッセージをPCLのPointCloudに変換して蓄積
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);
        *cloud_ += temp_cloud; // 受け取ったクラウドを追加
    }
};

// Ctrl+Cなどで終了する際のハンドラ
void signalHandler(int signum)
{
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdSaver>(argc, argv); // 修正部分
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
