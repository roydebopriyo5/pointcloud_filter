#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode()
    : Node("pointcloud_filter_node")
    {
        // declare parameters and values
        this->declare_parameter<double>("min_height", 0.0);
        this->declare_parameter<double>("max_height", 1.0);

        // get values
        this->get_parameter("min_height", min_height_);
        this->get_parameter("max_height", max_height_);

        RCLCPP_INFO(this->get_logger(), "min_height: %f", min_height_);
        RCLCPP_INFO(this->get_logger(), "max_height: %f", max_height_);

        // subscribing to one of the topics from lidar_data_0.mcap
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidars/bpearl_front_right", 10, 
            std::bind(&PointCloudFilterNode::pointcloud_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /lidars/bpearl_front_right");

        // publishing to new topic
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/filtered_pointcloud", 10
        );

        RCLCPP_INFO(this->get_logger(), "Publishing to /filtered_pointcloud");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pointcloud with width: %d and height: %d", msg->width, msg->height);

        // converting ROS PointCloud2 to PCL PointCloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);

        RCLCPP_INFO(this->get_logger(), "Converted to PCL PointCloud with %zu points", pcl_cloud->points.size());

        // creating a new pointcloud for filtered data
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // filtering points by height ('z' coordinate)
        for (const auto& point : pcl_cloud->points)
        {
            if (point.z >= min_height_ && point.z <= max_height_)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Filtered PCL PointCloud to %zu points", filtered_cloud->points.size());

        // converting back to ROS PointCloud2
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toPCLPointCloud2(*filtered_cloud, pcl_pc2);
        pcl_conversions::moveFromPCL(pcl_pc2, output_msg);

        // set header and publishing the filtered pointcloud
        output_msg.header = msg->header;
        pointcloud_pub_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Published filtered pointcloud");
    }

    double min_height_;
    double max_height_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

// initialising the ROS2 system and spins the node to keep it running
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}

