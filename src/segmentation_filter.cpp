#include "segmentation_filter/segmentation_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h> 

void PointCloudFilter::load_parameters()
{
    this->declare_parameter<std::string>("input_topic", "");
    m_input_topic = this->get_parameter("input_topic").get_value<std::string>();

    this->declare_parameter<std::string>("output_topic", "");
    m_output_topic = this->get_parameter("output_topic").get_value<std::string>();

    this->declare_parameter<bool>("intensity", false);
    m_intensity = this->get_parameter("intensity").get_value<bool>();

    this->declare_parameter<std::vector<std::string>>("vertical_zones", std::vector<std::string>{});
    auto vertical_zones = this->get_parameter("vertical_zones").as_string_array();

    try{
        
        for (const auto& zone_str : vertical_zones) {
            VerticalZone zone;
            std::sscanf(zone_str.c_str(), "start: %lf, end: %lf, downsample: %d", 
            &zone.start, &zone.end, &zone.downsample);
            m_vertical_zones.push_back(zone);
            RCLCPP_INFO(this->get_logger(), "Zone - Start: %f, End: %f, Downsample: %d", zone.start, zone.end, zone.downsample);
        }
    }catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Error parsing zone: %s", e.what());
    }
}

void PointCloudFilter::initialize()
{
    // Load parameters
    this->load_parameters();

    rclcpp::QoS qos_rel(rclcpp::KeepLast(1));
    qos_rel.reliable();

    // Initialize publishers and subscribers
    if (m_intensity) {
        m_input_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_input_topic, qos_rel,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pointcloud_callback<pcl::PointXYZI>(msg);
            });
    } else {
        m_input_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_input_topic, qos_rel,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pointcloud_callback<pcl::PointXYZ>(msg);
            });
    }

    m_output_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_output_topic, 10);
}

PointCloudFilter::PointCloudFilter() : Node("segmentation_filter_node") 
{
    this->initialize();
}

template<typename T>
void PointCloudFilter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
    pcl::fromROSMsg(*msg, *cloud);

    // Apply vertical zone downsampling if enabled
    if (!m_vertical_zones.empty()) {
        typename pcl::PointCloud<T>::Ptr filtered_cloud(new pcl::PointCloud<T>);
        for (const auto& zone : m_vertical_zones) {
            int counter = 0;
            for (const auto& point : cloud->points) {
                if (point.z >= zone.start && point.z < zone.end) {
                    if (counter++ % zone.downsample == 0) {
                        filtered_cloud->points.push_back(point);
                    }
                }
            }
        }
        
        cloud->points = filtered_cloud->points;
        cloud->width = filtered_cloud->points.size();

        /*
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = false;
        cloud = filtered_cloud;
        */
    }

    // Convert back to ROS message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = msg->header;

    m_output_pub->publish(output_msg);
}