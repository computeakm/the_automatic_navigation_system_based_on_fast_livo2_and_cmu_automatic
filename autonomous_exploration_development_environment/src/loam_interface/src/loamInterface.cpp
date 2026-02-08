#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
nav_msgs::msg::Odometry odomData;
tf2::Stamped<tf2::Transform> odomTrans;
geometry_msgs::msg::TransformStamped transformTfGeom ; 

class LoamInterface : public rclcpp::Node
{
public:
  LoamInterface()
  : Node("loamInterface")
  {
    // Declare Parameters
    this->declare_parameter<std::string>("stateEstimationTopic", stateEstimationTopic);
    this->declare_parameter<std::string>("registeredScanTopic", registeredScanTopic);
    this->declare_parameter<bool>("flipStateEstimation", flipStateEstimation);
    this->declare_parameter<bool>("flipRegisteredScan", flipRegisteredScan);
    this->declare_parameter<bool>("sendTF", sendTF);
    this->declare_parameter<bool>("reverseTF", reverseTF);

    // Initialize Parameters
    this->get_parameter("stateEstimationTopic", stateEstimationTopic);
    this->get_parameter("registeredScanTopic", registeredScanTopic);
    this->get_parameter("flipStateEstimation", flipStateEstimation);
    this->get_parameter("flipRegisteredScan", flipRegisteredScan);
    this->get_parameter("sendTF", sendTF);
    this->get_parameter("reverseTF", reverseTF);

    tfBroadcasterPointer = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 5);
    pubOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);

    subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      registeredScanTopic, 5, std::bind(&LoamInterface::laserCloudHandler, this, std::placeholders::_1));
    subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      stateEstimationTopic, 5, std::bind(&LoamInterface::odometryHandler, this, std::placeholders::_1));

  }

private:

/**
 * @brief 处理激光点云数据的回调函数
 * @param laserCloudIn 接收到的激光点云消息的共享指针
 * @note 该函数负责将接收到的点云数据进行处理并发布
 */
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn) const
  {
    // 清空激光点云容器
    laserCloud->clear();
    // 将ROS消息格式的点云数据转换为PCL点云格式
    pcl::fromROSMsg(*laserCloudIn, *laserCloud);

    // 如果启用了翻转注册扫描的选项
    if (flipRegisteredScan) {
        // 获取点云数据的大小
      int laserCloudSize = laserCloud->points.size();
        // 遍历点云中的每个点
      for (int i = 0; i < laserCloudSize; i++) {
            // 交换点的x和z坐标，然后交换z和y坐标
        float temp = laserCloud->points[i].x;
        laserCloud->points[i].x = laserCloud->points[i].z;
        laserCloud->points[i].z = laserCloud->points[i].y;
        laserCloud->points[i].y = temp;
      }
    }

    // publish registered scan messages
    sensor_msgs::msg::PointCloud2 laserCloud2;
    pcl::toROSMsg(*laserCloud, laserCloud2);
    laserCloud2.header.stamp = laserCloudIn->header.stamp;
    laserCloud2.header.frame_id = "map";
    pubLaserCloud->publish(laserCloud2);
  }

/**
 * @brief 处理里程计消息的回调函数
 * @param odom 接收到的里程计消息指针
 * 该函数处理里程计数据，可选择进行坐标变换，并发布里程计消息和TF变换
 */
  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom) const
  {
    double roll, pitch, yaw;  // 存储欧拉角变量
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;  // 获取四元数
    odomData = *odom;  // 复制里程计数据

    // 如果启用了状态估计翻转
    if (flipStateEstimation) {
        // 将四元数转换为旋转矩阵，然后获取欧拉角
      tf2::Matrix3x3(tf2::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        // 翻转俯仰角和偏航角
      pitch = -pitch;
      yaw = -yaw;

        // 将欧拉角转换回四元数
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(roll, pitch, yaw);
      tf2::convert(quat_tf, geoQuat);

        // 更新里程计数据的位置和方向
      odomData.pose.pose.orientation = geoQuat;
      odomData.pose.pose.position.x = odom->pose.pose.position.z;
      odomData.pose.pose.position.y = odom->pose.pose.position.x;
      odomData.pose.pose.position.z = odom->pose.pose.position.y;
    }

    // publish odometry messages
    odomData.header.frame_id = "map";
    odomData.child_frame_id = "sensor";
    pubOdometry->publish(odomData);

    // publish tf messages
    odomTrans.frame_id_ = "map";
    odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf2::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

    if (sendTF) {
      if (!reverseTF) {
        transformTfGeom = tf2::toMsg(odomTrans);
        transformTfGeom.child_frame_id = "sensor";
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      } 
      else{
        transformTfGeom.transform = tf2::toMsg(odomTrans.inverse());
        transformTfGeom.header.frame_id = "sensor";
        transformTfGeom.child_frame_id = "map";
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPointer;

  const double PI = 3.1415926;

  string stateEstimationTopic = "/integrated_to_init";
  string registeredScanTopic = "/velodyne_cloud_registered";
  bool flipStateEstimation = true;
  bool flipRegisteredScan = true;
  bool sendTF = true;
  bool reverseTF = false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoamInterface>());
  rclcpp::shutdown();
  
  return 0;
}
