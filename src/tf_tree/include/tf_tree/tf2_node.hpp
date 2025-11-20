#include "interfaces/msg/tool_center_pose.hpp"
#include "rclcpp/node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include <atomic>
#include <sensor_msgs/msg/joint_state.hpp>

class TF2Node: public rclcpp::Node {
public:
    explicit TF2Node(const rclcpp::NodeOptions& options);

    void TCPStatusCallback(const interfaces::msg::ToolCenterPose::SharedPtr msg);

private:
    /**
         * @brief 发布坐标系转换，先平移再旋转
         * 
         * @param frame_id 当前坐标系
         * @param child_frame_id 子坐标
         * @param v 平移向量
         */
    void SendTransform(
        const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
        const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
        const rclcpp::Time& timestamp,
        const std::string& frame_id,
        const std::string& child_frame_id,
        const tf2::Quaternion& q,
        const tf2::Vector3& v
    );

    // 广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_base2tool_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_tool2_sucker_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_tool2_camera_;
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_base2tool_;
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_tool2sucker_;
    std::unique_ptr<geometry_msgs::msg::TransformStamped> tfs_tool2camera_;

    rclcpp::Subscription<interfaces::msg::ToolCenterPose>::SharedPtr TCP_status_sub_;
    std::vector<double> tool2sucker_tran_; // 枪口坐标系到相机坐标系的平移及旋转向量
    std::vector<double> tool2camera_tran_; // 枪口坐标系到相机坐标系的平移及旋转向量

    // 坐标系名称
    std::string base_coordinate,
        camera_coordinate,
        sucker_coordinate,
        tool_coordinate;
};
