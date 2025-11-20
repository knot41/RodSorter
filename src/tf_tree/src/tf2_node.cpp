#include "tf_tree/tf2_node.hpp"
#include <memory>
#include "check_utils.hpp"

TF2Node::TF2Node(const rclcpp::NodeOptions& options):
    Node("tf2_node", options) {
    this->tool2sucker_tran_ = declare_parameter(
        "tool2sucker_tran",
        std::vector<double> { 0.0, 0.0, 118.0 , 0.0 , 0.0, 0.0 }
    );
    this->tool2camera_tran_ = declare_parameter(
        "tool2camera_tran",
        std::vector<double> {-88.1, -52.2, 61.8 , 0.0 , 0.0, 0.0}
    );
    this->base_coordinate = declare_parameter("base_coordinate", "base");
    this->sucker_coordinate = declare_parameter("sucker_coordinate", "sucker");
    this->tool_coordinate = declare_parameter("tool_coordinate", "tool");
    this->camera_coordinate = declare_parameter("camera_coordinate", "camera");
    broadcaster_base2tool_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    broadcaster_tool2_sucker_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    broadcaster_tool2_camera_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tfs_base2tool_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    tfs_tool2sucker_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    tfs_tool2camera_ = std::make_unique<geometry_msgs::msg::TransformStamped>();

    this->TCP_status_sub_ = this->create_subscription<interfaces::msg::ToolCenterPose>(
        "/TCPStatus",
        rclcpp::SensorDataQoS(),
        std::bind(&TF2Node::TCPStatusCallback, this, std::placeholders::_1)
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!Check_is_Init(this->tool2sucker_tran_)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    if (!Check_is_Init(this->tool2camera_tran_)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    // RCLCPP_WARN(this->get_logger(), "tool2sucker_tran=%f, %f, %f", this->tool2sucker_tran_[0], this->tool2sucker_tran_[1], this->tool2sucker_tran_[2]);
    // RCLCPP_WARN(this->get_logger(), "tool2sucker_tran=%f, %f, %f", this->tool2sucker_tran_[3], this->tool2sucker_tran_[4], this->tool2sucker_tran_[5]);
    // RCLCPP_WARN(this->get_logger(), "tool2camera_tran=%f, %f, %f", this->tool2camera_tran_[0], this->tool2camera_tran_[1], this->tool2camera_tran_[2]);
}


//TODO: tf转换需更改
void TF2Node::TCPStatusCallback(const interfaces::msg::ToolCenterPose::SharedPtr msg) {
    auto& timestamp = msg->header.stamp;
    //发布base坐标系到tool坐标系的变换
    SendTransform(
        this->broadcaster_base2tool_,
        this->tfs_base2tool_,
        timestamp,
        this->base_coordinate,
        this->tool_coordinate,
        [msg]() {
            tf2::Quaternion q;
            q.setRPY(msg->tcp_pos.data[3]/180.0*M_PI, msg->tcp_pos.data[4]/180.0*M_PI, msg->tcp_pos.data[5]/180.0*M_PI);
            return q;
        }(),
        tf2::Vector3(msg->tcp_pos.data[0]/1000.0, msg->tcp_pos.data[1]/1000.0, msg->tcp_pos.data[2]/1000.0)
    );

    // RCLCPP_INFO(this->get_logger(), "发布ing...");

    //发布tool坐标系到sucker坐标系的变换
    SendTransform(
        this->broadcaster_tool2_sucker_,
        this->tfs_tool2sucker_,
        timestamp,
        this->tool_coordinate,
        this->sucker_coordinate,
        [msg, this]() {
            tf2::Quaternion q;
            q.setRPY(0,0,0);
            return q;
        }(),
        tf2::Vector3(this->tool2sucker_tran_[0]/1000.0, this->tool2sucker_tran_[1]/1000.0, this->tool2sucker_tran_[2]/1000.0)
    );

    //发布tool坐标系到camera坐标系的变换
    SendTransform(
        this->broadcaster_tool2_camera_,
        this->tfs_tool2camera_,
        timestamp,
        this->tool_coordinate,
        this->camera_coordinate,
        [msg, this]() {
            tf2::Quaternion q;
            q.setEuler(0, 0, 0);
            return q;
        }(),
        tf2::Vector3(this->tool2camera_tran_[0]/1000.0, this->tool2camera_tran_[1]/1000.0, this->tool2camera_tran_[2]/1000.0)
    );
}
void TF2Node::SendTransform(
    const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
    const std::unique_ptr<geometry_msgs::msg::TransformStamped>& tfs,
    const rclcpp::Time& timestamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const tf2::Quaternion& q,
    const tf2::Vector3& v
) {
    tfs->header.stamp = timestamp;
    tfs->header.frame_id = frame_id;
    tfs->child_frame_id = child_frame_id;
    tfs->transform.rotation.x = q.getX();
    tfs->transform.rotation.y = q.getY();
    tfs->transform.rotation.z = q.getZ();
    tfs->transform.rotation.w = q.getW();
    tfs->transform.translation.x = v.getX();
    tfs->transform.translation.y = v.getY();
    tfs->transform.translation.z = v.getZ();

    broadcaster->sendTransform(*tfs);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(TF2Node)