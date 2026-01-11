#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// 存储接收到的焊缝轨迹点
std::vector<geometry_msgs::msg::PoseStamped> trajectory_butt;

// 回调函数：接收焊缝位姿并转换单位（毫米->米）
void posebuttCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    geometry_msgs::msg::PoseStamped converted_pose;
    converted_pose.header.frame_id = "base_link";
    // 将位置从毫米转换为米
    converted_pose.pose.position.x = msg->position.x / 1000.0;
    converted_pose.pose.position.y = msg->position.y / 1000.0;
    converted_pose.pose.position.z = msg->position.z / 1000.0;
    converted_pose.pose.orientation = msg->orientation;
    
    trajectory_butt.push_back(converted_pose);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "接收到焊缝位姿 [%zu] - 位置(米): [%.3f, %.3f, %.3f]",
                trajectory_butt.size(),
                converted_pose.pose.position.x,
                converted_pose.pose.position.y,
                converted_pose.pose.position.z);
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fr3_weld_executing");

    // 创建订阅者 - 订阅焊缝位姿
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    auto pose_subscriber = node->create_subscription<geometry_msgs::msg::Pose>(
        "/butt_torch_pose", qos, posebuttCallback);
    
    RCLCPP_INFO(node->get_logger(), "已订阅 /butt_torch_pose 话题");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
    arm.setEndEffectorLink("welding_gun_footprint_link");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);


    //--------------------------------------------------------------------------------
    // std::vector<double> joints = {1.536, -1.851, 1.941, 0.592, 1.767, -1.502};

    std::vector<double> joints = {1.536, -1.984, 1.941, 0.658, 1.767, -1.502};
    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        arm.execute(plan1);
    }
    // 保持当前末端姿态，仅修改位置到目标点
    geometry_msgs::msg::Quaternion keep_orientation = arm.getCurrentPose().pose.orientation;

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.006;
    target_pose.pose.position.y = -0.4;
    target_pose.pose.position.z = 0.02;
    target_pose.pose.orientation = keep_orientation;
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2)
    {
        arm.execute(plan2);
    }

    // 等待接收焊缝轨迹点 - 如果1秒内没有新点到来，认为接收完成
    std::chrono::steady_clock::time_point last_update = std::chrono::steady_clock::now();
    while (trajectory_butt.empty() || 
           (std::chrono::steady_clock::now() - last_update) < std::chrono::milliseconds(1000))
    {
        size_t prev_size = trajectory_butt.size();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (trajectory_butt.size() > prev_size)
        {
            last_update = std::chrono::steady_clock::now();
            RCLCPP_INFO(node->get_logger(), "已接收 %zu 个焊缝轨迹点", trajectory_butt.size());
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "已接收 %zu 个焊缝轨迹点，开始笛卡尔路径规划", trajectory_butt.size());
    
    // 笛卡尔路径规划 - 使用焊缝轨迹点，保持当前姿态
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // 使用反向迭代器，因为工件坐标是从基座坐标系平移过来的，所以坐标反了，反向执行后更合理
    for (auto it = trajectory_butt.rbegin(); it != trajectory_butt.rend(); ++it)
    {
        geometry_msgs::msg::Pose waypoint;
        waypoint.position = it->pose.position;
        waypoint.orientation = keep_orientation;
        waypoints.push_back(waypoint);
    }
    
    arm.setMaxVelocityScalingFactor(0.1);  // 降速到10%
    arm.setMaxAccelerationScalingFactor(0.1);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);
    
    RCLCPP_INFO(node->get_logger(), "笛卡尔路径规划完成度: %.2f%%", fraction * 100.0);

    if (fraction > 0.8)  // 降低阈值到80%
    {
        RCLCPP_INFO(node->get_logger(), "开始执行笛卡尔轨迹...");
        arm.execute(trajectory);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "笛卡尔路径规划完成度过低(%.2f%%)，跳过执行", fraction * 100.0);
    }

    // 重用之前声明的 joints 变量

    std::vector<double> joints1 = {1.403, -2.083, 1.696, 0.460, 1.436, -0.908};
    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints1);
    arm.setMaxVelocityScalingFactor(1.0); // 恢复原速
    arm.setMaxAccelerationScalingFactor(1.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success3)
    {
        arm.execute(plan3);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}