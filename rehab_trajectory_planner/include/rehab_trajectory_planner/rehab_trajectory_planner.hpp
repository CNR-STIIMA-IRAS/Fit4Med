#ifndef REHAB_TRAJECTORY_PLANNER_H
#define REHAB_TRAJECTORY_PLANNER_H

#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <yaml-cpp/yaml.h>
#include <tuple>
#include <thread>
#include <sisl.h>

class RehabTrajectoryPlanner : public rclcpp::Node
{
public:
    RehabTrajectoryPlanner();

private:
    void initializeMoveGroup();
    void startPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void endPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void computeTrajectory();
    void computeTimeIntervals(int num_points, int num_joints, std::vector<std::vector<double>> joint_positions, moveit_msgs::msg::RobotTrajectory & trajectory);
    void printSISLCurveInfo(SISLCurve* result_curve, double* gpar, int jnbpar, double cstartpar, double cendpar);
    void printSISLCurveInfo(SISLCurve* result_curve);
    double computeDistance(const std::vector<double>& pos1, const std::vector<double>& pos2);
    void EvaluateAndLogSISL(SISLCurve* result_curve, moveit_msgs::msg::RobotTrajectory sisl_trajectory, int dimension, std::string trajectory_log_label); 
    double EvaluateErrorSISL(SISLCurve *original, SISLCurve *optimized, double *params, int num_params, int dim);
    SISLCurve* ReduceControlPoints(SISLCurve *curve, int new_num_control_points, int dim);
    SISLCurve* OptimizeCurveSISL(SISLCurve *initial_curve, double max_allowable_error, int dim);
    void saveTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory, std::string trajectory_id);
    void logTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory, std::string trajectory_id);
    std::tuple<std::vector<std::vector<double>>, std::vector<double>> generateGaussianProfiles(
        int num_points, 
        std::vector<double> max_velocity, 
        double t_end,
        const std::vector<std::vector<double>>& joint_positions);
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr start_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr end_point_sub_;
    geometry_msgs::msg::Point start_point_;
    geometry_msgs::msg::Point end_point_;
    bool received_start_ = false;
    bool received_end_ = false;
    bool move_group_defined = false;
    bool compute_trajectory = true;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string move_group_name;

    // New members for the executor and separate node
    rclcpp::Node::SharedPtr move_group_node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
};

#endif // REHAB_TRAJECTORY_PLANNER_H
