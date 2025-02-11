#include "rehab_trajectory_planner/rehab_trajectory_planner.hpp"
#include <fstream>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <rclcpp/logging.hpp>
#include <sisl.h>
#include <vector>
#include <cstdlib>


RehabTrajectoryPlanner::RehabTrajectoryPlanner()
    : Node("rehab_trajectory_planner_node")
{
    start_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/start_point", 10, std::bind(&RehabTrajectoryPlanner::startPointCallback, this, std::placeholders::_1));

    end_point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/end_point", 10, std::bind(&RehabTrajectoryPlanner::endPointCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("move_group_name", "fmrrehab");

    // Initialize the separate node and executor
    move_group_node_ = std::make_shared<rclcpp::Node>("move_group_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(move_group_node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&RehabTrajectoryPlanner::initializeMoveGroup, this));
}

void RehabTrajectoryPlanner::initializeMoveGroup()
{
    if (!move_group_defined)
    {
        this->get_parameter("move_group_name", move_group_name);

        moveit::planning_interface::MoveGroupInterface::Options options(
            move_group_name, "robot_description", move_group_node_->get_namespace());
        move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(move_group_node_, options);

        move_group_defined = true;
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized with group: %s", move_group_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());

        auto starting_pose = move_group_interface->getCurrentPose();
    }
}

void RehabTrajectoryPlanner::startPointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    start_point_ = *msg;
    received_start_ = true;
    if (received_start_ && received_end_)
    {
        computeTrajectory();
        compute_trajectory = false;
    }
}

void RehabTrajectoryPlanner::computeTrajectory()
{
    if (compute_trajectory)
    {
        if (!move_group_defined)
        {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface is not yet initialized.");
            return;
        }

        const moveit::core::JointModelGroup* joint_model_group = move_group_interface->getCurrentState(5)->getJointModelGroup(move_group_name);

        // Set start state
        std::vector<std::vector<double>> start_joint_positions(3);
        start_joint_positions.at(0).push_back(start_point_.x);
        start_joint_positions.at(1).push_back(start_point_.y);
        start_joint_positions.at(2).push_back(start_point_.z);

        // auto start_state = move_group_interface->getCurrentState();
        moveit::core::RobotState start_state(move_group_interface->getRobotModel());
        for (int i=0; i<start_joint_positions.size(); i++){
            start_state.setJointPositions(move_group_interface->getJointNames().at(i), start_joint_positions.at(i));
        }
        // std::vector<double> starting_joint_positions;
        // start_state->copyJointGroupPositions(joint_model_group, starting_joint_positions);

        // Set target state
        std::vector<double> target_joint_positions;
        target_joint_positions.push_back(end_point_.x);
        target_joint_positions.push_back(end_point_.y);
        target_joint_positions.push_back(end_point_.z);

        move_group_interface->setStartState(start_state);
        // move_group_interface->setStartStateToCurrentState();
        move_group_interface->setMaxVelocityScalingFactor(1.0);
        move_group_interface->setMaxAccelerationScalingFactor(1.0);
        move_group_interface->setJointValueTarget(target_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(this->get_logger(), "Visualizing plan (joint space goal) info %s", success ? "" : "FAILED");

        RCLCPP_INFO(this->get_logger(), "starting joint positions: [%f, %f, %f]",
        my_plan.trajectory.joint_trajectory.points.front().positions[0],
        my_plan.trajectory.joint_trajectory.points.front().positions[1],
        my_plan.trajectory.joint_trajectory.points.front().positions[2]);

        int middle_idx = int(my_plan.trajectory.joint_trajectory.points.size()/2);
        RCLCPP_INFO(this->get_logger(), "joint velocities at middle: [%f, %f, %f]",
        my_plan.trajectory.joint_trajectory.points.at(middle_idx).velocities[0],
        my_plan.trajectory.joint_trajectory.points.at(middle_idx).velocities[1],
        my_plan.trajectory.joint_trajectory.points.at(middle_idx).velocities[2]);

        RCLCPP_INFO(this->get_logger(), "target joint positions: [%f, %f, %f]",
        my_plan.trajectory.joint_trajectory.points.back().positions[0],
        my_plan.trajectory.joint_trajectory.points.back().positions[1],
        my_plan.trajectory.joint_trajectory.points.back().positions[2]);

        double t_end = my_plan.trajectory.joint_trajectory.points.back().time_from_start.sec +
                my_plan.trajectory.joint_trajectory.points.back().time_from_start.nanosec * 1e-9; // Compute the total end time in seconds

            RCLCPP_INFO(this->get_logger(), "trajectory end time: %f", t_end);

        if (success)
        {
            int num_points = my_plan.trajectory.joint_trajectory.points.size();
            int num_joints = my_plan.trajectory.joint_trajectory.joint_names.size();

            RCLCPP_INFO(this->get_logger(), "number of points: %d", num_points);

            // Extract joint positions from the trajectory
            std::vector<std::vector<double>> joint_positions(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                const auto& point = my_plan.trajectory.joint_trajectory.points[i];
                joint_positions[i] = point.positions;
            }

            //---------------LINEAR--------------------------------
            auto trajectory_linear_iptp = my_plan.trajectory;
            robot_trajectory::RobotTrajectory rt(move_group_interface->getRobotModel(), move_group_name);
            rt.setRobotTrajectoryMsg(start_state, trajectory_linear_iptp);

            trajectory_processing::TimeOptimalTrajectoryGeneration totg;

            if (totg.computeTimeStamps(rt, 1.0, 1.0)){
                rt.getRobotTrajectoryMsg(trajectory_linear_iptp);
            }
            else RCLCPP_ERROR(this->get_logger(), "Error computing the time parametrization!");

            // logTrajectory(trajectory_linear_iptp, "linear");
            // computeTimeIntervals(num_points, num_joints, joint_positions, trajectory_linear_iptp);
            // logTrajectory(trajectory_linear_iptp, "linear_manual_ints");

            //---------------SISL------------------------------------
            auto sisl_trajectory = my_plan.trajectory;

            double cendpar;
            SISLCurve* initial_curve = 0;
            double* gpar = 0;
            int jnbpar;
            int jstat = 0;
            const double cstartpar = 0;
            double points_flattened[num_points*3];
            int p_types[num_points];
            int dimension = 3;

            int idx = 0;
            for (int i=0; i<num_points; i++){
                p_types[i] = 1;
                for(int j=0; j<num_joints; j++){
                points_flattened[idx] = sisl_trajectory.joint_trajectory.points.at(i).positions.at(j);
                idx++;
                }
            }

            s1356(points_flattened,        // pointer to where the point coordinates are stored
                  num_points,    // number of points to be interpolated
                  dimension,             // the dimension
                  p_types,          // what type of information is stored at a particular point
                  0,             // no additional condition at start point
                  0,             // no additional condition at end point
                  1,             // open curve
                  4,             // order of the spline curve to be produced
                  cstartpar,     // parameter value to be used at start of curve
                  &cendpar,      // parameter value at the end of the curve (to be determined)
                  &initial_curve, // the resulting spline curve (to be determined)
                  &gpar,         // pointer to the parameter values of the points in the curve (to be determined)
                  &jnbpar,       // number of unique parameter values (to be determined)
                  &jstat);       // status message

            if (jstat < 0) {
	        throw std::runtime_error("Error occured inside call to SISL routine.");
            } else if (jstat > 0) {
                std::cerr << "WARNING: warning occured inside call to SISL routine. \n" << std::endl;
            }
            
            printSISLCurveInfo(initial_curve);
            EvaluateAndLogSISL(initial_curve, sisl_trajectory, 6, "intial_sisl");

            SISLCurve* optimized_curve = OptimizeCurveSISL(initial_curve, 0.01, dimension);
            printSISLCurveInfo(optimized_curve);
            EvaluateAndLogSISL(optimized_curve, sisl_trajectory, 6, "optimized_sisl");

            //----------------GAUSSIAN-------------------------------
            auto trajectory_gaussian = my_plan.trajectory;

            std::vector<double> max_velocities; //get maximum joint velocities from moveit
            const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getJointModels();

            for (const moveit::core::JointModel* joint_model : joint_models)
            {
                if(joint_model->getType()!=5){
                    const moveit::core::VariableBounds& bounds = joint_model->getVariableBounds()[0]; // Assuming single variable joints
                    max_velocities.push_back(bounds.max_velocity_);
                }
            }

            RCLCPP_INFO(this->get_logger(), "max velocities (limits): [%f, %f, %f]", max_velocities[0], max_velocities[1], max_velocities[2]);

            auto [velocity_profiles, time_intervals] = generateGaussianProfiles(num_points, max_velocities, t_end, joint_positions);

            // Adjust trajectory velocities
            for (int i = 0; i < num_points; ++i)
            {
                for (int j = 0; j < num_joints; ++j)
                {
                    trajectory_gaussian.joint_trajectory.points[i].velocities[j] = velocity_profiles[j][i];
                }
                int point_sec = int(time_intervals[i]);
                trajectory_gaussian.joint_trajectory.points[i].time_from_start = rclcpp::Duration(point_sec, int((time_intervals[i]- point_sec) * 1e9));
            }

            // logTrajectory(trajectory_gaussian, "gaussian");
        }
    }
}

void RehabTrajectoryPlanner::computeTimeIntervals(int num_points, int num_joints, std::vector<std::vector<double>> joint_positions, moveit_msgs::msg::RobotTrajectory & trajectory){
    std::vector<double> times_linear(num_points);
    std::vector<double> dist_linear(num_points);

    // Compute distances between consecutive waypoints
    for (int i = 0; i < num_points - 1; ++i) {
        dist_linear[i] = computeDistance(joint_positions[i], joint_positions[i + 1]);
    }

    // Calculate time intervals based on distances and velocities
    times_linear[0] = 0.0;
    for (int i = 1; i < num_points; ++i) {
        for (int j = 0; j < num_joints; ++j) {
            double lin_vel = trajectory.joint_trajectory.points[i].velocities[j];
            times_linear[i] = times_linear[i - 1] + dist_linear[i] / lin_vel;
            int point_sec = int(times_linear[i]);
            trajectory.joint_trajectory.points[i].time_from_start = rclcpp::Duration(point_sec, int((times_linear[i]- point_sec) * 1e9));
        }
    }
}

std::tuple<std::vector<std::vector<double>>, std::vector<double>> RehabTrajectoryPlanner::generateGaussianProfiles(
    int num_points, 
    std::vector<double> max_velocity, 
    double t_end, 
    const std::vector<std::vector<double>>& joint_positions
) {
    int num_joints = max_velocity.size();
    std::vector<std::vector<double>> profiles(num_joints, std::vector<double>(num_points));
    std::vector<double> cumulative_times(num_points);
    std::vector<double> distances(num_points - 1);
    double mean_time = t_end / 2.0;
    double std_dev = t_end / 3.0;
    double dt = t_end / (num_points - 1);

    // Generate Gaussian velocity profiles
    for (int j = 0; j < num_joints; ++j) {
        for (int i = 0; i < num_points; ++i) {
            double t = i * dt;
            double velocity = max_velocity[j] * exp(-0.5 * pow((t - mean_time) / std_dev, 2));
            profiles[j][i] = velocity;
        }
    }

    // Compute distances between consecutive waypoints
    for (int i = 0; i < num_points - 1; ++i) {
        distances[i] = computeDistance(joint_positions[i], joint_positions[i + 1]);
    }

    // Calculate time intervals based on distances and velocities
    cumulative_times[0] = 0.0;
    for (int i = 1; i < num_points; ++i) {
        for (int j = 0; j < num_joints; ++j) {
            cumulative_times[i] = cumulative_times[i - 1] + distances[i] / profiles[j][i];
        }
    }

    return {profiles, cumulative_times};
}

void RehabTrajectoryPlanner::endPointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    end_point_ = *msg;
    received_end_ = true;
    if (received_start_ && received_end_)
    {
        computeTrajectory();
        compute_trajectory = false;
    }
}

double RehabTrajectoryPlanner::computeDistance(const std::vector<double>& pos1, const std::vector<double>& pos2) {
    double distance = 0.0;
    for (size_t i = 0; i < pos1.size(); ++i) {
        distance += std::pow(pos1[i] - pos2[i], 2);
    }
    return std::sqrt(distance);
}

void RehabTrajectoryPlanner::printSISLCurveInfo(SISLCurve* curve, double* gpar, int jnbpar, double cstartpar, double cendpar) {
    std::cout << "gpar size: " << jnbpar << std::endl;
    std::cout << "Total parameter interval of curve: [" << cstartpar << ", " << cendpar << "]\n\n";

    std::cout << "Point parameter values (gpar): \n";
    for (int i = 0; i < jnbpar; ++i) {
        std::cout << gpar[i] << ' ';
    }
    std::cout << std::endl;

    std::cout << "idim: " << curve->idim << std::endl;
    std::cout << "ik: " << curve->ik << std::endl;
    std::cout << "in: " << curve->in << std::endl;

    std::cout << "et (knot vector): \n";
    for (int i = 0; i < curve->in + curve->ik; i++) {
        std::cout << curve->et[i] << std::endl;
    }

    std::cout << "ecoef (control points): \n";
    for (int i = 0; i < curve->in * curve->idim; i++) {
        std::cout << curve->ecoef[i] << std::endl;
    }
}

void RehabTrajectoryPlanner::printSISLCurveInfo(SISLCurve* curve) {
    std::cout << "idim: " << curve->idim << std::endl;
    std::cout << "ik: " << curve->ik << std::endl;
    std::cout << "in: " << curve->in << std::endl;

    std::cout << "et (knot vector): \n";
    for (int i = 0; i < curve->in + curve->ik; i++) {
        std::cout << curve->et[i] << std::endl;
    }

    std::cout << "ecoef (control points): \n";
    for (int i = 0; i < curve->in * curve->idim; i++) {
        std::cout << curve->ecoef[i] << std::endl;
    }
}

void RehabTrajectoryPlanner::EvaluateAndLogSISL(SISLCurve* curve, moveit_msgs::msg::RobotTrajectory sisl_trajectory, int dimension, std::string trajectory_log_label)
    {
    // Evaluate the spline at evenly spaced parameter values
    int num_eval_points = curve->in;
    const double cstartpar = curve->et[0];
    double cendpar = curve->et[curve->in];
    double step = (cendpar - cstartpar) / (num_eval_points - 1);
    int jstat = 0;

    std::cout << "Evaluate function - num_eval_points: " << num_eval_points << std::endl;
    std::cout << "Evaluate function - cstartpar: " << cstartpar << std::endl;
    std::cout << "Evaluate function - cendpar: " << cendpar << std::endl;
    

    for (int i = 0; i < num_eval_points; ++i) {
        double parvalue = cstartpar + i * step;
        if (parvalue > cendpar) parvalue = cendpar; // Ensure parvalue does not exceed cendpar

        double derive[dimension];
        int leftknot = 0;

        s1227(curve, 1, parvalue, &leftknot, derive, &jstat);

        if (jstat < 0) {
            throw std::runtime_error("Error occurred inside call to SISL routine s1227.");
        }

        for (int j = 0; j < dimension/2; ++j) {
            sisl_trajectory.joint_trajectory.points.at(i).positions.at(j) = derive[j];
            sisl_trajectory.joint_trajectory.points.at(i).velocities.at(j) = derive[j+dimension/2];
        }
    }

    logTrajectory(sisl_trajectory, trajectory_log_label);
}

// Function to compute distance between two points
double distance(double *p1, double *p2, int dim) {
    double sum = 0;
    for (int i = 0; i < dim; i++) {
        sum += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    }
    return sqrt(sum);
}

// Evaluate error between two curves at given parameter values
double RehabTrajectoryPlanner::EvaluateErrorSISL(SISLCurve *original, SISLCurve *optimized, double *params, int num_params, int dim) {
    double max_error = 0;
    int jstat = 0;
    int leftknot = 0;
    double original_point[dim];
    double optimized_point[dim];
    
    for (int i = 0; i < num_params; i++) {
        s1221(original, 0, params[i], &leftknot, original_point, &jstat);
        s1221(optimized, 0, params[i], &leftknot, optimized_point, &jstat);
        double err = distance(original_point, optimized_point, dim);
        if (err > max_error) {
            max_error = err;
        }
    }
    return max_error;
}

void sample_curve(SISLCurve *curve, std::vector<double>& sampled_points, int num_samples, int dim) {
    int jstat = 0;
    int leftknot = 0;
    double start_param = curve->et[curve->ik - 1];
    double end_param = curve->et[curve->in];

    sampled_points.resize(num_samples * dim);
    for (int i = 0; i < num_samples; ++i) {
        double param = start_param + i * (end_param - start_param) / (num_samples - 1);
        s1221(curve, 0, param, &leftknot, &sampled_points[i * dim], &jstat);
        if (jstat < 0) {
            throw std::runtime_error("Error occured inside call to SISL s1221 routine.");
            return;
        }
    }

    // Debug prints
    std::cout << "Sampled points:" << std::endl;
    for (int i = 0; i < num_samples; ++i) {
        for (int d = 0; d < dim; ++d) {
            std::cout << sampled_points[i * dim + d] << " ";
        }
        std::cout << std::endl;
    }
}


SISLCurve* fit_curve_to_points(const std::vector<double>& points, int num_points, int dim, int order, int open) {
    int jstat;
    double cendpar;
    double* gpar = 0;
    int jnbpar;
    SISLCurve *new_curve = 0;
    std::vector<int> point_types(num_points, 1); // All points are ordinary points

    s1356(const_cast<double*>(points.data()), num_points, dim, const_cast<int*>(point_types.data()), 0, 0, open, order, 0.0, &cendpar, &new_curve, &gpar, &jnbpar, &jstat);

    if (jstat < 0) {
        // Handle error
        throw std::runtime_error("Error occured inside call to SISL s1356 routine.");
        return nullptr;
    }

    // Debug print
    std::cout << "Fitted curve status: " << jstat << ", number of control points: " << new_curve->in << std::endl;

    // Ensure no memory leaks
    if (gpar) free(gpar);

    return new_curve;
}


SISLCurve* RehabTrajectoryPlanner::ReduceControlPoints(SISLCurve *curve, int new_num_control_points, int dim) {
    std::vector<double> sampled_points;
    sample_curve(curve, sampled_points, new_num_control_points, dim);

    SISLCurve *new_curve = fit_curve_to_points(sampled_points, new_num_control_points, dim, curve->ik, 1);

    if (new_curve->in != new_num_control_points) {
        std::cerr << "Failed to reduce control points. Expected: " << new_num_control_points << ", Got: " << new_curve->in << std::endl;
        freeCurve(new_curve);
        return nullptr; // Avoid throwing an exception to prevent termination
    }

    std::cout << "Reduced control points to: " << new_curve->in << std::endl;
    return new_curve;
}



SISLCurve* RehabTrajectoryPlanner::OptimizeCurveSISL(SISLCurve *initial_curve, double max_allowable_error, int dim) {
    SISLCurve *optimized_curve = copyCurve(initial_curve);
    int num_control_points = initial_curve->in;
    int previous_num_control_points = num_control_points + 1;
    int iteration_count = 0;
    const int max_iterations = 100;
    std::cout << "initial num control points: " << num_control_points << std::endl;

    while (num_control_points > 3 && num_control_points < previous_num_control_points && iteration_count < max_iterations) {
        previous_num_control_points = num_control_points;
        iteration_count++;

        SISLCurve *temp_curve = ReduceControlPoints(optimized_curve, num_control_points - 1, dim);

        if (!temp_curve) {
            std::cout << "Failed to reduce control points further. Stopping iteration." << std::endl;
            break;
        }

        int num_params = initial_curve->in;
        double params[num_params];
        for (int i = 0; i < num_params; ++i) {
            params[i] = i / static_cast<double>(num_params - 1);
        }

        double error = EvaluateErrorSISL(initial_curve, temp_curve, params, num_params, dim);

        if (error <= max_allowable_error) {
            freeCurve(optimized_curve);
            optimized_curve = temp_curve;
            num_control_points = optimized_curve->in;
            std::cout << "num control points: " << num_control_points << std::endl;
            std::cout << "max detected error: " << error << std::endl;
        } else {
            freeCurve(temp_curve);
            std::cout << "Error exceeded maximum allowable error." << std::endl;
            break;
        }
    }

    if (iteration_count == max_iterations) {
        std::cout << "Max iterations reached. Possible infinite loop detected." << std::endl;
    }

    return optimized_curve;
}



std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_c);
    
    std::ostringstream oss;
    oss << std::put_time(now_tm, "%Y-%m-%d-%H:%M:%S");

    return oss.str();
}

void RehabTrajectoryPlanner::logTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory, std::string trajectory_id)
{
    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "trajectory" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "positions" << YAML::Value << YAML::BeginSeq;
    for (const auto &point : trajectory.joint_trajectory.points) {
        out << YAML::Flow << point.positions;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "velocities" << YAML::Value << YAML::BeginSeq;
    for (const auto &point : trajectory.joint_trajectory.points) {
        out << YAML::Flow << point.velocities;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "time from start" << YAML::Value << YAML::BeginSeq;
    for (const auto &point : trajectory.joint_trajectory.points) {
        double times = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
        out << YAML::Flow << times;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap; // End trajectory map
    out << YAML::EndMap; // End root map

    std::string folder = "/home/adriano/projects/ros2_ws/src/rehab_trajectory_planner/config/";
    std::string filename = folder + "tlog_" + trajectory_id + "_" + getCurrentDateTime() + ".yaml";
    std::ofstream file(filename);
    RCLCPP_INFO(this->get_logger(), "Trajectory (%s) saved in yaml file!", filename.c_str());
    file << out.c_str();
}

void RehabTrajectoryPlanner::saveTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory, std::string trajectory_id)
{
    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "cart_trj3" << YAML::Value << YAML::BeginMap;
    
    out << YAML::Key << "MotionLaw" << YAML::Value << YAML::BeginSeq;
    out << YAML::Flow << 0.0;
    out << YAML::Flow << 0.0;
    out << YAML::Flow << 0.0;
    out << YAML::EndSeq;

    out << YAML::Key << "cart_positions" << YAML::Value << YAML::BeginSeq;
    for (const auto &point : trajectory.joint_trajectory.points) {
        out << YAML::Flow << point.positions;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "joint_names" << YAML::Value << YAML::BeginSeq;
    for (int i=0; i<trajectory.joint_trajectory.joint_names.size(); i++) {
        out << YAML::Flow << trajectory.joint_trajectory.joint_names.at(i);
    }
    out << YAML::EndSeq;

    out << YAML::Key << "time_from_start" << YAML::Value << YAML::BeginSeq;
    for (const auto &point : trajectory.joint_trajectory.points) {
        double times = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
        out << YAML::Flow << times;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap; // End trajectory map
    out << YAML::EndMap; // End root map

    std::string folder = "/home/adriano/projects/ros2_ws/src/rehab_gui/config/";
    std::string filename = folder + "movement_" + trajectory_id + "_" + getCurrentDateTime() + ".yaml";
    std::ofstream file(filename);
    RCLCPP_INFO(this->get_logger(), "Trajectory (%s) saved in yaml file!", filename.c_str());
    file << out.c_str();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<RehabTrajectoryPlanner>();
    rclcpp::spin(move_group_node);
    rclcpp::shutdown();
    return 0;
}
