#include <pluginlib/class_list_macros.h>
#include <mpc_planner_rosnavigation/custom_global_planner.h>
#include <navfn/navfn_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <limits>

using namespace global_planner;

CustomGlobalPlanner::CustomGlobalPlanner() : initialized_(false), costmap_ros_(nullptr) {}

CustomGlobalPlanner::CustomGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}

void CustomGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) return;

    costmap_ros_ = costmap_ros;
    ros::NodeHandle nh("~/" + name);

    path_sub_ = nh.subscribe("/my_global_path", 1, &CustomGlobalPlanner::pathCallback, this);

    navfn_planner_.reset(new navfn::NavfnROS());
    navfn_planner_->initialize(name + "_navfn", costmap_ros_);

    initialized_ = true;
}

void CustomGlobalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    external_plan_ = msg->poses;
}

// 장애물 확인 및 로봇까지의 최소 거리 계산
bool CustomGlobalPlanner::checkObstacleNearPath(const std::vector<geometry_msgs::PoseStamped>& path,
                                                double obstacle_threshold,
                                                double radius,
                                                const geometry_msgs::PoseStamped& robot_pose,
                                                double& min_obstacle_distance) {
    if (!costmap_ros_) return false;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    double resolution = costmap->getResolution();
    int radius_cells = static_cast<int>(radius / resolution);

    min_obstacle_distance = std::numeric_limits<double>::infinity();
    bool found_obstacle = false;

    for (const auto& pose : path) {
        unsigned int mx, my;
        if (!costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
            continue;

        int start_x = std::max(0, static_cast<int>(mx) - radius_cells);
        int end_x = std::min(static_cast<int>(costmap->getSizeInCellsX()) - 1, static_cast<int>(mx) + radius_cells);
        int start_y = std::max(0, static_cast<int>(my) - radius_cells);
        int end_y = std::min(static_cast<int>(costmap->getSizeInCellsY()) - 1, static_cast<int>(my) + radius_cells);

        for (int x = start_x; x <= end_x; ++x) {
            for (int y = start_y; y <= end_y; ++y) {
                unsigned char cost = costmap->getCost(x, y);
                if (cost >= obstacle_threshold) {
                    double wx, wy;
                    costmap->mapToWorld(x, y, wx, wy);
                    double dx = wx - robot_pose.pose.position.x;
                    double dy = wy - robot_pose.pose.position.y;
                    double dist = std::hypot(dx, dy);

                    min_obstacle_distance = std::min(min_obstacle_distance, dist);
                    found_obstacle = true;

                    ROS_INFO("Obstacle detected near path at (%d, %d), cost=%d", x, y, cost);
                }
            }
        }
    }

    return found_obstacle;
}

bool CustomGlobalPlanner::navfnFallback(const geometry_msgs::PoseStamped& start,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan) {
    std::vector<geometry_msgs::PoseStamped> navfn_plan;
    bool success = navfn_planner_->makePlan(start, goal, navfn_plan);
    if (success && !navfn_plan.empty()) {
        plan = navfn_plan;
        ROS_INFO("Navfn fallback plan generated");
        return true;
    }
    ROS_WARN("Navfn fallback failed");
    return false;
}

bool CustomGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("CustomGlobalPlanner has not been initialized.");
        return false;
    }

    costmap_ros_->updateMap();
    ros::spinOnce();

    if (external_plan_.empty()) {
        ROS_WARN("External topology plan is empty.");
        return false;
    }

    double obstacle_threshold = 254;
    double obstacle_check_radius = 0.4;
    double fallback_distance = 4.0;

    double min_obstacle_distance;
    bool obstacle_exists = checkObstacleNearPath(external_plan_, obstacle_threshold, obstacle_check_radius, start, min_obstacle_distance);

    ROS_INFO("Obstacle exists: %s", obstacle_exists ? "true" : "false");
    ROS_INFO("Minimum obstacle distance to robot: %.3f meters", min_obstacle_distance);

    static bool using_navfn = false;

    if (obstacle_exists && min_obstacle_distance <= fallback_distance) {
        ROS_WARN("Obstacle within %.2fm → switching to navfn", fallback_distance);
        using_navfn = true;
        return navfnFallback(start, goal, plan);
    }

    if (using_navfn && (!obstacle_exists || min_obstacle_distance > fallback_distance)) {
        ROS_INFO("Obstacle cleared beyond %.2fm → reverting to topology path", fallback_distance);
        using_navfn = false;
    }

    // 보간 수행
    plan.clear();
    double interpolation_resolution = 0.1;

    for (size_t i = 0; i < external_plan_.size() - 1; ++i) {
        const auto& p1 = external_plan_[i];
        const auto& p2 = external_plan_[i + 1];

        double dx = p2.pose.position.x - p1.pose.position.x;
        double dy = p2.pose.position.y - p1.pose.position.y;
        double distance = std::hypot(dx, dy);
        int steps = std::max(1, static_cast<int>(distance / interpolation_resolution));

        for (int s = 0; s < steps; ++s) {
            double t = static_cast<double>(s) / steps;

            geometry_msgs::PoseStamped interp;
            interp.header = p1.header;
            interp.pose.position.x = p1.pose.position.x + t * dx;
            interp.pose.position.y = p1.pose.position.y + t * dy;
            interp.pose.position.z = 0.0;
            interp.pose.orientation = p1.pose.orientation;

            plan.push_back(interp);
        }
    }

    plan.push_back(external_plan_.back());

    // 인터폴레이션된 경로 기준 장애물 재확인
    double min_obstacle_distance_interp;
    bool obstacle_in_interpolated = checkObstacleNearPath(plan, obstacle_threshold, obstacle_check_radius, start, min_obstacle_distance_interp);

    ROS_INFO("Obstacle in interpolated path: %s", obstacle_in_interpolated ? "true" : "false");
    ROS_INFO("Minimum obstacle distance in interpolated path: %.3f meters", min_obstacle_distance_interp);

    if (obstacle_in_interpolated && min_obstacle_distance_interp <= fallback_distance) {
        ROS_WARN("Obstacle in interpolated path within %.2fm → fallback to navfn", fallback_distance);
        using_navfn = true;
        return navfnFallback(start, goal, plan);
    }

    if (using_navfn && (!obstacle_in_interpolated || min_obstacle_distance_interp > fallback_distance)) {
        ROS_INFO("Obstacle cleared beyond %.2fm → reverting to topology path", fallback_distance);
        using_navfn = false;
    }

    return true;
}

PLUGINLIB_EXPORT_CLASS(global_planner::CustomGlobalPlanner, nav_core::BaseGlobalPlanner)

