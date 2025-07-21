#include <pluginlib/class_list_macros.h>
#include <mpc_planner_rosnavigation/custom_global_planner.h>
#include <cmath>
#include <limits>

using namespace global_planner;

CustomGlobalPlanner::CustomGlobalPlanner()
    : initialized_(false), costmap_ros_(nullptr), current_waypoint_idx_(0), waypoint_interval_(1.5) {}

CustomGlobalPlanner::CustomGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), costmap_ros_(nullptr), current_waypoint_idx_(0), waypoint_interval_(1.5) {
    initialize(name, costmap_ros);
}

void CustomGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) return;

    costmap_ros_ = costmap_ros;
    ros::NodeHandle nh("~/" + name);

    path_sub_ = nh.subscribe("/topology_path", 1, &CustomGlobalPlanner::pathCallback, this);

    navfn_planner_.reset(new navfn::NavfnROS());
    navfn_planner_->initialize(name + "_navfn", costmap_ros_);

    initialized_ = true;
}

void CustomGlobalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    external_plan_ = msg->poses;
    waypoints_.clear();
    current_waypoint_idx_ = 0;

    if (external_plan_.empty()) return;

    waypoints_.push_back(external_plan_.front());
    geometry_msgs::Pose last_wp = external_plan_.front().pose;

    for (const auto& pose : external_plan_) {
        double dx = pose.pose.position.x - last_wp.position.x;
        double dy = pose.pose.position.y - last_wp.position.y;
        if (std::hypot(dx, dy) >= waypoint_interval_) {
            waypoints_.push_back(pose);
            last_wp = pose.pose;
        }
    }

    if (waypoints_.back().pose.position != external_plan_.back().pose.position)
        waypoints_.push_back(external_plan_.back());
}

bool CustomGlobalPlanner::navfnFallback(const geometry_msgs::PoseStamped& start,
                                        const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan) {
    std::vector<geometry_msgs::PoseStamped> navfn_plan;
    bool success = navfn_planner_->makePlan(start, goal, navfn_plan);
    if (success && !navfn_plan.empty()) {
        plan = navfn_plan;
        return true;
    }
    //ROS_WARN("Navfn fallback failed");
    return false;
}

geometry_msgs::PoseStamped CustomGlobalPlanner::shiftGoalIfNecessary(
    const geometry_msgs::PoseStamped& goal) {

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;

    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
        ROS_WARN("Goal is out of costmap bounds.");
        return goal;
    }

    unsigned char cost = costmap->getCost(mx, my);
    if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return goal;  // Safe
    }

    // Try previous waypoints for safe shift
    //ROS_WARN("Goal is in high-cost area. Attempting to shift.");

    for (int i = static_cast<int>(waypoints_.size()) - 2; i >= 0; --i) {
        const auto& wp = waypoints_[i].pose.position;
        if (costmap->worldToMap(wp.x, wp.y, mx, my) &&
            costmap->getCost(mx, my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            geometry_msgs::PoseStamped shifted = goal;
            shifted.pose.position = wp;
            //ROS_INFO("Shifted goal to previous safe waypoint.");
            return shifted;
        }
    }

    ROS_WARN("No safe shift target found. Keeping original goal.");
    return goal;
}

bool CustomGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_ || waypoints_.empty()) {
        ROS_WARN("Planner not initialized or no waypoints.");
        return false;
    }

    costmap_ros_->updateMap();
    ros::spinOnce();

    geometry_msgs::PoseStamped target_wp = waypoints_[current_waypoint_idx_];
    double dx = target_wp.pose.position.x - start.pose.position.x;
    double dy = target_wp.pose.position.y - start.pose.position.y;
    double dist_to_waypoint = std::hypot(dx, dy);

    if (dist_to_waypoint < 1.0 && current_waypoint_idx_ < waypoints_.size() - 1) {
        current_waypoint_idx_++;
        target_wp = waypoints_[current_waypoint_idx_];
        ROS_INFO("Switching to next waypoint: %lu", current_waypoint_idx_);
    }

    // Shift goal if necessary
    geometry_msgs::PoseStamped safe_goal = shiftGoalIfNecessary(goal);

    std::vector<geometry_msgs::PoseStamped> partial_plan;
    if (!navfnFallback(start, target_wp, partial_plan)) return false;

    plan.clear();
    plan.insert(plan.end(), partial_plan.begin(), partial_plan.end());

    if (!plan.empty()) {
        plan.back() = safe_goal;  // Replace last point with shifted goal
    } else {
        plan.push_back(safe_goal);
    }

    return true;
}

PLUGINLIB_EXPORT_CLASS(global_planner::CustomGlobalPlanner, nav_core::BaseGlobalPlanner)
