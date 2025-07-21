#ifndef CUSTOM_GLOBAL_PLANNER_H_
#define CUSTOM_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <navfn/navfn_ros.h>

namespace global_planner {

class CustomGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    CustomGlobalPlanner();
    CustomGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    bool navfnFallback(const geometry_msgs::PoseStamped& start,
                       const geometry_msgs::PoseStamped& goal,
                       std::vector<geometry_msgs::PoseStamped>& plan);

    geometry_msgs::PoseStamped shiftGoalIfNecessary(const geometry_msgs::PoseStamped& goal);

    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::Subscriber path_sub_;
    std::vector<geometry_msgs::PoseStamped> external_plan_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    size_t current_waypoint_idx_;
    double waypoint_interval_;
    boost::shared_ptr<navfn::NavfnROS> navfn_planner_;
};

}  // namespace global_planner

#endif  // CUSTOM_GLOBAL_PLANNER_H_
