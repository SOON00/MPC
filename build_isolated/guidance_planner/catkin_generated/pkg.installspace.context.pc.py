# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;ros_tools;dynamic_reconfigure".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lguidance_planner;-lguidance_planner_homotopy;-lgsl;-lgslcblas;-lm".split(';') if "-lguidance_planner;-lguidance_planner_homotopy;-lgsl;-lgslcblas;-lm" != "" else []
PROJECT_NAME = "guidance_planner"
PROJECT_SPACE_DIR = "/home/soon/workspace/install_isolated"
PROJECT_VERSION = "0.0.0"
