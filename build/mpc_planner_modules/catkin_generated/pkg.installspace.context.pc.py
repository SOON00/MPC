# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "mpc_planner_types;mpc_planner_util;mpc_planner_solver;ros_tools;guidance_planner;decomp_util".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmpc_planner_modules".split(';') if "-lmpc_planner_modules" != "" else []
PROJECT_NAME = "mpc_planner_modules"
PROJECT_SPACE_DIR = "/workspace/install"
PROJECT_VERSION = "0.0.0"
