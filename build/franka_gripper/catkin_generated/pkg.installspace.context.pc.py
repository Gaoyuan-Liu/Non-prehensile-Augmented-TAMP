# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;message_runtime;control_msgs;actionlib;sensor_msgs;xmlrpcpp;actionlib_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_gripper;/usr/lib/libfranka.so.0.8.0".split(';') if "-lfranka_gripper;/usr/lib/libfranka.so.0.8.0" != "" else []
PROJECT_NAME = "franka_gripper"
PROJECT_SPACE_DIR = "/home/liu/panda_tamp/install"
PROJECT_VERSION = "0.9.0"
