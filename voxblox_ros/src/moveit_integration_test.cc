#include "voxblox_ros/esdf_server.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_integration_test");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::EsdfServer node(nh, nh_private);

  ros::spin();
  return 0;
}
